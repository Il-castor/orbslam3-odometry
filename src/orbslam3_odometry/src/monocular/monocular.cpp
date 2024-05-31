#include "monocular.hpp"

#include <opencv2/core/core.hpp>

// If defined, the pointcloud created by orbslam will be published
#define PUBLISH_POINT_CLOUD

using std::placeholders::_1;

void MonocularSlamNode::loadParameters()
{
    /* ***** DECLARING PARAMETERS ***** */

    declare_parameter("topic_camera_left", "/camera/left_image");
    declare_parameter("topic_camera_right", "/camera/right_image");
    declare_parameter("topic_imu", "/imu/data");
    declare_parameter("topic_orbslam_odometry", "/Odometry/orbSlamOdom");
    declare_parameter("topic_header_frame_id", "os_track");
    declare_parameter("topic_child_frame_id", "orbslam3");
    declare_parameter("is_camera_left", true);
    declare_parameter("scale_position_mono", 1);
    declare_parameter("degree_move_pose_mono", 0);
    declare_parameter("cutting_x", -1);
    declare_parameter("cutting_y", 0);
    declare_parameter("cutting_width", 0);
    declare_parameter("cutting_height", 0);

    /* ******************************** */

    /* ***** READING PARAMETERS ***** */

    get_parameter("topic_camera_left", this->camera_left);
    get_parameter("topic_camera_right", this->camera_right);
    get_parameter("topic_imu", this->imu);
    get_parameter("topic_orbslam_odometry", this->topic_pub_quat);
    get_parameter("topic_header_frame_id", this->header_id_frame);
    get_parameter("topic_child_frame_id", this->child_id_frame);
    get_parameter("is_camera_left", this->isCameraLeft);
    get_parameter("scale_position_mono", this->scale_position_mono);
    get_parameter("degree_move_pose_mono", this->degree_move_pose_mono);
    get_parameter("cutting_x", this->cutting_x);
    get_parameter("cutting_y", this->cutting_y);
    get_parameter("cutting_width", this->cutting_width);
    get_parameter("cutting_height", this->cutting_height);

}

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("orbslam3_odometry")
{
    this->loadParameters();

    // Compute cutting rect
    if (cutting_x != -1)
        cutting_rect = cv::Rect(cutting_x, cutting_y, cutting_width, cutting_height);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    m_SLAM = pSLAM;

    if (this->isCameraLeft)
    {
        m_image_subscriber = this->create_subscription<ImageMsg>(
            this->camera_left,
            qos,
            std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
            
        // The provided degree must become negative if the tracked camera is the left
        this->degree_move_pose_mono *= -1;

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN MONOCULAR MODE. NODE WILL WAIT FOR IMAGES IN TOPIC %s", this->camera_left.c_str());
    }
    else
    {
        m_image_subscriber = this->create_subscription<ImageMsg>(
            this->camera_right,
            qos,
            std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN MONOCULAR MODE. NODE WILL WAIT FOR IMAGES IN TOPIC %s", this->camera_right.c_str());
    }

    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_pub_quat, 10);

    #ifdef PUBLISH_POINT_CLOUD
	    publisherPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/pointCloud", 10);
	#endif

    Utility::printCommonInfo(qos);
    // std::cout << "End Costructor" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    // Save camera and keyframe trajectory
    RCLCPP_INFO(this->get_logger(), "Exit and saving... ");

    m_SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");

    RCLCPP_INFO(this->get_logger(), "Saved KeyFrameTrajectory.txt ");
    RCLCPP_INFO(this->get_logger(), "Saved CameraTrajectory.txt ");
}

/**
 * Function that returns a string with quaternion, each value is separated by a space.
 */
static std::string quaternionToString(const Eigen::Quaternionf &q)
{
    std::stringstream ss;
    ss << setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    return ss.str();
}

/**
 * Image callback
*/
void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Sono nella callback");
    
    // Inizial time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Copy the ros image message to cv::Mat.
    try
    {
        // m_cvImPtr = cv_bridge::toCvShare(msg, "bgr8");  // For image
        m_cvImPtr = cv_bridge::toCvCopy(msg, "bgr8");      // For compressed images

        // m_cvImPtr = cv_bridge::toCvCopy(msg); // Prima c'era questo
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    std::cout << "one frame has been sent" << std::endl;

    // Perform changes to image here
    cv::Mat image_for_orbslam = m_cvImPtr->image;
    if (cutting_x != -1)
        Utility::cutting_image(image_for_orbslam, cutting_rect);
        
    // Call ORB-SLAM3 with provided image
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(image_for_orbslam, Utility::StampToSec(msg->header.stamp));

    // Obtain the position and the orientation
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // String containing the quaternion
    std::string messaggio_quaternion = quaternionToString(q);

    // I publish position and quaternion (rotated)
    auto message = nav_msgs::msg::Odometry();
    geometry_msgs::msg::Pose output_pose{};

    // Multipling is necessary in the monocular, since there is no depth info
    output_pose.position.x = twc.z() * this->scale_position_mono;   
    output_pose.position.y = -twc.x() * this->scale_position_mono;
    output_pose.position.z = 0;

    output_pose.orientation.x = -q.z();
    output_pose.orientation.y = -q.x();
    output_pose.orientation.z = -q.y();
    output_pose.orientation.w = q.w();

    if (this->degree_move_pose_mono != 0){
        // If required, I move the yaw of specified degrees
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(output_pose.orientation, tf2_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(tf2_quat);
        m.getRPY(roll, pitch, yaw);
        
        tf2_quat.setRPY(0, 0, yaw+(this->degree_move_pose_mono * (M_PI / 180.0))); 
        output_pose.orientation = tf2::toMsg(tf2_quat);
    }

    message.pose.pose = output_pose;
    message.header.frame_id = header_id_frame;
    message.child_frame_id = child_id_frame;
    
    // add timestamp to message
    message.header.stamp = this->now();

    quaternion_pub->publish(message);

    // "End" time and saving times. File with times:
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tempo = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
    m_SLAM->InsertTrackTime(tempo);


    #ifdef PUBLISH_POINT_CLOUD
		// Point cloud pubblication
		RCLCPP_INFO(this->get_logger(), "prima del mappiont to pointcloud");
		// depths.clear();
		sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud( m_SLAM->GetTrackedMapPoints(), message.header.stamp, twc);
		publisherPointCloud->publish(cloud);
		RCLCPP_INFO(this->get_logger(), "dopo il mappiont to pointcloud");
    
    	// Tracked points showing is currently disabled...
		std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();
		std::cout << "Size key point: " << keypoints.size() << std::endl;
		
		// Remove this code to enable it 
		float minDepth = *std::min_element(depths.begin(), depths.end());
		float maxDepth = *std::max_element(depths.begin(), depths.end());

		for (size_t i = 0; i < keypoints.size(); ++i) {
		   cv::Point2f point = keypoints[i].pt;
		//    float depth = depths[i];
		//    cv::Scalar color = interpolateColor(depth, minDepth, maxDepth);
            cv::Vec3b color(0, 0, 255);
		   cv::circle(image_for_orbslam, point, 3, color, cv::FILLED);
		}
		
		
		
		cv::imshow("Keypoints", image_for_orbslam);
		cv::waitKey(1);
	#endif
}



// Key point color interpolation
cv::Scalar MonocularSlamNode::interpolateColor(float value, float minDepth, float maxDepth) { 
	// Doesn't working
    float range = maxDepth - minDepth;
    float normalized = (value - minDepth) / range;

    // Cold color (blue)
    cv::Vec3b coldColor(255, 0, 0);
    // Warm color (red)
    cv::Vec3b warmColor(0, 0, 255);

    cv::Vec3b color = (1 - normalized) * coldColor + normalized * warmColor;
    return cv::Scalar(color[0], color[1], color[2]);
}

// Converter from MapPoint to Point Cloud  
sensor_msgs::msg::PointCloud2 MonocularSlamNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time, Eigen::Vector3f actualPosition) {

    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = "velodyne";	// So it can be shown with lidar's velodyne
    cloud.height = 1;
    cloud.width = map_points.size();
    std::cout << "Size map point: " << map_points.size() << std::endl;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            
            //map_points[i]->SetWorldPos(vectorWorldPos );
            //Eigen::Vector3f tmp = map_points[i]->GetWorldPos();
            //std::cout << "Vector world pos: " << tmp.x() << " " << tmp.y() << " " << tmp.z() << std::endl;  
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            tf2::Vector3 point_translation(P3Dw.x()-actualPosition.x(), P3Dw.y()-actualPosition.y(), P3Dw.z()-actualPosition.z());

            float data_array[num_channels] = {
                point_translation.z()*this->scale_position_mono,
                -point_translation.x()*this->scale_position_mono,
                -point_translation.y()*this->scale_position_mono
            };
            
            depths.push_back(data_array[0]);

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}


