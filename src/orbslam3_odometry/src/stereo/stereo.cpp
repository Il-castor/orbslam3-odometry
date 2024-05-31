#include "stereo.hpp"

#include <opencv2/core/core.hpp>
#include <chrono>

// If defined the node will print debug information and will show disparity map
//#define DEBUG

// If defined, it will pre-rectify images, before giving it in input to orbslam.
// It should be enables for Basler and disabled for stereo-camera like Zed
#define PRE_RECTIFY_IMAGES

// If defined, the pointcloud created by orbslam will be published
//#define PUBLISH_POINT_CLOUD

// To have debug prints, comment this line:
#define RCLCPP_INFO(...) (void)0


using std::placeholders::_1;
using std::placeholders::_2;

void StereoSlamNode::loadParameters()
{
    /* ***** DECLARING PARAMETERS ***** */

    declare_parameter("topic_camera_left", "/camera/left_image");
    declare_parameter("topic_camera_right", "/camera/right_image");
    declare_parameter("topic_orbslam_odometry", "/Odometry/orbSlamOdom");
    declare_parameter("topic_header_frame_id", "os_track");
    declare_parameter("topic_child_frame_id", "orbslam3");
    declare_parameter("cutting_x", -1);
    declare_parameter("cutting_y", 0);
    declare_parameter("cutting_width", 0);
    declare_parameter("cutting_height", 0);


    /* ******************************** */

    /* ***** READING PARAMETERS ***** */

    get_parameter("topic_camera_left", this->camera_left);
    get_parameter("topic_camera_right", this->camera_right);
    get_parameter("topic_orbslam_odometry", this->topic_pub_quat);
    get_parameter("topic_header_frame_id", this->header_id_frame);
    get_parameter("topic_child_frame_id", this->child_id_frame);
    get_parameter("cutting_x", this->cutting_x);
    get_parameter("cutting_y", this->cutting_y);
    get_parameter("cutting_width", this->cutting_width);
    get_parameter("cutting_height", this->cutting_height);

}

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System *pSLAM, const string &strSettingsFile)
    : Node("orbslam3_odometry"),
      m_SLAM(pSLAM)
{
    // Load parameters
    this->loadParameters();

    // Read stereo-rectification parameters
    #ifdef PRE_RECTIFY_IMAGES
    	RCLCPP_INFO(this->get_logger(), "Reading stereo parameters for pre-rectification");
        readParameters(strSettingsFile, map1_L, map2_L, roi_L, map1_R, map2_R, roi_R) ;
        common_roi = roi_L & roi_R;
    #endif

    // Compute cutting rect
    if (cutting_x != -1)
        cutting_rect = cv::Rect(cutting_x, cutting_y, cutting_width, cutting_height);

#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "Topic camera left: %s", this->camera_left.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic camera right: %s", this->camera_right.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic imu: %s", this->imu.c_str());
    RCLCPP_INFO(this->get_logger(), "header_id_frame: %s", this->header_id_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "child_id_frame: %s", this->child_id_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "topic_orbslam_odometry: %s", this->topic_pub_quat.c_str());
#endif

    // Images subscriptions and odomotry pubblication

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    subImgLeft_ = this->create_subscription<ImageMsg>(this->camera_left, qos, std::bind(&StereoSlamNode::leftCallback, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>(this->camera_right, qos, std::bind(&StereoSlamNode::rightCallback, this, _1));

    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_pub_quat, 10);

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN STEREO MODE. NODE WILL WAIT FOR IMAGES IN TOPICS %s and %s", this->camera_left.c_str(), this->camera_right.c_str());
    Utility::printCommonInfo(qos);
    
    contImageLeft = 0;
    contImageRight = 0;
    contTrackStereo = 0;
    firstTimeStampLeft = -1;
    
    #ifdef PUBLISH_POINT_CLOUD
	    publisherPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/pointCloud", 10);
	#endif

    // Starts orbslam3. This thread is used to syncronize the two images
    syncThread_ = new std::thread(&StereoSlamNode::SyncImg, this);
    
}

StereoSlamNode::~StereoSlamNode()
{
    RCLCPP_INFO(this->get_logger(), "Number of left images arrived:\t" + std::to_string(contImageLeft));
    RCLCPP_INFO(this->get_logger(), "Number of right images arrived:\t" + std::to_string(contImageRight));
    RCLCPP_INFO(this->get_logger(), "Number of TrackStereo calls:\t" + std::to_string(contTrackStereo));
    RCLCPP_INFO(this->get_logger(), "First timestamp of left image:\t" + std::to_string(firstTimeStampLeft));
    RCLCPP_INFO(this->get_logger(), "Last timestamp of left image:\t" + std::to_string(lastTimeStampLeft));
    
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera and keyframe trajectory
    RCLCPP_INFO(this->get_logger(), "Exit and saving... ");
    m_SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Saved KeyFrameTrajectory.txt ");
    RCLCPP_INFO(this->get_logger(), "Saved CameraTrajectory.txt ");
}


/**
 *  Function that saves left image in variable left_image_
 */
void StereoSlamNode::leftCallback(const ImageMsg::SharedPtr msg)
{
    contImageLeft ++;
    bufMutexLeft_.lock();
    try
    {
	

	double t =  msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);
	if (firstTimeStampLeft == -1)
		firstTimeStampLeft = t;
	lastTimeStampLeft = t;
	
        RCLCPP_INFO(this->get_logger(), "left" );
        left_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;  
        
        tImLeft = t;

        timestamp = Utility::StampToSec(msg->header.stamp);

        // cv::imshow("Left NON Rectified", left_image_);
        // RCLCPP_INFO(this->get_logger(), "left ok" );
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        // return;
    }
    bufMutexLeft_.unlock();
}

/**
 *  Function that saves right image in variable right_image_
 */
void StereoSlamNode::rightCallback(const ImageMsg::SharedPtr msg)
{
    bufMutexRight_.lock();
    try
    {
        contImageRight ++;
        RCLCPP_INFO(this->get_logger(), "right" );
        right_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;    
        tImRight = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);
 

        // cv::imshow("Right NON Rectified", right_image_);

        // RCLCPP_INFO(this->get_logger(), "right ok" );
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        // return;
    }
    bufMutexRight_.unlock();
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

void StereoSlamNode::SyncImg()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft, imRight;
        if (!left_image_.empty() && !right_image_.empty())
        {
            /*if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                //std::cout << "big time difference" << std::endl;
                continue;
            }*/

            contTrackStereo ++;
            
            // Compute ORB-SLAM3 on the 2 grabbed images
            StereoSlamNode::GrabStereo(left_image_, right_image_);
            
            // Remove the images, so they won't be re-computated
            left_image_.release();
            right_image_.release();

        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}



void StereoSlamNode::GrabStereo(cv::Mat image_L, cv::Mat image_R)
{
    RCLCPP_INFO(this->get_logger(), "GrabStereo ");
    // Initial time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::Mat left_rectified_non_cropped, right_rectified_non_cropped;

	#ifdef PRE_RECTIFY_IMAGES
		// First of all we pre-rectify the images
		RCLCPP_INFO(this->get_logger(), "Prima del remap ");
		
		
		bufMutexLeft_.lock();
		rectify_image(image_L, map1_L, map2_L, common_roi, left_rectified_non_cropped);
		bufMutexLeft_.unlock();
		
		bufMutexRight_.lock();
		rectify_image(image_R, map1_R, map2_R, common_roi, right_rectified_non_cropped);
		bufMutexRight_.unlock();

		#ifdef DEBUG
			show_disparity(image_L, image_R);
		#endif
		RCLCPP_INFO(this->get_logger(), "Prima del cutting ");
		
		// If necessary, perform changes to the images here.
		if (cutting_x != -1){
		    Utility::cutting_image(left_rectified_non_cropped, cutting_rect);
		    Utility::cutting_image(right_rectified_non_cropped, cutting_rect);
		}
		RCLCPP_INFO(this->get_logger(), "Prima della track stereo ");
		
		// Call ORB-SLAM3 on the 2 pre-rectified images
		Sophus::SE3f Tcw = m_SLAM->TrackStereo(left_rectified_non_cropped, right_rectified_non_cropped, timestamp);
		RCLCPP_INFO(this->get_logger(), "dopo la track stereo ");
	#else

    
		// If necessary, perform changes to the images here.
		if (cutting_x != -1){
			bufMutexLeft_.lock();
		    Utility::cutting_image(image_L, cutting_rect);
		    bufMutexLeft_.unlock();
		    
		    bufMutexRight_.lock();
		    Utility::cutting_image(image_R, cutting_rect);
		    bufMutexRight_.unlock();
		}

		// Call ORB-SLAM3 on the 2 original images
		Sophus::SE3f Tcw = m_SLAM->TrackStereo(image_L, image_R, timestamp);
		
	#endif
     
    // Obtain the position and the quaternion
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // String containing the quaternion 
    std::string messaggio_quaternion = quaternionToString(q);

    // I publish position and quaternion (rotated)
    auto message = nav_msgs::msg::Odometry();
    geometry_msgs::msg::Pose output_pose{};

    output_pose.position.x = twc.z();
    output_pose.position.y = -twc.x();
    output_pose.position.z = 0;

    output_pose.orientation.x = -q.z();
    output_pose.orientation.y = -q.x();
    output_pose.orientation.z = -q.y();
    output_pose.orientation.w = q.w();
    
    
    // Set Roll and Pitch to Zero
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(output_pose.orientation, tf2_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(tf2_quat);
    m.getRPY(roll, pitch, yaw);
    tf2_quat.setRPY(0, 0, yaw); 
    output_pose.orientation = tf2::toMsg(tf2_quat);
    

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
		depths.clear();
		sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud( m_SLAM->GetTrackedMapPoints(), message.header.stamp, twc);
		publisherPointCloud->publish(cloud);
		RCLCPP_INFO(this->get_logger(), "dopo il mappiont to pointcloud");
    
    	// Tracked points showing is currently disabled...
		std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();
		std::cout << "Size key point: " << keypoints.size() << std::endl;
		
		/*// Remove this code to enable it 
		float minDepth = *std::min_element(depths.begin(), depths.end());
		float maxDepth = *std::max_element(depths.begin(), depths.end());

		for (size_t i = 0; i < keypoints.size(); ++i) {
		   cv::Point2f point = keypoints[i].pt;
		   float depth = depths[i];
		   cv::Scalar color = interpolateColor(depth, minDepth, maxDepth);
		   cv::circle(left_rectified_non_cropped, point, 3, color, cv::FILLED);
		}
		
		
		
		cv::imshow("Keypoints", left_rectified_non_cropped);
		cv::waitKey(1);*/
	#endif
}

// Key point color interpolation
cv::Scalar StereoSlamNode::interpolateColor(float value, float minDepth, float maxDepth) { 
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
sensor_msgs::msg::PointCloud2 StereoSlamNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time, Eigen::Vector3f actualPosition) {

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
                point_translation.z(),
                -point_translation.x(),
                -point_translation.y()
            };
            
            depths.push_back(data_array[0]);

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}


