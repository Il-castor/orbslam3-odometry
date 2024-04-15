#include "monocular.hpp"

#include <opencv2/core/core.hpp>

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

}

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("orbslam3_odometry")
{
    this->loadParameters();

    m_SLAM = pSLAM;

    if (this->isCameraLeft)
    {
        m_image_subscriber = this->create_subscription<ImageMsg>(
            this->camera_left,
            10,
            std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN MONOCULAR MODE. NODE WILL WAIT FOR IMAGES IN TOPIC %s", this->camera_left.c_str());
    }
    else
    {
        m_image_subscriber = this->create_subscription<ImageMsg>(
            this->camera_right,
            10,
            std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN MONOCULAR MODE. NODE WILL WAIT FOR IMAGES IN TOPIC %s", this->camera_right.c_str());
    }

    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_pub_quat, 10);

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

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw;
    std::cout << "one frame has been sent" << std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // String containing the quaternion
    std::string messaggio_quaternion = quaternionToString(q);

    // "filename" (in ASL format)
    //double timestamp = Utility::StampToSec(msg->header.stamp);

    // I publish timestamp and quaternion
    auto message = nav_msgs::msg::Odometry();
    geometry_msgs::msg::Pose output_pose{};

    output_pose.position.x = twc.z() * this->scale_position_mono;
    output_pose.position.y = -twc.x() * this->scale_position_mono;
    output_pose.position.z = 0;

    output_pose.orientation.x = -q.z();
    output_pose.orientation.y = -q.x();
    output_pose.orientation.z = -q.y();
    output_pose.orientation.w = q.w();

    if (this->degree_move_pose_mono != 0){
        // Move position and orientation of specified degree
        // TODO position
        
        // Orientation
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(output_pose.orientation, tf2_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(tf2_quat);
        m.getRPY(roll, pitch, yaw);
        tf2_quat.setRPY(0, 0, yaw-(this->degree_move_pose_mono * (M_PI / 180.0))); 
        output_pose.orientation = tf2::toMsg(tf2_quat);
    }

    message.pose.pose = output_pose;
    message.header.frame_id = header_id_frame;
    message.child_frame_id = child_id_frame;

    quaternion_pub->publish(message);

    // "End" time and saving times. File with times:
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tempo = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
    m_SLAM->InsertTrackTime(tempo);
}
