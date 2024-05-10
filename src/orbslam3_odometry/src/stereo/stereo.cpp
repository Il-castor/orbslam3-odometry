#include "stereo.hpp"

#include <opencv2/core/core.hpp>
#include <chrono>

// If defined the node will print debug information and will show disparity map
// #define DEBUG

// If defined, it will pre-rectify images, before giving it in input to orbslam.
// It should be enables for Basler and disabled for stereo-camera like Zed
#define PRE_RECTIFY_IMAGES

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

    // Starts orbslam3. This thread is used to syncronize the two images
    syncThread_ = new std::thread(&StereoSlamNode::SyncImg, this);
    // std::cout << "End Costructor" << endl;
}

StereoSlamNode::~StereoSlamNode()
{
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
    bufMutexLeft_.lock();
    try
    {

        RCLCPP_INFO(this->get_logger(), "left" );

        // left_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;  // For image
        left_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;      // For compressed images

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
        RCLCPP_INFO(this->get_logger(), "right" );

        // right_image_ = cv_bridge::toCvShare(msg, "bgr8")->image; // For image
        right_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;     // For compressed images

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
    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!left_image_.empty() && !right_image_.empty())
        {
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
    // Initial time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

#ifdef PRE_RECTIFY_IMAGES
    // First of all we pre-rectify the images
    cv::Mat left_rectified_non_cropped, right_rectified_non_cropped;
    rectify_image(image_L, map1_L, map2_L, common_roi, left_rectified_non_cropped);
    rectify_image(image_R, map1_R, map2_R, common_roi, right_rectified_non_cropped);

#ifdef DEBUG
    show_disparity(image_L, image_R);
#endif

    // If necessary, perform changes to the images here.
    if (cutting_x != -1){
        Utility::cutting_image(left_rectified_non_cropped, cutting_rect);
        Utility::cutting_image(right_rectified_non_cropped, cutting_rect);
    }

    // Call ORB-SLAM3 on the 2 pre-rectified images
    Sophus::SE3f Tcw = m_SLAM->TrackStereo(left_rectified_non_cropped, right_rectified_non_cropped, timestamp);
#else
    // If necessary, perform changes to the images here.
    if (cutting_x != -1){
        Utility::cutting_image(image_L, cutting_rect);
        Utility::cutting_image(image_R, cutting_rect);
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

    message.pose.pose = output_pose;
    message.header.frame_id = header_id_frame;
    message.child_frame_id = child_id_frame;

    quaternion_pub->publish(message);

    // "End" time and saving times. File with times:
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tempo = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
    m_SLAM->InsertTrackTime(tempo);
}