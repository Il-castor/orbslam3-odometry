#include "stereo.hpp"

#include <opencv2/core/core.hpp>
#include <chrono>

#define DEBUG true

using std::placeholders::_1;
using std::placeholders::_2;

void StereoSlamNode::loadParameters()
{
    /* ***** DECLARING PARAMETERS ***** */

    declare_parameter("topic_camera_left", "/camera/left_image");
    declare_parameter("topic_camera_right", "/camera/right_image");
    declare_parameter("topic_imu", "/imu/data");
    declare_parameter("topic_orbslam_odometry", "/Odometry/orbSlamOdom");
    declare_parameter("topic_header_frame_id", "os_track");
    declare_parameter("topic_child_frame_id", "orbslam3");

    /* ******************************** */

    /* ***** READING PARAMETERS ***** */

    get_parameter("topic_camera_left", this->camera_left);
    get_parameter("topic_camera_right", this->camera_right);
    get_parameter("topic_imu", this->imu);
    get_parameter("topic_orbslam_odometry", this->topic_pub_quat);
    get_parameter("topic_header_frame_id", this->header_id_frame);
    get_parameter("topic_child_frame_id", this->child_id_frame);
}

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System *pSLAM, const string &strSettingsFile, const string &strDoRectify)
    : Node("orbslam3_odometry"),
      m_SLAM(pSLAM)
{
    this->loadParameters();

    readParameters(strSettingsFile, map1_L, map2_L, roi_L, map1_R, map2_R, roi_R) ;
    common_roi = roi_L & roi_R;

    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    std::cout << "strDoRectify " << boolalpha << " " << doRectify << std::endl;

#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "Topic camera left: %s", this->camera_left.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic camera right: %s", this->camera_right.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic imu: %s", this->imu.c_str());
    RCLCPP_INFO(this->get_logger(), "header_id_frame: %s", this->header_id_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "child_id_frame: %s", this->child_id_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "topic_orbslam_odometry: %s", this->topic_pub_quat.c_str());
#endif

    // std::cout << "dorectify: "  << strDoRectify << "\tBoolean:" << doRectify <<endl;

    if (doRectify) // TODO questo io lo caverei e metterei il nostro codice
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            // NB: Passa come ultimo argomento false, in modo da non entrare in questo ramo.
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    subImgLeft_ = this->create_subscription<ImageMsg>(this->camera_left, qos, std::bind(&StereoSlamNode::leftCallback, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>(this->camera_right, qos, std::bind(&StereoSlamNode::rightCallback, this, _1));

    // left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), this->camera_left);
    // right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), this->camera_right);
    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_pub_quat, 10);

    // syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    // syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 STARTED IN STEREO MODE. NODE WILL WAIT FOR IMAGES IN TOPICS %s and %s", this->camera_left.c_str(), this->camera_right.c_str());

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
    const double maxTimeDiff = 0.01;    // TODO check big time difference

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!left_image_.empty() && !right_image_.empty())
        {
            

            StereoSlamNode::GrabStereo(left_image_, right_image_);
            left_image_.release();
            right_image_.release();

        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void StereoSlamNode::GrabStereo(cv::Mat image_L, cv::Mat image_R)
{
    // init time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


    // Quaternion
    Sophus::SE3f Tcw;

    rectify_image(image_L, map1_L, map2_L, common_roi);
    rectify_image(image_R, map1_R, map2_R, common_roi);

#ifdef DEBUG
    show_disparity(image_L, image_R);
#endif

    if (doRectify)
    {
        cv::remap(image_L, image_L, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(image_R, image_R, M1r, M2r, cv::INTER_LINEAR);
        Tcw = m_SLAM->TrackStereo(image_L, image_R, timestamp);
    }
    else
    {
        Tcw = m_SLAM->TrackStereo(image_L, image_R, timestamp);
    }
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // String containing the quaternion
    std::string messaggio_quaternion = quaternionToString(q);

    // "filename" (in ASL format)
    // double timestamp = Utility::StampToSec(msgLeft->header.stamp);

    // I publish position and quaternion
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