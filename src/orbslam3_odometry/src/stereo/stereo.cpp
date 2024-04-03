#include "stereo.hpp"

#include<opencv2/core/core.hpp>
#include<chrono>

// #include <geometry_msgs/msg/pose_stamped.h> 
// #include <geometry_msgs/msg/dds_connext/PoseStamped_.h>

using std::placeholders::_1; // TODO check se si possono cavare
using std::placeholders::_2;

const char* TOPIC_PUB_QUAD = "/Odometry/orbSlamOdom";

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("orbslam3_odometry"),
    m_SLAM(pSLAM)
{
    declare_parameter("topic_camera_left", "pippo");

    //stringstream ss(strDoRectify);
    //ss >> boolalpha >> doRectify;
    std::string my_param = ""; 
    get_parameter("topic_camera_left", my_param);
    std::cout << "FENICOTTERO " << my_param << endl;

    //std::cout << "FENICOTTERO - dorectify: "  << strDoRectify << "\tBoolean:" << doRectify <<endl;
    bool doRectify = true;
    if (doRectify){
        

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
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

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            // NB: Passa come ultimo argomento false, in modo da non entrare in questo ramo.
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/zed/zed_node/left/image_rect_color");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/zed/zed_node/right/image_rect_color");
    //quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry/orbSlamOdom", 10);
    quaternion_pub = this->create_publisher<nav_msgs::msg::Odometry>(TOPIC_PUB_QUAD, 10);
    

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
    
    img_proc = 0;

    std::cout << "Fine costruttore" << endl;

}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    std::cout << "Esco" << endl;
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

/**
 * Funzione che ritorna una stringa col quaternione, ogni valore e' separato da uno spazio.
*/
static std::string quaternionToString(const Eigen::Quaternionf& q){
    std::stringstream ss;
    // ss << q.x() << " " << q.y() << " " << q.z() << " " << q.w() ;
    ss << setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    return ss.str();
}

//#define ESTRAI_IMMAGINI
#ifdef ESTRAI_IMMAGINI
void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight){
    try
    {
        cv_bridge_ = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // auto timestamp = msg->header.stamp;
    // auto sec = std::to_string(timestamp.sec);
    // auto nsec = std::to_string(timestamp.nanosec);

    //std::string image_filename = main_dir + "/" + camera_name + "/" + sec + "." + nsec + ".png";
    std::string image_filename =  "images/left/" + img_proc + ".png";
    cv::imwrite(image_filename, cv_bridge_->image);
    RCLCPP_INFO(this->get_logger(), "[LEFT] Saved %s image: %s", camera_name.c_str(), image_filename.c_str());
    
    
    
    
    
    img_proc++;

}

#else
void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    
    // Tempo "inizio"
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Prendo il quaternione
    Sophus::SE3f Tcw ;
    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        //std::cout << "FENICOTTERO - IF" << endl;
        Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    } else {
        //std::cout << "FENICOTTERO - ELSE" << endl;
        Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));

    }
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();    // TODO capire cosa sia e se serve
    Eigen::Quaternionf q = Twc.unit_quaternion();

    // Stringa che contiene il quaternione
    std::string messaggio_quaternion  = quaternionToString(q);

    // "Nome" del file (nel formato ASL)
    double timestamp = Utility::StampToSec(msgLeft->header.stamp);  
        
    // Pubblico timestamp e quaternione
    // auto message = std_msgs::msg::String();
    auto message = nav_msgs::msg::Odometry();
    geometry_msgs::msg::Pose output_pose{};
    
    output_pose.position.x =  twc.z();
    output_pose.position.y = -twc.x();
    output_pose.position.z = 0 ;

     output_pose.orientation.x = -q.z() ;
    output_pose.orientation.y = -q.x();
    output_pose.orientation.z = -q.y() ;
    output_pose.orientation.w = q.w();
 

    message.pose.pose = output_pose;
    message.header.frame_id = "fl_track";
    message.child_frame_id = "orbslam3";


    //message.data = std::to_string(timestamp) + " " + messaggio_quaternion + "\n";
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    quaternion_pub->publish(message);

    // Tempo "fine" e salvataggio tempi. File con i tempi: 
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tempo = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
//    m_SLAM->InsertTrackTime(tempo); 
    //TODO cercare di capire perche' vengono inseriti meno tempi 
    //TODO forse perche' viene chiamata sempre TrackTime TUM 

}
#endif