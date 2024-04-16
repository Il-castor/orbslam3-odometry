#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "orbslam3_odometry/utility.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "stereo_rectification.h"


class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify);

    ~StereoSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::CompressedImage;     // Se cambiato, cambia anche le due callbacks
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    // void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    void GrabStereo(cv::Mat image_L, cv::Mat image_R);
    void SyncImg();
    void leftCallback(const ImageMsg::SharedPtr msg);
    void rightCallback(const ImageMsg::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l,M2l,M1r,M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;
    std::thread *syncThread_;
    
    // Image
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    cv::Mat  left_image_, right_image_;
    double timestamp;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

    // std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    

    void loadParameters();
    /*List of all parameters */
    std::string camera_left, camera_right, imu, header_id_frame, child_id_frame; 
    std::string topic_pub_quat; 

    // Rectification params
    cv::Mat map1_L, map2_L, map1_R,  map2_R;
    cv::Rect roi_L, roi_R, common_roi;


};
#endif
