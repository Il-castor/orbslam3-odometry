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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile);

    ~StereoSlamNode();

private:

    void loadParameters();
    void GrabStereo(cv::Mat image_L, cv::Mat image_R);
    void SyncImg();
    void leftCallback(const ImageMsg::SharedPtr msg);
    void rightCallback(const ImageMsg::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    // Images stuff
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;
    std::mutex bufMutexLeft_, bufMutexRight_;
    cv::Mat  left_image_, right_image_;
    double timestamp;

    std::thread *syncThread_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

    
    // List of all parameters 
    std::string camera_left, camera_right, imu, header_id_frame, child_id_frame; 
    std::string topic_pub_quat; 

    // Cutting parameters
    int cutting_x, cutting_y, cutting_width, cutting_height;
    cv::Rect cutting_rect;

    // Rectification params
    cv::Mat map1_L, map2_L, map1_R,  map2_R;
    cv::Rect roi_L, roi_R, common_roi;
    
    int contImageLeft, contImageRight, contTrackStereo;
    double firstTimeStampLeft, lastTimeStampLeft;
    
    // Left and right timestamp
    double tImLeft, tImRight;
    

    // Point cloud and Key points varables/methods
    std::vector<float> depths ;
    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time, Eigen::Vector3f actualPosition) ;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherPointCloud;
    cv::Scalar interpolateColor(float value, float minDepth, float maxDepth) ;


};
#endif
