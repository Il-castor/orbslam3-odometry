#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "orbslam3_odometry/utility.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:

    void GrabImage(const ImageMsg::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

    void loadParameters();
    /*List of all parameters */
    std::string camera_left, camera_right, imu, header_id_frame, child_id_frame; 
    std::string topic_pub_quat; 
    bool isCameraLeft;
    int scale_position_mono;
    int degree_move_pose_mono;

    // Cutting parameters
    int cutting_x, cutting_y, cutting_width, cutting_height;
    cv::Rect cutting_rect;

    // Point cloud and Key points varables/methods
    std::vector<float> depths ;
    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time, Eigen::Vector3f actualPosition) ;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherPointCloud;
    cv::Scalar interpolateColor(float value, float minDepth, float maxDepth) ;

};

#endif
