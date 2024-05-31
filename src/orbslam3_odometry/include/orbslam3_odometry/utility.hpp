#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"

// image_msg_define.hpp
#ifndef IMAGE_MSG_DEFINE_HPP
#define IMAGE_MSG_DEFINE_HPP

//#define ImageMsg sensor_msgs::msg::Image
#define ImageMsg sensor_msgs::msg::CompressedImage

#endif // IMAGE_MSG_DEFINE_HPP

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
  }

  static void cutting_image(cv::Mat &image, cv::Rect roiRect){
    image = image(roiRect);
  }

  static void printCommonInfo(rclcpp::QoS qos){
    #define STRINGIFY(x) #x
    #define TOSTRING(x) STRINGIFY(x)

    std::cout << "****\n\nYou are using images of type: " << TOSTRING(ImageMsg) << 
      ".\nIf you are using compresssed images, remember that you will not visualize them in rviz2" << std::endl;

    if (qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
        std::cout << "QoS is set to best effort." << std::endl;
    } else if (qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
        std::cout << "QoS is set to reliable." << std::endl;
    } else {
        std::cout << "QoS reliability policy is unknown." << std::endl;
    }

    std::cout << "\n\n****" << std::endl;

  }
};

#endif
