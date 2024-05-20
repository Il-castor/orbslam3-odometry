#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"

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
};

#endif

// image_msg_define.hpp
#ifndef IMAGE_MSG_DEFINE_HPP
#define IMAGE_MSG_DEFINE_HPP

#define ImageMsg sensor_msgs::msg::CompressedImage

#endif // IMAGE_MSG_DEFINE_HPP

