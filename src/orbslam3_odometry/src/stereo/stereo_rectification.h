#include <opencv2/opencv.hpp>
#include <iostream>

// Documentation is in .cpp file

void readParameters(std::string path_yaml, cv::Mat &map1_L, cv::Mat &map2_L, cv::Rect &roi_L, cv::Mat &map1_R, cv::Mat &map2_R, cv::Rect &roi_R) ;

void rectify_image(cv::Mat &img, const cv::Mat &map1, const cv::Mat &map2, const cv::Rect &roi, cv::Mat &img_non_cropped);

void rectify_image(cv::Mat &img, const cv::Mat &map1, const cv::Mat &map2, const cv::Rect &roi) ; 

void show_disparity(const cv::Mat &img1_non_cropped, const cv::Mat &img2_non_cropped) ; 

