#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo/stereo.hpp" //  TODO forse metti in include

#include "System.h"

#include <yaml-cpp/yaml.h>


int main(int argc, char **argv)
{
    // if(argc < 4)
    // {
    //     std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
    //     return 1;
    // }

        // TODO change questo come argomento
    // std::string file_path = "config/orbslam3_odometry.yaml";
    // std::ifstream fin(file_path);
    // YAML::Node data = YAML::Load("config/orbslam3_odometry.yaml");

    // if (data["PathVocabulary"]) {
        // std::cout << "Value of 'key': " << data["PathVocabulary"].as<std::string>() << std::endl;
    // }
    // return 0;




    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
