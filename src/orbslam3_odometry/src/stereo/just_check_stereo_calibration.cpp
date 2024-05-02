#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "stereo_rectification.h"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;

class JustCheckStereoCalibration : public rclcpp::Node
{
public:
    JustCheckStereoCalibration(const std::string &strSettingsFile) : Node("orbslam3_odometry")
    {

        this->loadParameters();

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();

        subscription_left = this->create_subscription<ImageMsg>(
            this->camera_left_topic, qos, std::bind(&JustCheckStereoCalibration::leftCallback, this, std::placeholders::_1));
        subscription_right = this->create_subscription<ImageMsg>(
            this->camera_right_topic, qos, std::bind(&JustCheckStereoCalibration::rightCallback, this, std::placeholders::_1));

        // Read the parametes and compute the common_roi, so that cropped and rectified images will have the same size
        readParameters(strSettingsFile, map1_L, map2_L, roi_L, map1_R, map2_R, roi_R);
        common_roi = roi_L & roi_R;

        RCLCPP_INFO(this->get_logger(), "CHECKING STEREO RECTIFICATION STARTED. IMAGES WILL BE READ FROM TOPICS %s and %s", this->camera_left_topic.c_str(), this->camera_right_topic.c_str());

        // cv::namedWindow("Synchronized Images", cv::WINDOW_NORMAL);

        // cv::namedWindow("Left NON Rectified", cv::WINDOW_NORMAL);
        // cv::namedWindow("Right NON Rectified", cv::WINDOW_NORMAL);
        cont_salva_immagini = 0;

        sync_timer_ = this->create_wall_timer(100ms, std::bind(&JustCheckStereoCalibration::SyncImages, this));
    }

private:
    using ImageMsg = sensor_msgs::msg::CompressedImage; // + cambia le due callbacks delle immagini

    void loadParameters()
    {

        declare_parameter("topic_camera_left", "/camera/left_image");
        declare_parameter("topic_camera_right", "/camera/right_image");
        declare_parameter("just_take_picture", false);

        get_parameter("topic_camera_left", this->camera_left_topic);
        get_parameter("topic_camera_right", this->camera_right_topic);
        get_parameter("just_take_picture", this->isTakingPicture);
    }

    void leftCallback(const ImageMsg::SharedPtr msg)
    {
        try
        {
            // RCLCPP_INFO(this->get_logger(), "left" );

            // left_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;  // For image
            left_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image; // For compressed images

            // cv::imshow("Left NON Rectified", left_image_);
            // RCLCPP_INFO(this->get_logger(), "left ok" );
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    void rightCallback(const ImageMsg::SharedPtr msg)
    {
        try
        {
            // RCLCPP_INFO(this->get_logger(), "right" );

            // right_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            right_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // cv::imshow("Right NON Rectified", right_image_);

            // RCLCPP_INFO(this->get_logger(), "right ok" );
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    void SyncImages()
    {
        // Main thread

        if (!left_image_.empty() && !right_image_.empty())
        {

            if (!isTakingPicture)
            {
                // isTakingPicture is false, so i will show rectification results and disparity map
                RCLCPP_INFO(this->get_logger(), "rectification...");

                // Perform stereo rectification. I save also the rectified but not cropped images
                cv::Mat img_non_cropped_L, img_non_cropped_R;
                rectify_image(left_image_, map1_L, map2_L, common_roi, img_non_cropped_L);
                rectify_image(right_image_, map1_R, map2_R, common_roi, img_non_cropped_R);

                // Draw horizontal lines on rectified images
                const int DISTANZA_RIGHE_PIXEL = 50;

                for (int y = 0; y < left_image_.rows; y += DISTANZA_RIGHE_PIXEL)
                {
                    cv::line(left_image_, cv::Point(0, y), cv::Point(left_image_.cols - 1, y), cv::Scalar(0, 0, 255), 1);
                }

                for (int y = 0; y < right_image_.rows; y += DISTANZA_RIGHE_PIXEL)
                {
                    cv::line(right_image_, cv::Point(0, y), cv::Point(right_image_.cols - 1, y), cv::Scalar(0, 0, 255), 1);
                }

                // Display the Rectified images
                // cv::imshow("Left Rectified (non cropped)", img_non_cropped_L);
                // cv::imshow("Right Rectified (non cropped)", img_non_cropped_R);

                // Perform stereo matching
                std::cout << "disparity map" << std::endl;
                show_disparity(left_image_, right_image_);

                // Clear matrixes so they won't be re-processed
                left_image_.release();
                right_image_.release();

                int key = cv::waitKey(1);
            }
            else
            {
                // Save PNG images

                // Show images (resized)
                cv::Mat concatenated;
                cv::Mat resizedImage1, resizedImage2;
                cv::resize(left_image_, resizedImage1, cv::Size(left_image_.cols / 2, left_image_.rows / 2));
                cv::resize(right_image_, resizedImage2, cv::Size(left_image_.cols / 2, left_image_.rows / 2));
                cv::hconcat(resizedImage1, resizedImage2, concatenated);
                cv::imshow("Left + Right ", concatenated);

                // Save images if spacebar is pressed
                int key = cv::waitKey(1);
                if (key == 32)
                { // Spacebar was pressed
                    // watch -n1 ls -Rl /home/formula-student/immagini
                    std::string path = "/home/formula-student/immagini";
                    std::cout << "Salvo immagini. CONTROLLA CHE IL PATH " << path << " + SOTTOCARTELLE left E right ESISTANO" << std::endl;
                    cv::imwrite(path + "/left/" + std::to_string(cont_salva_immagini) + ".jpg", left_image_);
                    cv::imwrite(path + "/right/" + std::to_string(cont_salva_immagini) + ".jpg", right_image_);
                    cont_salva_immagini++;

                    // Clear matrixes so they won't be re-processed
                    left_image_.release();
                    right_image_.release();
                }
            }

            // cv::waitKey(3000); // Adjust as needed
        }

        // rclcpp::spin_some(this->get_node_base_interface());

        // std::chrono::milliseconds tSleep(1000);
        // std::this_thread::sleep_for(tSleep);
    }

    // Rectification output params
    cv::Mat map1_L, map2_L, map1_R, map2_R;
    cv::Rect roi_L, roi_R, common_roi;

    std::string camera_left_topic, camera_right_topic;
    bool isTakingPicture;

    rclcpp::Subscription<ImageMsg>::SharedPtr subscription_left;
    rclcpp::Subscription<ImageMsg>::SharedPtr subscription_right;

    cv::Mat left_image_, right_image_;

    rclcpp::TimerBase::SharedPtr sync_timer_;

    int cont_salva_immagini;
};
