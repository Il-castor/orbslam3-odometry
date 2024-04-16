#include <opencv2/opencv.hpp>
#include <iostream>

void readParameters(std::string path_yaml, cv::Mat &map1_L, cv::Mat &map2_L, cv::Rect &roi_L, cv::Mat &map1_R, cv::Mat &map2_R, cv::Rect &roi_R) {

    // Load the YAML file
    cv::FileStorage fs(path_yaml, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Failed to open YAML file." << std::endl;
        return;
    }
    
    std::cout << "Reading cameras parameters..." << std::endl;

    // Read cameras parameters    
    double fx, fy, cx, cy, k1, k2, p1, p2; 

    fs["Camera1.fx"] >> fx;
    fs["Camera1.fy"] >> fy;
    fs["Camera1.cx"] >> cx;
    fs["Camera1.cy"] >> cy;
    
    cv::Mat camera_matrix_1 = (cv::Mat_<double>(3, 3,CV_64F) << fx, 0, cx,
                                                         0, fy, cy,
                                                         0, 0, 1);
    
    fs["Camera1.k1"] >> k1;
    fs["Camera1.k2"] >> k2;
    fs["Camera1.p1"] >> p1;
    fs["Camera1.p2"] >> p2;
    
    cv::Mat dist_coeffs_1 = (cv::Mat_<double>(1, 5,CV_64F) << k1, k2, p1, p2, 0);
    
    fs["Camera2.fx"] >> fx;
    fs["Camera2.fy"] >> fy;
    fs["Camera2.cx"] >> cx;
    fs["Camera2.cy"] >> cy;
    
    cv::Mat camera_matrix_2 = (cv::Mat_<double>(3, 3,CV_64F) << fx, 0, cx,
                                                         0, fy, cy,
                                                         0, 0, 1);
    
    fs["Camera2.k1"] >> k1;
    fs["Camera2.k2"] >> k2;
    fs["Camera2.p1"] >> p1;
    fs["Camera2.p2"] >> p2;
    
    cv::Mat dist_coeffs_2 = (cv::Mat_<double>(1, 5,CV_64F) << k1, k2, p1, p2, 0);


    // Read other parameters
    int camera_width, camera_height;
    cv::Mat T_c1_c2(4, 4,CV_64F);
    fs["Camera.width"] >> camera_width;
    fs["Camera.height"] >> camera_height;
    fs["Stereo.T_c1_c2"] >> T_c1_c2;

    // Print other parameters
    std::cout << "Camera width: " << camera_width << std::endl;
    std::cout << "Camera height: " << camera_height << std::endl;
    std::cout << "Reading stereo parameters..." << std::endl;

    // Define R and T from the stereo transformation matrix
    cv::Mat R, T;
    T_c1_c2(cv::Rect(0, 0, 3, 3)).convertTo(R, CV_64F);
    T_c1_c2(cv::Rect(3, 0, 1, 3)).convertTo(T, CV_64F);

    
    std::cout << "All parameter have been read. Check types: " << std::endl;
    
    // Check tipo
    std::cout << "camera_matrix_1.type():\t"<< camera_matrix_1.type() << std::endl;
    std::cout << "dist_coeffs_1.type():\t"<< dist_coeffs_1.type() << std::endl;
    
    std::cout << "camera_matrix_2.type():\t"<< camera_matrix_2.type() << std::endl;
    std::cout << "dist_coeffs_2.type():\t"<< dist_coeffs_2.type() << std::endl;
    
    std::cout << "R.type():\t"<< R.type() << std::endl;
    std::cout << "T.type():\t"<< T.type() << std::endl;



    // Perform stereo rectification
    std::cout << "Performing stereo rectification..." << std::endl;
    cv::Mat R1, R2, P1, P2, Q;
    try {
        cv::stereoRectify(camera_matrix_1, dist_coeffs_1, camera_matrix_2, dist_coeffs_2, cv::Size(camera_width, camera_height), R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, cv::Size(), &roi_L, &roi_R);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return;
    }

    // Perform initUndistortRectifyMap
    std::cout << "Performing initUndistortRectifyMap..." << std::endl;
    cv::initUndistortRectifyMap(camera_matrix_1, dist_coeffs_1, R1, P1, cv::Size(camera_width, camera_height), CV_32FC1, map1_L, map2_L);
    cv::initUndistortRectifyMap(camera_matrix_2, dist_coeffs_2, R2, P2, cv::Size(camera_width, camera_height), CV_32FC1, map1_R, map2_R);

    std::cout << "All parameter have been read. You can now perform remapping" << std::endl;
}



void rectify_image(cv::Mat &img, const cv::Mat &map1, const cv::Mat &map2, const cv::Rect &roi, cv::Mat &img_non_cropped){
    // Perform rectification (and cropping) of one image. 
    // In img -> saves rectified and cropped image in img.
    // In img_non_cropped -> saves rectified and NON cropped image in img_non_cropped (for disparity map)
    
    cv::remap(img, img_non_cropped, map1, map2, cv::INTER_LINEAR);
    img = img_non_cropped(roi);
}


void rectify_image(cv::Mat &img, const cv::Mat &map1, const cv::Mat &map2, const cv::Rect &roi){
    // Perform rectification (and cropping) of one image
    rectify_image(img, map1, map2, roi, img);
}

void show_disparity(const cv::Mat &img1_non_cropped, const cv::Mat &img2_non_cropped){
    cv::Mat disparityMap;
    int numDisparities = 16;
    int blockSize = 15;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, numDisparities, blockSize);
    sgbm->compute(img1_non_cropped, img2_non_cropped, disparityMap);

    // Normalize the disparity map for display
    cv::normalize(disparityMap, disparityMap, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Just show disparity map...
    // cv::imshow("Disparity Map", disparityMap);
    // return;


    //... or alternatively shom images and disparity map concatenated
    /*std::cout << "Image 1 dimensions: " << img1_non_cropped.size() << ", type: " << img1_non_cropped.type() << std::endl;
    std::cout << "Image 2 dimensions: " << img2_non_cropped.size() << ", type: " << img2_non_cropped.type() << std::endl;
    std::cout << "Image 3 dimensions: " << disparityMap.size() << ", type: " << disparityMap.type() << std::endl;*/



    cv::Mat concatenated;
    cv::hconcat(img1_non_cropped, img2_non_cropped, concatenated);
    // std::cout << "qui" << std::endl;
    cv::cvtColor(disparityMap, disparityMap, cv::COLOR_GRAY2BGR);
    cv::hconcat(concatenated, disparityMap, concatenated);

    cv::imshow("Left and right rectified + Disparity map", concatenated);

}


