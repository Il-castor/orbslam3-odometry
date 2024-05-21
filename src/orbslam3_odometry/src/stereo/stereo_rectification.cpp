#include <opencv2/opencv.hpp>
#include <iostream>

/**
Function that reads the parameters contained in the path_yaml file. 
Of course, this function must be called once at the beginning of the execution.
The only input is path_yaml, that must be the path of the settings file of ORB-SLAM3.
All the other variables (map1_L,map2_L,roi_L, map1_R, map2_R, roi_R) are the output of the stereoRectification and undistortion.
The first three outputs (map1_L,map2_L,roi_L) are for the left camera (aka Camera1), while the other three are for the right (aka Camera2).
*/
void readParameters(std::string path_yaml, cv::Mat &map1_L, cv::Mat &map2_L, cv::Rect &roi_L, cv::Mat &map1_R, cv::Mat &map2_R, cv::Rect &roi_R) {

	std::cerr << "YAML file:" << path_yaml << std::endl;
    
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


/**
This function performs rectification of the image. 
- cv::Mat &img is both an input and output value. The expected image is the image that you want to rectify and crop. The output image is the rectified and cropped image.
- img_non_cropped is an output value. It contains the rectified (and NOT cropped) image.
- map1, map2, roi are the output of the readParameters function. 

Tip: in order to have left and right cropped images of the same size, in the parameter cv::Rect &roi provide (roi_L & roi_R) for both cameras.
*/
void rectify_image(cv::Mat &img, const cv::Mat &map1, const cv::Mat &map2, const cv::Rect &roi, cv::Mat &img_non_cropped){
    // In img -> saves rectified and cropped image in img.
    // In img_non_cropped -> saves rectified and NON cropped image in img_non_cropped (for disparity map)
    
    cv::remap(img, img_non_cropped, map1, map2, cv::INTER_LINEAR);
    img = img_non_cropped(roi);
}

/**
Same as above, but img_non_cropped will not be provided. 
img is still both an input and output value. The expected image is still the image that you want to rectify and crop. The output image is the rectified and cropped image.
*/
void rectify_image(cv::Mat &img, const cv::Mat &map1, const cv::Mat &map2, const cv::Rect &roi){
    // Perform rectification (and cropping) of one image
    rectify_image(img, map1, map2, roi, img);
}


/**
Function that shows the disparity map of the left and right images.
The two input images are the rectified left and right images.
THE IMAGES MUST BE OF THE SAME SIZE. To have images of the same size you can provide:
- rectified but not cropped images 
or
- rectified and cropped images, where the crop was made with the roi=(roi_L & roi_R)
*/
void show_disparity(const cv::Mat &img1_non_cropped, const cv::Mat &img2_non_cropped){
    cv::Mat disparityMap;
    const int numDisparitiesVecchio = 16;
    const int blockSizeVecchio = 15;
    cv::Ptr<cv::StereoSGBM> sgbmVecchio = cv::StereoSGBM::create(0, numDisparitiesVecchio, blockSizeVecchio);
    sgbmVecchio->compute(img1_non_cropped, img2_non_cropped, disparityMap);

    // Normalize the disparity map for display
    cv::normalize(disparityMap, disparityMap, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Just show disparity map...
     cv::imshow("Disparity Map Vecchia", disparityMap);
    // return;


    //... or alternatively shom images and disparity map concatenated
    /*std::cout << "Image 1 dimensions: " << img1_non_cropped.size() << ", type: " << img1_non_cropped.type() << std::endl;
    std::cout << "Image 2 dimensions: " << img2_non_cropped.size() << ", type: " << img2_non_cropped.type() << std::endl;
    std::cout << "Image 3 dimensions: " << disparityMap.size() << ", type: " << disparityMap.type() << std::endl;*/

    //nuovo metodo
    // Create SGBM object
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);

    // Set SGBM parameters
    const int numDisparities = 16 * 5;  // Maximum disparity minus minimum disparity
    const int blockSize = 5;            // Matched block size. It must be an odd number >=1

    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numDisparities);
    sgbm->setBlockSize(blockSize);
    sgbm->setP1(8 * img1_non_cropped.channels() * blockSize * blockSize);
    sgbm->setP2(32 * img2_non_cropped.channels() * blockSize * blockSize);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setPreFilterCap(63);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    // Compute the disparity map
    cv::Mat disparity;
    cv::Mat disparityMapNuova;
    sgbm->compute(img1_non_cropped, img2_non_cropped, disparity);

    // Normalize the disparity map for visualization
    cv::Mat disparity8U;
    normalize(disparity, disparity8U, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Mat concatenated;
    cv::hconcat(img1_non_cropped, img2_non_cropped, concatenated);
    // std::cout << "qui" << std::endl;
    cv::cvtColor(disparity8U, disparityMapNuova, cv::COLOR_GRAY2BGR);
    cv::hconcat(concatenated, disparityMapNuova, concatenated);
    
    cv::resize(concatenated, concatenated, cv::Size(concatenated.cols / 2, concatenated.rows / 2));

    cv::imshow("Left and right rectified + Disparity map Nuova ", concatenated);

}


