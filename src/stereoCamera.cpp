#include "stereoCamera.h"
#include <iostream>

void InitStereoCamera(StereoCamera &stereo_cam) {
  const std::string camera_params_file =
      "/home/nate/Development/StereoSLAM/config/EuRoC.yaml";
  cv::FileStorage fs(camera_params_file, cv::FileStorage::READ);

  // L_cam params
  fs["LEFT.D"] >> stereo_cam.L_cam.D;
  fs["LEFT.K"] >> stereo_cam.L_cam.K;
  // fs["LEFT.R"] >> stereo_cam.L_cam.R;
  // fs["LEFT.P"] >> stereo_cam.L_cam.P;

  // R_cam params
  fs["RIGHT.D"] >> stereo_cam.R_cam.D;
  fs["RIGHT.K"] >> stereo_cam.R_cam.K;
  // fs["RIGHT.R"] >> stereo_cam.R_cam.R;
  // fs["RIGHT.P"] >> stereo_cam.R_cam.P;

  // img size
  fs["LEFT.width"] >> stereo_cam.imageSize.width;
  fs["RIGHT.height"] >> stereo_cam.imageSize.height;

  cv::Mat T_B_LEFT, T_B_RIGHT, T_RIGHT_LEFT, T_R_L;

  fs["T_B_LEFT"] >> T_B_LEFT;
  fs["T_B_RIGHT"] >> T_B_RIGHT;

  T_RIGHT_LEFT = T_B_LEFT.inv() * T_B_RIGHT;
  T_RIGHT_LEFT = T_RIGHT_LEFT.inv();
  T_RIGHT_LEFT.convertTo(T_R_L, CV_64F);

  // Set the Base lenght value
  stereo_cam.B = cv::norm(T_R_L.rowRange(0, 3).col(3));

  // Temp output mats
  cv::Mat R1, R2, P1, P2;

  cv::stereoRectify(stereo_cam.L_cam.K, stereo_cam.L_cam.D, stereo_cam.R_cam.K,
                    stereo_cam.R_cam.D, stereo_cam.imageSize,
                    T_R_L.rowRange(0, 3).colRange(0, 3),
                    T_R_L.rowRange(0, 3).col(3), R1, R2, P1, P2, stereo_cam.Q);

  stereo_cam.L_cam.P = P1;
  stereo_cam.R_cam.P = P2;

  cv::initUndistortRectifyMap(stereo_cam.L_cam.K, stereo_cam.L_cam.D, R1,
                              P1.rowRange(0, 3).colRange(0, 3),
                              stereo_cam.imageSize, CV_32FC1,
                              stereo_cam.L_cam.map1, stereo_cam.L_cam.map2);

  cv::initUndistortRectifyMap(stereo_cam.R_cam.K, stereo_cam.R_cam.D, R2,
                              P2.rowRange(0, 3).colRange(0, 3),
                              stereo_cam.imageSize, CV_32FC1,
                              stereo_cam.R_cam.map1, stereo_cam.R_cam.map2);

  // Get images from EuRoC
  GetImages(stereo_cam.imgStream);
}

void GetImages(ImageStream &imgStream) {
  const std::string Lcam_path =
      "/home/nate/Development/StereoSLAM/datasets/EuRoC/mav0/cam0/data/";
  const std::string Rcam_path =
      "/home/nate/Development/StereoSLAM/datasets/EuRoC/mav0/cam1/data/";
  const std::string ts_path =
      "/home/nate/Development/StereoSLAM/datasets/EuRoC/mav0/cam0/data.csv";

  std::ifstream ts_file(ts_path);
  std::string line, word, img_name;

  // Strip header line
  std::getline(ts_file, word);
  while (std::getline(ts_file, line)) {
    std::getline(std::stringstream{line}, word, ',');
    imgStream.time_stamp.push_back(word);
    img_name = word + ".png";
    imgStream.left_images.push_back(Lcam_path + img_name);
    imgStream.right_images.push_back(Rcam_path + img_name);
  }
}
