#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include "opencv2/core.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/matx.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct ImageStream {
  std::vector<std::string> left_images, right_images, time_stamp;
  int idx;
};

struct CameraParams {
  cv::Mat K;    // Camera intrinsic matric
  cv::Mat D;    // Distortion coefficients
  cv::Mat R;    // Rectified rotation matrix
  cv::Mat P;    // Rectified extrinsix camera matrix
  cv::Mat map1; // map1 for remap stereo undistort rectify
  cv::Mat map2; // map2 for remap stereo undistort rectify
};

struct StereoCamera {
  ImageStream imgStream;
  CameraParams L_cam, R_cam; // Left and right cameras
  cv::Mat Q;                 // 4x4 disparity-depth matrix
  cv::Size2i imageSize;
  double B; // Baseline
};

void GetImages(ImageStream &imgStream);

void InitStereoCamera(StereoCamera &stereo_cam);

#endif /* STEREO_CAMERA_H */
