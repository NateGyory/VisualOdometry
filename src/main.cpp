#include "ros/ros.h"
#include "stereoCamera.h"
#include "tracking.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "DronePose");

  StereoCamera stereo_cam;

  InitStereoCamera(stereo_cam);

  TrackPose(stereo_cam);

  return 0;
}
