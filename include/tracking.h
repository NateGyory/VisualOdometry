#ifndef TRACKING_H
#define TRACKING_H

#include "stereoCamera.h"
#include "keyframe.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"

//struct PoseGraph
//{
//    std::vector<KeyFrame> keyframes;
//};

void TrackPose(StereoCamera &stereo_cam);

#endif /* TRACKING_H */
