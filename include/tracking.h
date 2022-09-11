#ifndef TRACKING_H
#define TRACKING_H

#include "stereoCamera.h"
#include "keyframe.h"
#include <opencv2/highgui/highgui.hpp>

struct PoseGraph
{
    std::vector<KeyFrame> keyframes;
};

void TrackPose(StereoCamera &stereo_cam);

#endif /* TRACKING_H */