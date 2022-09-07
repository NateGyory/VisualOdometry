#ifndef TRACKING_H
#define TRACKING_H

#include "stereoCamera.h"
#include "keyframe.h"

struct PoseGraph
{
    std::vector<KeyFrame> keyframes;
};

void TrackPose(StereoCamera &stereo_cam);

#endif /* TRACKING_H */
