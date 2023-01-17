#ifndef TRACKING_H
#define TRACKING_H

#include "opencv2/imgproc.hpp"
#include "stereoCamera.h"
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

void TrackPose(StereoCamera &stereo_cam);

#endif /* TRACKING_H */
