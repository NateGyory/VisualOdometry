#ifndef UTILS_H
#define UTILS_H

#include <fstream>

#include "nav_msgs/Path.h"
#include "rapidcsv.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

namespace utils {
namespace euroc {
void ParseGroundTruth(nav_msgs::Path &groundTruth);
void ParseImuPath(nav_msgs::Path &imuPath);

}; // namespace euroc
}; // namespace utils

#endif /* UTILS_H */
