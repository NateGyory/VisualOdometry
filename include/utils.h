#ifndef UTILS_H
#define UTILS_H

#include <fstream>

#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "rapidcsv.h"
#include "ros/ros.h"

namespace utils
{
namespace euroc
{
    void ParseGroundTruth(nav_msgs::Path &groundTruth);

}; // -- euroc
}; // -- utils

#endif /* UTILS_H */
