#ifndef UTILS_H
#define UTILS_H

#include <fstream>

#include "nav_msgs/Path.h"
#include "rapidcsv.h"

namespace utils
{
namespace euroc
{
    void ParseGroundTruth(std::vector<nav_msgs::Path> &groundTruth);

}; // -- euroc
}; // -- utils

#endif /* UTILS_H */
