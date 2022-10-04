#include "utils.h"

void utils::euroc::ParseGroundTruth(std::vector<nav_msgs::Path> &groundTruth)
{
    std::string groundTruthFile = "/home/nate/Development/StereoSLAM/datasets/EuRoC/state_groundtruth_estimate0/data.csv";
    rapidcsv::Document doc(groundTruthFile);

    std::vector<float> col = doc.GetColumn<float>("Close");
    std::cout << "Read " << col.size() << " values." << std::endl;

}
