#include "utils.h"

namespace utils
{
namespace euroc
{

void ParseGroundTruth(nav_msgs::Path &groundTruth)
{
    std::string groundTruthFile = "/home/nate/Development/StereoSLAM/datasets/EuRoC/mav0/state_groundtruth_estimate0/data.csv";
    rapidcsv::Document doc(groundTruthFile);

    std::vector<geometry_msgs::PoseStamped> poseArr;
    poseArr.reserve(1000);

    ros::Time time;
    int seq = 0;

    std::vector<unsigned long> col = doc.GetColumn<unsigned long>("#timestamp");
    for ( int i = 0 ; i < col.size(); i++ )
    {
        double q_x = doc.GetCell<double>(5, i);
        double q_y = doc.GetCell<double>(6, i);
        double q_z = doc.GetCell<double>(7, i);
        double q_w = doc.GetCell<double>(4, i);

        double x = doc.GetCell<double>(1, i);
        double y = doc.GetCell<double>(2, i);
        double z = doc.GetCell<double>(3, i);

        std_msgs::Header header;
        header.seq = seq++;
        header.stamp = time.now();
        header.frame_id = "map";

        geometry_msgs::PoseStamped poseStamped;
        geometry_msgs::Pose pose;
        geometry_msgs::Point point;
        point.x = y;
        point.y = -z;
        point.z = -x;
        geometry_msgs::Quaternion quat;
        quat.x = q_y;
        quat.y = -q_z;
        quat.z = -q_x;
        quat.w = q_w;
        pose.position = point;
        pose.orientation = quat;

        poseStamped.header = header;
        poseStamped.pose = pose;

        poseArr.push_back(poseStamped);
    }

    std_msgs::Header pathHeader;
    pathHeader.seq = 0;
    pathHeader.stamp = time.now();
    pathHeader.frame_id = "map";

    groundTruth.header = pathHeader;
    groundTruth.poses = poseArr;
}

} // -- euroc
} // -- utils
