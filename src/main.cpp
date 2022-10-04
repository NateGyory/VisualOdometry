//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include "opencv2/imgproc.hpp"
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/voxel_grid.h>

//void GeneratePointCloud(StereoCamera &stereo_cam)
//{
//    cv::Ptr<cv::StereoBM> block_matcher = cv::StereoBM::create();
//    cv::namedWindow("disparity", cv::WINDOW_NORMAL);
//
//    cv::Mat disp, disparity, imgL_rec, imgR_rec, mat_pointcloud;
//
//    for( int i = 0; i < stereo_cam.imgStream.left_images.size(); i++)
//    {
//        cv::Mat imgL = cv::imread(stereo_cam.imgStream.left_images[i], cv::IMREAD_GRAYSCALE);
//        cv::Mat imgR = cv::imread(stereo_cam.imgStream.right_images[i], cv::IMREAD_GRAYSCALE);
//
//        if(imgL.empty() || imgR.empty())
//        {
//            std::cout << "Could not read the image: " << std::endl;
//            std::exit(1);
//        }
//
//        cv::remap(imgL, imgL_rec, stereo_cam.L_cam.map1, stereo_cam.L_cam.map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
//        cv::remap(imgR, imgR_rec, stereo_cam.R_cam.map1, stereo_cam.R_cam.map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
//
//
//        // Create disparity images
//        block_matcher->compute(imgL_rec, imgR_rec, disp);
//        disp.convertTo(disparity,CV_32F, 1.0);
//        disparity = (disparity/16.0f);
//
//        cv::reprojectImageTo3D(disparity, mat_pointcloud, stereo_cam.Q, true);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//        const double max_z = 1.0e4;
//
//        for (int i = 0; i < mat_pointcloud.rows; i++)
//        {
//            for (int j = 0; j < mat_pointcloud.cols; j++)
//            {
//                cv::Vec3f matPoint = mat_pointcloud.at<cv::Vec3f>(i, j);
//                pcl::PointXYZ pclPoint;
//                if(fabs(matPoint[2] - max_z) < FLT_EPSILON || fabs(matPoint[2]) > max_z) continue;
//                pclPoint.x = matPoint[0];
//                pclPoint.y = matPoint[1];
//                pclPoint.z = matPoint[2];
//                cloud->points.push_back(pclPoint);
//
//            }
//        }
//
//        cloud->width = cloud->points.size();
//        cloud->height = 1;
//        cloud->is_dense = false;
//
//        pcl::visualization::CloudViewer viewer("Cloud Viewer");
//        viewer.showCloud(cloud);
//
//        cv::imshow("Left IMG RECTIFIED", imgL_rec);
//        cv::imshow("Right IMG RECTIFIED", imgR_rec);
//        cv::imshow("disparity",disparity);
//        cv::waitKey(0);
//
//    }
//}

#include "stereoCamera.h"
#include "tracking.h"
#include "ros/ros.h"
#include "utils.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DronePose");

    //StereoCamera stereo_cam;

    //InitStereoCamera(stereo_cam);

    // Read in and publish ground truth Path
    std::vector<nav_msgs::Path> groundTruth;
    utils::euroc::ParseGroundTruth(groundTruth);

    //TrackPose(stereo_cam);

    return 0;
}
