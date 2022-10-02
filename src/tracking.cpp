#include "tracking.h"
#include <iostream> // !!! Deleteme
#include <string>
#include "Eigen/Core"
//#include <pangolin/pangolin.h>
#include <unistd.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include <ros/time.h>

cv::Mat rotation(3, 3, CV_64F);
cv::Point3f translation(0.0f, 0.0f, 0.0f);
cv::Ptr<cv::StereoBM> block_matcher = cv::StereoBM::create();
cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

//void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {
//
//    if (pointcloud.empty()) {
//        std::cerr << "Point cloud is empty!" << std::endl;
//        return;
//    }
//
//    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//    pangolin::OpenGlRenderState s_cam(
//        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
//        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
//    );
//
//    pangolin::View &d_cam = pangolin::CreateDisplay()
//        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
//        .SetHandler(new pangolin::Handler3D(s_cam));
//
//    while (pangolin::ShouldQuit() == false) {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        d_cam.Activate(s_cam);
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//
//        glPointSize(2);
//        glBegin(GL_POINTS);
//        for (auto &p: pointcloud) {
//            glColor3f(p[3], p[3], p[3]);
//            glVertex3d(p[0], p[1], p[2]);
//        }
//        glEnd();
//        pangolin::FinishFrame();
//        usleep(5000);   // sleep 5 ms
//    }
//    return;
//}

void undistortRectifyStereoPair(StereoCamera &stereo_cam, std::string Limgf, std::string Rimgf, cv::Mat &imgL, cv::Mat &imgR)
{
    cv::Mat _imgL = cv::imread(Limgf, cv::IMREAD_GRAYSCALE);
    cv::Mat _imgR = cv::imread(Rimgf, cv::IMREAD_GRAYSCALE);
    cv::remap(_imgL, imgL, stereo_cam.L_cam.map1, stereo_cam.L_cam.map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    cv::remap(_imgR, imgR, stereo_cam.R_cam.map1, stereo_cam.R_cam.map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
}

void computeDisparityOld(StereoCamera &stereo_cam, const cv::Mat &imgL, const cv::Mat &imgR, cv::Mat &disparity)
{
    cv::Mat _disparity;
    block_matcher->compute(imgL, imgR, _disparity);
    _disparity.convertTo(disparity,CV_32F, 1.0 / 16.0f);
    //cv::reprojectImageTo3D(disparity, scene3D, stereo_cam.Q, true);
}

void computeDisparityNew(StereoCamera &stereo_cam, const cv::Mat &imgL, const cv::Mat &imgR, cv::Mat &disparity)
{
    cv::Mat _disparity;
    sgbm->compute(imgL, imgR, _disparity);
    _disparity.convertTo(disparity, CV_32F, 1.0 / 16.0f);
    //cv::reprojectImageTo3D(disparity, scene3D, stereo_cam.Q, true);
}

void computePointcloud(StereoCamera &stereo_cam, cv::Mat disparity, const cv::Mat &img, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud)
{
    for (int v = 0; v < img.rows; v++)
    {
        for (int u = 0; u < img.cols; u++) {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;

            Eigen::Vector4d point(0, 0, 0, img.at<uchar>(v, u) / 255.0);

            double x = (u - stereo_cam.L_cam.P.at<double>(0,2)) / stereo_cam.L_cam.P.at<double>(0,0);
            double y = (v - stereo_cam.L_cam.P.at<double>(1,2)) / stereo_cam.L_cam.P.at<double>(1,1);
            double depth = stereo_cam.L_cam.P.at<double>(0,0) * stereo_cam.B / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
    }
}

void TrackPose(StereoCamera &stereo_cam)
{

    unsigned int seq = 0;
    std::string frame_id = "map";
    ros::NodeHandle n;
    ros::Time time;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("Pose", 1000);

    // Get initial image, undistort and rectify
    std::string Limgf = stereo_cam.imgStream.left_images[0];
    std::string Rimgf = stereo_cam.imgStream.right_images[0];
    cv::Mat prev_imgL, prev_imgR, current_imgL, current_imgR;
    undistortRectifyStereoPair(stereo_cam, Limgf, Rimgf, prev_imgL, prev_imgR);

    // Get initial image disparity
    cv::Mat disparity;
    computeDisparityNew(stereo_cam, prev_imgL, prev_imgR, disparity);

    // Make point cloud for 3D mapping
    //std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;
    //computePointcloud(stereo_cam, disparity, prev_imgL, pointcloud);
    //showPointCloud(pointcloud);

    // Get initial Limg key points and descriptors
    std::vector<cv::KeyPoint> prev_keypoints, current_keypoints;
    cv::Mat prev_descriptors, current_descriptors;
    detector->detectAndCompute(prev_imgL, cv::noArray(), prev_keypoints, prev_descriptors);

    // Global T_Initial_Current
    cv::Mat T_Init_Current = cv::Mat::eye(4, 4, CV_64F);
    int init_flag = 0;

    // Loop over images
    for(int i = 1; i < stereo_cam.imgStream.left_images.size(); i++)
    {
        // Create a key frame
        Limgf = stereo_cam.imgStream.left_images[i];
        Rimgf = stereo_cam.imgStream.right_images[i];

        undistortRectifyStereoPair(stereo_cam, Limgf, Rimgf, current_imgL, current_imgR);

        detector->detectAndCompute(current_imgL, cv::noArray(), current_keypoints, current_descriptors);

        std::vector<std::vector<cv::DMatch>> rawMatches;
        std::vector<cv::DMatch> knnMatches;
        std::vector<cv::Point2i> point_matches_current, point_matches_previous;
        matcher->knnMatch(current_descriptors, prev_descriptors, rawMatches, 2);

        for (size_t i = 0; i < rawMatches.size(); i++)
        {
            const std::vector<cv::DMatch>& m = rawMatches[i];
            if (m[0].distance < m[1].distance * 0.5)
            {
                point_matches_current.push_back(current_keypoints[m[0].queryIdx].pt);
                point_matches_previous.push_back(prev_keypoints[m[0].trainIdx].pt);
                knnMatches.push_back(m[0]);
            }
        }

        // Compute the 3D points for the point_matches_previous
        std::vector<cv::Point3d> points3D;
        std::vector<cv::Point2f> points2D;
        for (int i = 0; i < point_matches_previous.size(); i++)
        {
            int x_prev = point_matches_previous[i].x;
            int y_prev = point_matches_previous[i].y;

            int x_current = point_matches_current[i].x;
            int y_current = point_matches_current[i].y;

            float disparity_val = disparity.at<float>(y_prev, x_prev);
            if (disparity_val <= 0.0 || disparity_val >= 96.0) continue;

            double x = (x_prev - stereo_cam.L_cam.P.at<double>(0,2)) / stereo_cam.L_cam.P.at<double>(0,0);
            double y = (y_prev - stereo_cam.L_cam.P.at<double>(1,2)) / stereo_cam.L_cam.P.at<double>(1,1);
            double depth = stereo_cam.L_cam.P.at<double>(0,0) * stereo_cam.B / disparity_val;

            points3D.emplace_back(x*depth, y*depth, depth);
            points2D.emplace_back(x_current, y_current);
        }

        cv::imshow("Current frame", current_imgL);
        cv::waitKey(1);

        prev_imgL.release();
        prev_imgR.release();
        prev_keypoints.clear();
        prev_descriptors.release();
        prev_imgL = current_imgL;
        prev_imgR = current_imgR;
        prev_keypoints = current_keypoints;
        prev_descriptors = current_descriptors;
        current_imgL.release();
        current_imgR.release();
        current_keypoints.clear();
        current_descriptors.release();
        knnMatches.clear();
        disparity.release();

        computeDisparityNew(stereo_cam, prev_imgL, prev_imgR, disparity);

        // SolvePnP to get the Homogeneous transform
        cv::Mat rVec, tVec;
        if (points2D.size() >= 6 && points3D.size() >= 6)
        {
            cv::solvePnPRansac(points3D, points2D, stereo_cam.L_cam.P.rowRange(0,3).colRange(0,3), cv::noArray(), rVec, tVec);

            cv::Mat R;
            cv::Rodrigues(rVec, R);
            cv::Mat T_Cur_Prev;
            cv::hconcat(R, tVec, T_Cur_Prev);
            std::vector<double> vf = { 0.f, 0.f, 0.f, 1.f };
            cv::Mat vec(1, 4, CV_64F, vf.data());
            cv::vconcat(T_Cur_Prev, vec, T_Cur_Prev);
            cv::Mat T_Prev_Cur = T_Cur_Prev.inv();

            T_Init_Current = T_Init_Current * T_Prev_Cur;
            //cv::solvePnPRefineLM(points3D, points2D, stereo_cam.L_cam.K, stereo_cam.L_cam.D, rVec, tVec);

            // Create the Pose
            std_msgs::Header header;
            header.seq = seq++;
            header.stamp = time.now();
            header.frame_id = frame_id;

            Eigen::Matrix3d rot(3, 3);
            rot(0,0) = T_Init_Current.at<double>(0,0);
            rot(0,1) = T_Init_Current.at<double>(0,1);
            rot(0,2) = T_Init_Current.at<double>(0,2);
            rot(1,0) = T_Init_Current.at<double>(1,0);
            rot(1,1) = T_Init_Current.at<double>(1,1);
            rot(1,2) = T_Init_Current.at<double>(1,2);
            rot(2,0) = T_Init_Current.at<double>(2,0);
            rot(2,1) = T_Init_Current.at<double>(2,1);
            rot(2,2) = T_Init_Current.at<double>(2,2);
            //cv::cv2eigen(T_Init_Current, rot);
            Eigen::Quaterniond quat(rot);

            double x = T_Init_Current.at<double>(0,3);
            double y = T_Init_Current.at<double>(1,3);
            double z = T_Init_Current.at<double>(2,3);
            geometry_msgs::PoseStamped poseStamped;
            geometry_msgs::Pose pose;
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            geometry_msgs::Quaternion ros_quat;
            ros_quat.x = quat.x();
            ros_quat.y = quat.y();
            ros_quat.z = quat.z();
            ros_quat.w = quat.w();
            pose.position = point;
            pose.orientation = ros_quat;
            poseStamped.header = header;
            poseStamped.pose = pose;
            pose_pub.publish(poseStamped);
        }


        // Apply the rotation and translation to the global pose



        // Display the matches between the previous frame and current frame
//        cv::Mat img_match;
//        drawMatches (current_imgL, current_keypoints, prev_imgL, prev_keypoints, knnMatches, img_match );
//
//        imshow ( "KNN Matches", img_match );
//        cv::waitKey(0);

        // Set prev data to current to process next frame

        // Need to compute the new disparity

    }
    std::cout << "Current translation" << std::endl;
    std::cout << T_Init_Current << std::endl;

}
