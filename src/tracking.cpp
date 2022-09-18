#include "tracking.h"
#include <iostream> // !!! Deleteme

cv::Mat rotation(3, 3, CV_64F);
cv::Point3f translation(0.0f, 0.0f, 0.0f);
cv::Ptr<cv::StereoBM> block_matcher = cv::StereoBM::create();
cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

void undistortRectifyStereoPair(StereoCamera &stereo_cam, std::string Limgf, std::string Rimgf, cv::Mat &imgL, cv::Mat &imgR)
{
    cv::Mat _imgL = cv::imread(Limgf, cv::IMREAD_GRAYSCALE);
    cv::Mat _imgR = cv::imread(Rimgf, cv::IMREAD_GRAYSCALE);
    cv::remap(_imgL, imgL, stereo_cam.L_cam.map1, stereo_cam.L_cam.map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    cv::remap(_imgR, imgR, stereo_cam.R_cam.map1, stereo_cam.R_cam.map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
}

void compute3DScene(StereoCamera &stereo_cam, const cv::Mat &imgL, const cv::Mat &imgR, cv::Mat &scene3D)
{
    cv::Mat disparity, _disparity;
    block_matcher->compute(imgL, imgR, _disparity);
    _disparity.convertTo(disparity,CV_32F, 1.0);
    disparity = (disparity/16.0f);
    cv::reprojectImageTo3D(disparity, scene3D, stereo_cam.Q, true);
}

void TrackPose(StereoCamera &stereo_cam)
{
    // Steps:
    //   1) Get previous frame
    //   2) Get current frame
    //   3) Find matches between previous frame and current frame
    //   4) Get 3 matches which are furthest apart from each other
    //   5) Get the 3D coordinates from the previous frame
    //   6) Get the 2D pts from the current from
    //   7) SolvePnP
    //   8) Use rodrigues to get the rotation and translation matrix
    //   9) Multiply Rotation and add translation to get the new pose
    //   10) Display pose in 3D
    //   11) Check pose against ground truth
    //


    // Get initial image, undistort and rectify
    std::string Limgf = stereo_cam.imgStream.left_images[0];
    std::string Rimgf = stereo_cam.imgStream.right_images[0];
    cv::Mat prev_imgL, prev_imgR, current_imgL, current_imgR;
    undistortRectifyStereoPair(stereo_cam, Limgf, Rimgf, prev_imgL, prev_imgR);

    // Get initial image disparity
    cv::Mat scene3D;
    compute3DScene(stereo_cam, prev_imgL, prev_imgR, scene3D);
    //for (int i = 0; i < stereo_cam.imageSize.height; i++)
    //{
    //    for(int j = 0; j < stereo_cam.imageSize.width; j++)
    //    {
    //        float z = scene3D.at<cv::Vec3f>(i,j)[2];
    //        if (z != 10000 && z != std::numeric_limits<float>::infinity())
    //            std::cout << "Not 10000: " << scene3D.at<cv::Vec3f>(i,j) << std::endl;
    //        //std::cout << scene3D.at<cv::Vec3f>(i,j) << std::endl;
    //    }
    //}

    // Get initial Limg key points and descriptors
    std::vector<cv::KeyPoint> prev_keypoints, current_keypoints;
    cv::Mat prev_descriptors, current_descriptors;
    detector->detectAndCompute(prev_imgL, cv::noArray(), prev_keypoints, prev_descriptors);

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
        matcher->knnMatch(prev_descriptors, current_descriptors, rawMatches, 2);

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
        int count = 0;
        for (int i = 0; i < point_matches_previous.size(); i++)
        {
            int x = point_matches_previous[i].x;
            int y = point_matches_previous[i].y;

            cv::Vec3f point3D = scene3D.at<cv::Vec3f>(y,x);
            float z = point3D[2];
            if (z != 10000 && z != std::numeric_limits<float>::infinity())
            {
                points3D.emplace_back(point3D[0], point3D[1], z);
                points2D.emplace_back(x, y);
                count++;
                if (count == 4) break;
            }

        }

        // SolvePnP to get the Homogeneous transform
        cv::Mat rVec, tVec;
        cv::solvePnP(points3D, points2D, stereo_cam.L_cam.K, stereo_cam.L_cam.D, rVec, tVec);

        //std::vector<std::pair<cv::Point2f, cv::Point2f> > pointPairs;
        //std::vector<uchar> status;
        //// IGNORE RANSAC TIS SHEEET
        //cv::Mat H = cv::findHomography(p2, p1, status, cv::RANSAC, 100.0);
        //std::vector<cv::DMatch> finalMatches;
        //int inliers = 0;
        //for (size_t i = 0; i < status.size(); i++)
        //{
        //    if (status[i])
        //    {
        //        //pointPairs.push_back(std::make_pair(kp1[i], kp2[i]));
        //        finalMatches.push_back(knnMatches[i]);

        //    }
        //}


        // use rodriguez to get the rotation and transformation matrix

        // Apply the rotation and translation to the global pose

        // Print the global pose

        // Display the matches between the previous frame and current frame
        cv::Mat img_match;
        drawMatches ( prev_imgL, prev_keypoints, current_imgL, current_keypoints, knnMatches, img_match );
        imshow ( "KNN Matches", img_match );
        cv::waitKey(0);

        // Set prev data to current to process next frame
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
    }

}
