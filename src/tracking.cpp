#include "tracking.h"

void TrackPose(StereoCamera &stereo_cam)
{
    // Loop over images
    for(int i = 0; i < stereo_cam.imgStream.left_images.size(); i++)
    {
        // TODO: Get snap shot after 3 seconds
        // Create a key frame
        cv::Mat img1 = cv::imread(stereo_cam.imgStream.left_images[i]);
        cv::Mat img2 = cv::imread(stereo_cam.imgStream.right_images[i]);

        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

        detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);


        std::vector<std::vector<cv::DMatch>> rawMatches;
        std::vector<cv::DMatch> knnMatches;
        std::vector<cv::Point2f> p1, p2;
        std::vector<float> distances;
        matcher->knnMatch(descriptors1, descriptors2, rawMatches, 2);
        
        //matcher->match ( descriptors_1, descriptors_2, matches );

        for (size_t i = 0; i < rawMatches.size(); i++)
        {
            const std::vector<cv::DMatch>& m = rawMatches[i];
            if (m[0].distance < m[1].distance * 0.5)
            {
                p2.push_back(keypoints2[m[0].queryIdx].pt);
                p1.push_back(keypoints1[m[0].trainIdx].pt);
                knnMatches.push_back(m[0]);
            }
        }
        //std::vector<std::pair<cv::Point2f, cv::Point2f> > pointPairs;
        std::vector<uchar> status;
        // IGNORE RANSAC TIS SHEEET
        cv::Mat H = cv::findHomography(p2, p1, status, cv::RANSAC, 100.0);
        std::vector<cv::DMatch> finalMatches;
        int inliers = 0;
        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i])
            {
                //pointPairs.push_back(std::make_pair(kp1[i], kp2[i]));
                finalMatches.push_back(knnMatches[i]);

            }
        }

        cv::Mat img_match;
        cv::Mat img_goodmatch;
        drawMatches ( img1, keypoints1, img2, keypoints2, knnMatches, img_match );
        drawMatches ( img1, keypoints1, img2, keypoints2, finalMatches, img_goodmatch );
        imshow ( "KNN Matches", img_match );
        imshow ( "KNN + RANSAC Matches", img_goodmatch );
        cv::waitKey(0);
        // get pose from previous keyframe
    }

}
