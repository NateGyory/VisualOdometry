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

        std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

        cv::Mat descriptors_1, descriptors_2;
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

        cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

        detector->detect ( img1, keypoints_1 );
        detector->detect ( img2, keypoints_2 );

        descriptor->compute ( img1, keypoints_1, descriptors_1 );
        descriptor->compute ( img2, keypoints_2, descriptors_2 );

        cv::Mat outimg1;
        drawKeypoints( img1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
        imshow("ORB Keypoints",outimg1);

        std::vector<cv::DMatch> matches;
        matcher->match ( descriptors_1, descriptors_2, matches );

        double min_dist=10000, max_dist=0;

        for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = matches[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        printf ( "-- Max dist : %f \n", max_dist );
        printf ( "-- Min dist : %f \n", min_dist );

        std::vector< cv::DMatch > good_matches;
        for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            if ( matches[i].distance <= std::max ( 2*min_dist, 30.0 ) )
            {
                good_matches.push_back ( matches[i] );
            }
        }

        cv::Mat img_match;
        cv::Mat img_goodmatch;
        drawMatches ( img1, keypoints_1, img2, keypoints_2, matches, img_match );
        drawMatches ( img1, keypoints_1, img2, keypoints_2, good_matches, img_goodmatch );
        imshow ( "All Matches", img_match );
        imshow ( "Good Matches", img_goodmatch );
        cv::waitKey(0);
        // get pose from previous keyframe
    }

}
