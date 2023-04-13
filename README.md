# Visual Odometry

## Demo


https://user-images.githubusercontent.com/45575958/231634305-64fa44d4-291a-445a-9b11-a670eed14283.mp4



## Details
This codebase implements a visual odometry system for a drone with stereo camera visual sensor system. The images are from MAV0 in the EuRoC dataset. The steps to compute the visual odometry are

1. Undistort & rectify stereo image pairs using the lens interisic and extrinsic properties
2. Compute the disparity image
3. Find ORB feature matches between the current and previous frame
4. Solve for the Rotation and Translation between frames using the PnP algorithm
5. Publish an updated ROS trajectory message with the new pose

## Software used
1. OpenCV
2. ROS
3. Eigen
