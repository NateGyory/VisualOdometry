cmake_minimum_required(VERSION 3.16)
project(StereoSLAM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin PATHS /home/nate/Build/Pangolin-0.6/install)
find_package(OpenGL REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

add_executable(stereoSLAM ${SRCS})
target_link_libraries(stereoSLAM PRIVATE ${OpenCV_LIBS} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(stereoSLAM PRIVATE ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
