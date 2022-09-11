cmake_minimum_required(VERSION 3.16)
project(StereoSLAM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

add_executable(stereoSLAM ${SRCS})
target_link_libraries(stereoSLAM PRIVATE ${OpenCV_LIBS})
target_include_directories(stereoSLAM PRIVATE ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
#target_include_directories(stereoSLAM PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
