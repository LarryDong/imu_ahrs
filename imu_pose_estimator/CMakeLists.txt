cmake_minimum_required(VERSION 3.0.2)
project(imu_pose_estimator)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package()

include_directories(
    src/Fusion
    ${catkin_INCLUDE_DIRS}
)
message(WARNING "msg")

file(GLOB_RECURSE FusionFiles "${PROJECT_SOURCE_DIR}/Fusion*.c")
message(WARNING "msg: ${FusionFiles}")

add_executable(imu_pose_estimator 
    src/main.cpp
    ${FusionFiles}
)
target_link_libraries(imu_pose_estimator ${catkin_LIBRARIES})
