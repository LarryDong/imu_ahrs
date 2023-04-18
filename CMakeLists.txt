cmake_minimum_required(VERSION 3.0.2)
project(imu_ahrs)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)
catkin_package()

include_directories(
    Fusion
    ${PROJECT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE FusionFiles "${PROJECT_SOURCE_DIR}/Fusion*.c")

add_executable(estimate_orientation 
    src/estimate_orientation.cpp
    ${FusionFiles}
)
target_link_libraries(estimate_orientation ${catkin_LIBRARIES})

