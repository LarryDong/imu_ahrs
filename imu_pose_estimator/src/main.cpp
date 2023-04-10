
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "Fusion/Fusion.h"

#define SAMPLE_RATE (100)
#define SAMPLE_PERIOD (1.0f/SAMPLE_RATE)

using namespace std;

FusionAhrs ahrs;

template <typename T>
T deg2rad(const T& acc){
    const double scale = 180.0f / M_PI;
    T out;
    out.x = acc.x*scale;
    out.y = acc.y*scale;
    out.z = acc.z*scale;
    return out;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
    auto acc = msg->linear_acceleration;
    auto gyro = deg2rad(msg->angular_velocity);
    ROS_INFO("imuCallback: acc: %f, %f, %f, \t, w: %f, %f, %f", acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
    const FusionVector gyroscope = {gyro.x, gyro.y, gyro.z}; // replace this with actual gyroscope data in degrees/s
    const FusionVector accelerometer = {acc.x, acc.y, acc.z}; // replace this with actual accelerometer data in g

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "imu_pose_estimator");
	ros::NodeHandle nh;
    ROS_WARN("Start IMU pose estimator.");

    // subscribe the imu from y
    ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1000, imuCallback);

    FusionAhrsInitialise(&ahrs);
    // Set AHRS algorithm settings after initialization.
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    ros::Rate r(10000);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

