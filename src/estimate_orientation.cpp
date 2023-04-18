
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <fstream>
#include <eigen3/Eigen/Core>
#include "Fusion/Fusion.h"


#define SAMPLE_RATE (100)
#define SAMPLE_PERIOD (1.0f/SAMPLE_RATE)
typedef Eigen::Vector3d v3d;
typedef Eigen::Vector3f v3f;

using namespace std;

template <typename T>
T deg2rad(const T& acc){
    const double scale = 180.0f / M_PI;
    T out;
    out.x = acc.x*scale;
    out.y = acc.y*scale;
    out.z = acc.z*scale;
    return out;
}


void loadIMU(string filename, vector<double>& ts, vector<v3f>& gyros, vector<v3f>& accs){
    ifstream inFile(filename);
    // vector<v3d> eulars, translations;
    string lineStr;
    int skip_rows = 1;
    int skip_counter = 0;
    while(getline(inFile,lineStr)){
        if (skip_counter++ < skip_rows)
            continue;;

        stringstream ss(lineStr);
        string str;
        vector<float> data;
        while(getline(ss,str,',')){
            data.push_back(stof(str));
        }
        ts.push_back(data[0]);
        gyros.emplace_back(data[8],data[9],data[10]);
        accs.emplace_back(data[11],data[12],data[13]);
    }
    cout << "Loaded: " << ts.size() << " data from file: " << filename << endl;
}


void saveEstimator(string filename, const vector<FusionEuler>& eulars){
    ofstream of(filename);
    of << "# ts, roll, pitch, yaw(degree)," << endl;
    for(int i=0; i<eulars.size(); ++i){
        double ts = 1e-2*i;
        FusionEuler e = eulars[i];
        of << ts << "," << e.angle.roll << "," << e.angle.pitch << "," << e.angle.yaw << endl;
    }
    of.close();
    cout << "Saved " << eulars.size() << " data to file: " << filename << endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "imu_pose_estimator");
	ros::NodeHandle nh;
    ROS_WARN("Start IMU pose estimator.");

    FusionAhrs ahrs;
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

    vector<v3f> accs, gyros;
    vector<double> ts;
    vector<FusionEuler> eulars;
    string filename = "/home/larrydong/imu_ws/src/imu_simulation/data/imu.csv";
    loadIMU(filename, ts, gyros, accs);
    
    for(int i=0; i<ts.size(); ++i){
        v3f gyro = gyros[i];
        v3f acc = accs[i];
        FusionVector gyroscope = {gyro[0], gyro[1], gyro[2]};
        FusionVector accelerometer = {acc[0], acc[1], acc[2]};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        eulars.push_back(euler);
    }

    string save_filename = "/home/larrydong/imu_ws/src/imu_simulation/data/imu_pose_estimation.csv";
    saveEstimator(save_filename, eulars);

    return 0;
}

