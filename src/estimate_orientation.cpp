
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <fstream>
#include <eigen3/Eigen/Core>
#include "Fusion/Fusion.h"

#include <stdio.h>

#define SAMPLE_RATE (100)
#define SAMPLE_PERIOD (1.0f/SAMPLE_RATE)
#define Gravity (9.81f)
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
        accs.emplace_back(data[11],data[12],data[13]);       // Gravity

        // gyros.emplace_back(data[1],data[2],data[3]);
        // accs.emplace_back(data[4],data[5],data[6]);
    }
    cout << "Loaded: " << ts.size() << " data from file: " << filename << endl;
}


void saveEstimator(string filename, const vector<FusionEuler>& eulars, const vector<FusionQuaternion>& quartarions){
    ofstream of(filename);
    // of << "# ts, roll, pitch, yaw(degree)," << endl;
    of << "# ts, qw, qx, qy, qz(g). " << endl;
    for(int i=0; i<eulars.size(); ++i){
        double ts = 1e-2*i;
        FusionEuler e = eulars[i];
        auto q = quartarions[i].element;
        of << ts << "," << e.angle.roll << "," << e.angle.pitch << "," << e.angle.yaw << endl;
        // of << ts << "," << q.w << "," << q.x << "," << q.y << "," << q.z << "," << endl;
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
            .accelerationRejection = 0.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    vector<v3f> accs, gyros;
    vector<double> ts;
    vector<FusionEuler> eulars;
    vector<FusionQuaternion> quartarions;
    string filename = "/home/larrydong/imu_ws/src/imu_simulation/data/imu.csv";
    string save_filename = "/home/larrydong/imu_ws/src/imu_simulation/data/imu_pose_estimation.csv";
    // FILE *const outputFile = fopen(save_filename.c_str(), "w");

    loadIMU(filename, ts, gyros, accs);
    
    for(int i=0; i<ts.size(); ++i){
        auto gyro = gyros[i];
        auto acc = accs[i];
        FusionVector gyroscope = {gyro[0], gyro[1], gyro[2]};
        FusionVector accelerometer = {acc[0], acc[1], acc[2]};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionQuaternion q =  FusionAhrsGetQuaternion(&ahrs);
        const FusionEuler euler = FusionQuaternionToEuler(q);
        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        eulars.push_back(euler);
        quartarions.push_back(q);

        // fprintf(outputFile, "%f,%f,%f,%f,%f\n",
        //     ts[i],
        //     q.element.w,
        //     q.element.x,
        //     q.element.y,
        //     q.element.z);
    }
    // fclose(outputFile);

    saveEstimator(save_filename, eulars, quartarions);
    return 0;
}

