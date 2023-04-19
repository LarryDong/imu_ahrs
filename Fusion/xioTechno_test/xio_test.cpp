#include "Fusion/Fusion.h"
#include <stdio.h>

int main()
{
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    FILE *const inputFile = fopen("/home/larrydong/imu_ws/src/imu_ahrs/sensor_data.csv", "r");
    FILE *const outputFile = fopen("/home/larrydong/imu_ws/src/imu_ahrs/main.csv", "w");

    char csvLine[256];
    while (fgets(csvLine, sizeof(csvLine), inputFile) != NULL)
    {
        float timestamp;
        FusionVector gyroscope;
        FusionVector accelerometer;
        if (sscanf(csvLine, "%f,%f,%f,%f,%f,%f,%f",
                   &timestamp,
                   &gyroscope.axis.x,
                   &gyroscope.axis.y,
                   &gyroscope.axis.z,
                   &accelerometer.axis.x,
                   &accelerometer.axis.y,
                   &accelerometer.axis.z) != 7)
        {
            continue;
        }

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0f / 100.0f); // 100 Hz sample rate

        const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

        fprintf(outputFile, "%f,%f,%f,%f,%f\n",
                timestamp,
                quaternion.element.w,
                quaternion.element.x,
                quaternion.element.y,
                quaternion.element.z);
    }

    fclose(inputFile);
    fclose(outputFile);

    return 0;
}