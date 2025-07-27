#include "angle.h"
#include "AllHeader.h"

float gyro_angle_prev = 0;

float calc_angle(float* mpu_data)
{
    float gyro_x = mpu_data[3];
    // float gyro_y = mpu_data[4];
    // float gyro_z = mpu_data[5];
    // float acc_x = mpu_data[6];
    float acc_y = mpu_data[7];
    float acc_z = mpu_data[8];

    float acc_angle = 0;
    if(acc_z != 0) {acc_angle = atan(acc_y / acc_z) * 180 / PI;}
    
    if (acc_z < 0 && acc_angle > 0) {acc_angle = -90;} 
    else if (acc_z < 0 && acc_angle < 0) {acc_angle = 90;}

    float gyro_angle = gyro_angle_prev + gyro_x / 6855.0;

    float comp_angle = 0.1 * acc_angle + 0.9 * gyro_angle;
    gyro_angle_prev = comp_angle;
    
    // printf("%.1f\r\n", comp_angle);
    return comp_angle;
}
