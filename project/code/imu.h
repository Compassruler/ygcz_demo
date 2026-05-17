#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"

// IMU 原始数据
typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

} imu_raw_data_t;

// IMU 处理后的数据
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

} imu_data_t;

extern imu_raw_data_t imu_raw;
extern imu_data_t imu_data;    
extern float pitch_acc2angle;   // 俯仰轴加速度转角度数据
extern float roll_acc2angle;    // 横滚轴加速度转角度数据
extern float yaw_angle;         // 偏航角度数据，单位度

// 函数声明：获取 IMU 数据，并进行前期处理
void imu_data_get(void);

// 函数声明：对 IMU 数据进行标定转换
void imu_data_transition(void); 

// 函数声明：将加速度数据转换为角度
// 参数说明：acc_main  主方向加速度数据
// 参数说明：acc_oth1  另一个方向加速度数据
// 参数说明：acc_oth2  另一个方向加速度数据
float imu_acc2angle(float acc_main, float acc_oth1, float acc_oth2);

// 函数声明：更新 IMU 处理流程，执行互补滤波
void imu_update(void);

#endif