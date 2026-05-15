#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"

// IMU原始数据
typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

} imu_raw_data_t;

// IMU物理数据 
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
extern float pitch_acc2angle;   // 俯仰加速度转角度数据
extern float roll_acc2angle;    // 横滚加速度转角度数据
extern float yaw_acc2angle;     // 航向加速度转角度数据

// 函数简介：获取imu数据（初步偏移处理后的数据）
void imu_data_get(void);

// 函数简介：imu数据转化物理数据
void imu_data_transition(void); 

// 函数简介：将加速度转换成角度
// 参数说明：acc_main        主轴的物理加速度数据
// 参数说明：acc_oth1        另一个轴的物理加速度数据
// 参数说明：acc_oth2        另一个轴的物理加速度数据
float imu_acc2angle(float acc_main, float acc_oth1, float acc_oth2);

// 函数简介：集成了前几个函数算了一下pitch的加速度转角度数据
void imu_update(void);

#endif