#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"
#include <stdint.h>

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


// 函数简介：获取imu数据（初步偏移处理后的数据）
void imu_data_get(void);

// 函数简介：imu数据转化物理数据
void imu_data_transition(void);

#endif