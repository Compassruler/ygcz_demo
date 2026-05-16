#ifndef _FILTER_H_
#define _FILTER_H_

#include "zf_common_headfile.h"

#define IMU_DT      (0.01f)      // 10ms
#define RAD_TO_DEG  (57.295779f)

typedef struct
{
    float gyro_ration;      // 陀螺仪权重
    float acc_ration;       // 加速度权重
    float angle_temp;       // 中间姿态角
    float call_cycle;       // 调用周期（秒）
    float mechanical_zero;  // 零点补偿
    float filtering_angle;   // 最终输出角度
    
} cascade_common_value_struct;

extern cascade_common_value_struct pitch_filter;
extern cascade_common_value_struct roll_filter;
extern cascade_common_value_struct yaw_filter; 

// 函数简介：参数初始化
void filter_init();

// 函数简介：一阶互补滤波
// 参数说明：filter_value   滤波参数结构体
// 参数说明：gyro_data      陀螺仪角速度数据
// 参数说明：acc_data       加速度解算出来的角度数据
// 返回参数：void
void first_order_complementary_filtering(cascade_common_value_struct *filter, float gyro_data, float acc_data);

#endif