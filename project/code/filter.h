#ifndef _FILTER_H_
#define _FILTER_H_

#include "zf_common_headfile.h"

#define IMU_DT      (0.01f)      // 10ms
#define RAD_TO_DEG  (57.295779f)

typedef struct
{
    float gyro_ration;      // 陀螺系数
    float acc_ration;       // 加速度系数
    float angle_temp;       // 临时角度
    float call_cycle;       // 调用周期，单位秒
    float mechanical_zero;  // 机械零偏
    float filtering_angle;  // 最终滤波角度
    
} cascade_common_value_struct;

extern cascade_common_value_struct pitch_filter;
extern cascade_common_value_struct roll_filter;
extern cascade_common_value_struct yaw_filter; 

// 初始化滤波器参数
void filter_init(void);

// 一阶互补滤波
// 参数说明：filter - 互补滤波参数结构体
// 参数说明：gyro_data - 陀螺角速度数据
// 参数说明：acc_data - 加速度计算得到的角度数据
// 返回值：void
void first_order_complementary_filtering(cascade_common_value_struct *filter, float gyro_data, float acc_data);

#endif