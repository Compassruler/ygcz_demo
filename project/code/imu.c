#include "zf_common_headfile.h"

imu_raw_data_t imu_raw;         // 结构体变量
imu_data_t imu_data;

float pitch_acc2angle = 0;
float roll_acc2angle  = 0;
float yaw_acc2angle   = 0;

// imu_raw.gyro_x 横滚角 左正右负
// imu_raw.gyro_y 俯仰角 前正后负
// imu_raw.gyro_z 偏航角 左正右负

void imu_data_get(void)
{
    imu660rb_get_acc(); // 加速度数据
    imu660rb_get_gyro();// 陀螺仪数据

    imu_raw.acc_x = imu660rb_acc_x-160; // 偏移处理
    imu_raw.acc_y = imu660rb_acc_y-440;
    imu_raw.acc_z = imu660rb_acc_z;
    imu_raw.gyro_x = imu660rb_gyro_x - 5;
    imu_raw.gyro_y = imu660rb_gyro_y + 6;
    imu_raw.gyro_z = imu660rb_gyro_z + 5;

    if(func_abs(imu_raw.acc_x) <= 5)    // 死区处理
        imu_raw.acc_x = 0;

    if(func_abs(imu_raw.acc_y) <= 5)
        imu_raw.acc_y = 0;

    if(func_abs(imu_raw.acc_z) <= 5)
        imu_raw.acc_z = 0;

    if(func_abs(imu_raw.gyro_x) <= 5)
        imu_raw.gyro_x = 0;

    if(func_abs(imu_raw.gyro_y) <= 5)
        imu_raw.gyro_y = 0;

    if(func_abs(imu_raw.gyro_z) <= 5)
        imu_raw.gyro_z = 0;
}

void imu_data_transition(void)
{
    imu_data.acc_x = imu660rb_acc_transition(imu_raw.acc_x);    // 测量数据转化为物理加速度数据
    imu_data.acc_y = imu660rb_acc_transition(imu_raw.acc_y);
    imu_data.acc_z = imu660rb_acc_transition(imu_raw.acc_z);

    imu_data.gyro_x = imu660rb_gyro_transition(imu_raw.gyro_x);   // 测量数据转化为物理陀螺仪数据
    imu_data.gyro_y = imu660rb_gyro_transition(imu_raw.gyro_y);
    imu_data.gyro_z = imu660rb_gyro_transition(imu_raw.gyro_z);
}

float imu_acc2angle(float acc_main, float acc_oth1, float acc_oth2)
{
  float angle;
  angle = 0;
  angle = atan2f(acc_main,sqrtf(acc_oth1 * acc_oth1 + acc_oth2 * acc_oth2)) * 57.3f;
  return angle;
}

void imu_update(void)
{
//  imu_data_get();               // 原始数据
//  imu_data_transition();        // 转换后数据
  pitch_acc2angle = imu_acc2angle(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
  first_order_complementary_filtering(&pitch_filter, imu_data.gyro_y, pitch_acc2angle);        // 一阶互补滤波处理，这里输出pitch_filter.filtering_angle角
}