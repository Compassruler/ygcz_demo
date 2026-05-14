#include "zf_common_headfile.h"

cascade_common_value_struct pitch_filter;    //结构体变量 俯仰角
cascade_common_value_struct roll_filter;     //结构体变量 横滚角

// 先放在这里，后续移到别的pid相关.c文件去
void parameter_init(void)
{
  pitch_filter.gyro_ration = 4.0f;
  pitch_filter.acc_ration  = 4.0f;
  pitch_filter.angle_temp  = 0.0f;
  pitch_filter.call_cycle  = 0.005f; // 5ms
  pitch_filter.mechanical_zero = -4.0f;         // 机械零点（重要！）
  pitch_filter.filtering_angle = 0.0f;
}

void first_order_complementary_filtering(cascade_common_value_struct *filter_value, float gyro_data, float acc_data)
{
  float gyro_temp;    // 角速度计算临时变量
  float acc_temp;     // 加速度计算临时变量
  gyro_temp = gyro_data * filter_value->gyro_ration;      // 角速度数据 * 角速度置信度（一般给4）
  acc_temp = (acc_data - filter_value->angle_temp) * filter_value->acc_ration;    // 加速度误差 * 加速度置信度（一般给4）
  filter_value->angle_temp += ((gyro_temp + acc_temp) * filter_value->call_cycle);    // 积分得到角度输出
  filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero; // 最终滤波角度 + 机械零点补偿
}
