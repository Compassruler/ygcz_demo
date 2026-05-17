#include "zf_common_headfile.h"

cascade_common_value_struct pitch_filter;    // 俯仰轴互补滤波参数
cascade_common_value_struct roll_filter;     // 横滚轴互补滤波参数
cascade_common_value_struct yaw_filter;      // 偏航轴互补滤波参数

void filter_init(void)
{
  pitch_filter.gyro_ration = 4.0f;
  pitch_filter.acc_ration  = 4.0f;
  pitch_filter.angle_temp  = 0.0f;
  pitch_filter.call_cycle  = 0.005f; // 5ms
  pitch_filter.mechanical_zero = -5.0f; // 机械零偏
  pitch_filter.filtering_angle = 0.0f;
  
  roll_filter.gyro_ration = 4.0f;
  roll_filter.acc_ration  = 4.0f;
  roll_filter.angle_temp  = 0.0f;
  roll_filter.call_cycle  = 0.005f; // 5ms
  roll_filter.mechanical_zero = 0.0f;  // 机械零偏
  roll_filter.filtering_angle = 0.0f;
  
}

void first_order_complementary_filtering(cascade_common_value_struct *filter_value, float gyro_data, float acc_data)
{
  float gyro_temp;    // 陀螺角速度临时值
  float acc_temp;     // 加速度角度临时值

  gyro_temp = gyro_data * filter_value->gyro_ration;      // 陀螺数据 * 陀螺系数
  acc_temp = (acc_data - filter_value->angle_temp) * filter_value->acc_ration;    // (加速度数据 - 上次角度) * 加速度系数

  filter_value->angle_temp += ((gyro_temp + acc_temp) * filter_value->call_cycle);    // 更新滤波角度
  filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero; // 输出滤波角度 + 机械零偏
}
