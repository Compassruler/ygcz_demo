#include "zf_common_headfile.h"
#define LED1                    (P19_0)                                         // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI

char txt[128];
uint8 communication_count = 0; 

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); 	                       // 时钟配置及系统初始化<务必保留>
  debug_init();                                                   // 调试串口信息初始化
  wireless_uart_init();
  servo_init();                                                   // 舵机初始化
  parameter_init();                                              // 姿态解算参数初始化
  imu660rb_init();
  small_driver_uart_init();                                      // 电机初始化
  pid_init(&gyro_pid, 30.0f, 0.0f, 0.0f, 0, 10000, 1.0f);      // 角速度pid初始化//30
  pid_init(&angle_pid, 17.0f, 0.0f, 0.0f, 0, 10000, 1.0f);   // 角度pid初始化//17
  pid_init(&speed_pid, 0.01f, 0.0f, 0.0f, 0, 10000, 1.0f);    // 速度pid初始化
  pit_ms_init(PIT_CH0,1);                                      // 角速度中断，周期1ms
  
   while(true)
    { 
//       sprintf(txt,"gyro_y|output|angle|output:%f, %f,%f,%f\r\n",imu_data.gyro_y,gyro_pid.output ,pitch_filter.filtering_angle,angle_pid.output);
       sprintf(txt,"t_speed|r_speed: %d, %d,%f\r\n",10,(small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2,-speed_pid.output);
       wireless_uart_send_string(txt);
       system_delay_ms(20);
    }
}
