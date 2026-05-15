#include "zf_common_headfile.h"
#define LED1                    (P19_0)     // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI

char txt[128];
uint8 communication_count = 0; 

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M);                        // 时钟配置及系统初始化<务必保留>
  debug_init();                                         // 调试串口初始化
  wireless_uart_init();                                 // 无线串口初始化

  servo_init();                                         // 舵机初始化
  parameter_init();                                     // 动态参数初始化
  banlance_init();
  imu660rb_init();                                      // IMU初始化
  small_driver_uart_init();                             // 电机驱动初始化

  pit_ms_init(PIT_CH0,1);                               // PIT中断初始化 周期1ms

  while(true)
  { 
//    sprintf(txt, "t_speed|r_speed: %d, %d,%f\r\n", 10, (small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2, -speed_pid.output); // 速度环输出
    sprintf(txt,"1111:%f,%f,%f,%f,%f\r\n",servoLeftFront_now, servoLeftRear_now, servoRightFront_now, servoRightRear_now,speed_to_x_offset);
//    sprintf(txt,"1111:%d,%d,%f\r\n",-small_driver_value.receive_left_speed_data, small_driver_value.receive_right_speed_data, (float)(-small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2);
    
    wireless_uart_send_string(txt);
    system_delay_ms(20);
  }
}