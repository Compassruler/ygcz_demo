#include "zf_common_headfile.h"
#define LED1                    (P19_0)     // SPI 涓插彛 SPI 涓わ拷?锟藉睆 杩欓噷瀹忓畾涔夊～锟?? IPS200_TYPE_SPI

char txt[128];

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M);                        // 鏃堕挓閰嶇疆鍙婄郴缁熷垵濮嬪寲<鍔″繀淇濈暀>
  debug_init();                                         // 璋冭瘯涓插彛鍒濓拷?锟藉寲
  //wireless_uart_init();                                 // 鏃犵嚎涓插彛鍒濓拷?锟藉寲
  servo_init();                                         // 舵机初始化
  filter_init();                                        // 滤波初始化
  banlance_init();                                      // PID参数初始化
  imu660rb_init();                                      // IMU初始化
  small_driver_uart_init();                             // 电机驱动初始化
  pit_ms_init(PIT_CH0,1);                               // PIT中断初始化

  screen_init();                                        // 屏幕初始化
//  remote_control_init();                                // 遥控器初始化
  wireless_uart_init();
  road_memery_start_flag = 1;
  road_memery_finish_flag = 0;
  road_recurrent_flag = 0;
int i = 0;
  
  while(true)
  { 
                      
    
     
      
      
    
//    remote_update();   // 遥控器状态更新
//   
//
//     // 判断遥控器是否触发记录开始/结束
//    rc_state=remote_left_01_switch_ctrl();    
//    
//        screen_data_item_t imu_table[] =
//      {
//          {"state",  SCREEN_DATA_INT,   {.int_value = 0},   0},
//          {"x",  SCREEN_DATA_INT,   {.int_value = 0},   0},
//          {"y",  SCREEN_DATA_INT,   {.float_value = 0},   0},
//          {"yaw", SCREEN_DATA_INT,   {.float_value = 0},   0},
//          {"rc_state", SCREEN_DATA_INT,   {.int_value = 0},   0},
//          {"gyro_z", SCREEN_DATA_INT,   {.int_value = 0},   0},
//          {"state",  SCREEN_DATA_STRING,{.str_value = "OK"},0},
//      };
//   
//    imu_table[0].value.int_value = FLASH_PAGE_LENGTH;
//    imu_table[1].value.int_value = i;
//    imu_table[2].value.int_value = 0;
//    imu_table[3].value.float_value = 0;
//   
//    screen_show_data_table(imu_table,2);
    
    if(i>=FLASH_PAGE_LENGTH * 6)
    {
      i = FLASH_PAGE_LENGTH * 6 -1;
    }
    sprintf(txt,
            "(x|y):(%f,%f)\r\n",X_remenber[i],Y_remenber[i]);
    
//    sprintf(txt,
//            "vl|vr:%d,%d\r\n",small_driver_value.receive_left_speed_data,small_driver_value.receive_right_speed_data);
    i++;
    
//    sprintf(txt,
//            "true_speed:%f\r\n",(rpmtotrue(-small_driver_value.receive_left_speed_data) + 
//                    rpmtotrue(small_driver_value.receive_right_speed_data)) / 2);
//    
   
//    sprintf(txt,"1111:%.2f,%f\r\n", yaw_angle, yaw_angle_pid.output);
    wireless_uart_send_string(txt);
    
    
    
    

//    sprintf(txt, "t_speed|r_speed: %d, %d,%f\r\n", 10, (small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2, -speed_pid.output); // 閫熷害锟??杈撳嚭
//    sprintf(txt,"1111:%f,%f,%f,%f,%f\r\n",servoLeftFront_now, servoLeftRear_now, servoRightFront_now, servoRightRear_now,speed_to_x_offset);
//    sprintf(txt,"1111:%d,%d,%f\r\n",-small_driver_value.receive_left_speed_data, small_driver_value.receive_right_speed_data, (float)(-small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2);
    
    //wireless_uart_send_string(txt);
    system_delay_ms(20);
  }
}