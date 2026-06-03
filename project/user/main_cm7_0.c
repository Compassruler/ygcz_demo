#include "zf_common_headfile.h"
#define LED1                    (P19_0)     // SPI 涓插彛 SPI 涓わ拷?锟藉睆 杩欓噷瀹忓畾涔夊～锟?? IPS200_TYPE_SPI
#define BUZZER_PIN              (P19_4)    
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
  gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);  // 蜂鸣器初始化
  screen_init();                                        // 屏幕初始化
  flash_init();
  remote_control_init();                                // 遥控器初始化
  wireless_uart_init();
 // road_memery_start_flag = 1;
 // road_memery_finish_flag = 0;
 // road_recurrent_flag = 0;
int i = 0;
  flash_yaw_flag = 0;
  while(true)
  { 
                      
   
     if (i == 1000 && flash_yaw_flag == 0)
       flash_yaw_flag = 1;
     
     
     
     switch(flash_yaw_flag) {
    case 1: // 写入
        flash_road_memery_store();
        flash_road_memery_store_Plus();
        flash_yaw_flag = 2;
        break;
    case 2: // 读取
        flash_road_memery_get();
        flash_road_memery_get_Plus();
        gpio_toggle_level(BUZZER_PIN);
        flash_yaw_flag = 3; // 完成状态
        break;
    case 3:
        // 已完成，不再操作
        break;
}
      if(i>=FLASH_PAGE_LENGTH * 6)
    {
      i = FLASH_PAGE_LENGTH * 6 -1;
    }
    
   
      sprintf(txt,
             "(x|y):(%f,%f)\r\n",X_load[i],Y_load[i]);
      
     if(flash_yaw_flag!=2)
       i++;       
     else 
       i--;
     if(i<=0)
       i = 0;
//    sprintf(txt,
//            "vl|vr:%d,%d\r\n",small_driver_value.receive_left_speed_data,small_driver_value.receive_right_speed_data);
   
    
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