#include "zf_common_headfile.h"
#define LED1                    (P19_0)     // SPI 涓插彛 SPI 涓わ拷?锟藉睆 杩欓噷瀹忓畾涔夊～锟?? IPS200_TYPE_SPI
#define BUZZER_PIN              (P19_4)    

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

char txt[128];

/*测试flash写入读取，眉毛
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);                        // 时钟配置
    debug_init();                                         // 调试串口初始化
    servo_init();                                         // 舵机初始化
    filter_init();                                        // 滤波初始化
    banlance_init();                                      // PID 参数初始化
    imu660rb_init();                                      // IMU 初始化
    small_driver_uart_init();                             // 电机驱动初始化
    pit_ms_init(PIT_CH0, 1);                              // PIT 中断初始化
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);  // 蜂鸣器初始化
    screen_init();                                        // 屏幕初始化
    flash_init();                                         // FLASH 初始化
    ins_init();                                           // INS 初始化
    wireless_uart_init();                                 // 无线串口初始化

    road_memery_start_flag = 1;
    road_memery_finish_flag = 0;
    road_recurrent_flag = 0;
    flash_xy_flag = 0;
    int i = 0;
    bool flash_write_requested = false;
    bool flash_read_requested  = false;
    bool flash_store_done      = false;
    bool flash_load_done       = false;
    int display_index          = 0;
    float write_data[5] =
    {
        1.23f,
        2.34f,
        3.45f,
        4.56f,
        5.67f
    };   //测试数据
    flash_buffer_clear();
    for(int i = 0; i < 5; i++)
    {
        flash_union_buffer[i].float_type = write_data[i];
    }
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_9))
    {
        flash_erase_page(FLASH_SECTION_INDEX, X_memery_page_INDEX_9);
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_9, 5);
    system_delay_ms(100);
    if (flash_check(FLASH_SECTION_INDEX, X_memery_page_INDEX_9))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, X_memery_page_INDEX_9, FLASH_PAGE_LENGTH);
        for (size_t i = 0 ; i < 4; i++)
        {
            X_load[i] = flash_union_buffer[i].float_type;
        }
    }   
    while(true)
    {
      
      
      sprintf(txt, "FLASH data: %f\r\n", 
              X_load[i]);
      wireless_uart_send_string(txt);
       i++;
      if(i>4)
        i= 0;
      system_delay_ms(20);
    }
}
*/
/*按键控制标志位实现惯导数据记录，flash存入与读取
int main(void)
{
    //==========================
    // 初始化
    //==========================
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    servo_init();
    filter_init();
    banlance_init();

    imu660rb_init();
    small_driver_uart_init();

    pit_ms_init(PIT_CH0, 1);

    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    screen_init();

    flash_init();

    ins_init();

    wireless_uart_init();
    
    remote_control_init();
    //==========================
    // 按键初始化
    //==========================
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

    //==========================
    // 变量
    //==========================
    int i = 0;

    uint8 key1_last = 1;
    uint8 key2_last = 1;

    uint8 key1_now;
    uint8 key2_now;

    //==========================
    // 主循环
    //==========================
    while(true)
    {
        //==========================
        // 读取当前按键状态
        //==========================
        key1_now = gpio_get_level(KEY1);
        key2_now = gpio_get_level(KEY2);

        //==================================================
        // KEY1：按下一次 -> 写入Flash
        //==================================================
        if(key1_last == 1 && key1_now == 0)
        {
            sprintf(txt, "FLASH STORE...\r\n");
            wireless_uart_send_string(txt);

            // 写入Flash
            flash_road_memery_store_Plus();

            flash_yaw_flag = 1;

            sprintf(txt, "FLASH STORE DONE\r\n");
            wireless_uart_send_string(txt);
        }

        //==================================================
        // KEY2：按下一次 -> 读取Flash
        //==================================================
        if(key2_last == 1 && key2_now == 0)
        {
            sprintf(txt, "FLASH LOAD...\r\n");
            wireless_uart_send_string(txt);

            // 读取Flash
            flash_road_memery_get_Plus();

            flash_yaw_flag = 2;

            // 从头开始显示
            i = 0;

            sprintf(txt, "FLASH LOAD DONE\r\n");
            wireless_uart_send_string(txt);
        }

        //==========================
        // 更新按键历史状态
        //==========================
        key1_last = key1_now;
        key2_last = key2_now;

        //==================================================
        // 打印读取后的数据
        //==================================================
        if(flash_yaw_flag == 2)
        {
            if(i < sizeof(X_load) / sizeof(X_load[0]))
            {
                sprintf(txt,
                        "LOAD:(%f,%f)\r\n",
                        
                        X_load[i],
                        Y_load[i]);

                wireless_uart_send_string(txt);
                i++;
            }
            else 
              i = 0;
        }
        if(flash_yaw_flag == 0)
        {
          for(int j = 0; j<FLASH_PAGE_LENGTH*6-1;j++)
          {sprintf(txt,"REMENBER:(x,y):(%f,%f)\r\n",
                    X_remember[j],Y_remember[j]);
          wireless_uart_send_string(txt);
          }
        }
        system_delay_ms(20);  
    }
}*/ 
 /*spi屏幕显示，串口发送
screen_data_item_t remote_table[] =
{
    {"CH1",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH2",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH3",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH4",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH5",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH6",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH7",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH8",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH9",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"CH10", SCREEN_DATA_INT, {.int_value = 0}, 0},
};

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M);                      
  debug_init();                                       
  wireless_uart_init();                                 // 无线串口初始化
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

//int i = 0;
  flash_yaw_flag = 0;
  while(true)
  { 
    remote_update();
    remote_left_02_switch_ctrl();   // 左侧2保护开关初始化
    remote_right_02_switch_ctrl();  // 右侧2跳跃开关初始化

    remote_table[0].value.int_value = remote_get_channel(0); // 右摇杆 左右 992
    remote_table[1].value.int_value = remote_get_channel(1); // 右摇杆 上下 992
    remote_table[2].value.int_value = remote_get_channel(2); // 左摇杆 上下 992
    remote_table[3].value.int_value = remote_get_channel(3); // 左摇杆 左右 940 飘
    remote_table[4].value.int_value = remote_get_channel(4); // 左1开关 上 192 中 992 下 1792
    remote_table[5].value.int_value = remote_get_channel(5); // 左2开关 上192 下 1792
    remote_table[6].value.int_value = remote_get_channel(6); // 右2开关 上192 下 1792
    remote_table[7].value.int_value = remote_get_channel(7); // 右1开关 上 192 中 992 下 1792
    remote_table[8].value.int_value = remote_get_channel(8); // 左旋钮 192-1792
    remote_table[9].value.int_value = remote_get_channel(9); // 有旋钮 192-1792
    screen_show_data_table(remote_table, 10);
    
    wireless_uart_send_string(txt);
    
//    sprintf(txt, "t_speed|r_speed: %d, %d,%f\r\n", 10, (small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2, -speed_pid.output); // 閫熷害锟??杈撳嚭
    sprintf(txt,"1111:%f,%f,%f,%f,%f\r\n",servoLeftFront_now, servoLeftRear_now, servoRightFront_now, servoRightRear_now,speed_to_x_offset);
//    sprintf(txt,"1111:%d,%d,%f\r\n",-small_driver_value.receive_left_speed_data, small_driver_value.receive_right_speed_data, (float)(-small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) / 2);
    
    wireless_uart_send_string(txt);
    system_delay_ms(20);
  }
}*/
int main(void)
{
    //==========================
    // 初始化
    //==========================
    clock_init(SYSTEM_CLOCK_250M);                      
  debug_init();                                       
  wireless_uart_init();                                 // 无线串口初始化
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
  
    //==========================
    // 变量
    //==========================
    int i = 0;
    static int j = 0;


    screen_data_item_t remote_table[] =
{
    {"target_x",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"vx",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"road_memery_flag",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"right_last",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"right_now",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"vy",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"path_index",  SCREEN_DATA_UINT, {.uint_value = 0}, 0},
    {"target_speed",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"distance",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"num", SCREEN_DATA_INT, {.int_value = 0}, 0},
};
    //==========================
    // 主循环
    //==========================
    while(true)
    {
        remote_update();
        remote_left_02_switch_ctrl();   // 左侧2保护开关初始化
        remote_right_02_switch_ctrl();  // 右侧2跳跃开关初始化
//                ins_update();  // ins数据更新       
        //==========================
        // 读取当前遥控器状态
        //==========================
        remote_left_01_switch_ctrl();
        remote_right_01_switch_ctrl();
//        remote_table[0].value.int_value = remote_left_01_last_flag;
//        remote_table[1].value.int_value = remote_left_01_now_flag;
        remote_table[0].value.float_value = target_x;
        remote_table[1].value.float_value = vx;
        remote_table[2].value.int_value = road_memery_flag;
        remote_table[3].value.int_value = remote_right_01_last_flag;
        remote_table[4].value.int_value = remote_right_01_now_flag;
        remote_table[5].value.float_value = vy;
        remote_table[6].value.uint_value = path_index;
        remote_table[7].value.float_value = target_speed;
        remote_table[8].value.float_value = distance;
        remote_table[9].value.int_value = road_destination;
        screen_show_data_table(remote_table, 10);
        //==================================================
        // 01left: 0->1写入flash
        //==================================================
        if(remote_left_01_last_flag == 0 && remote_left_01_now_flag == 1)
        {
            sprintf(txt, "FLASH STORE...\r\n");
            wireless_uart_send_string(txt);
            
            // 写入Flash
            flash_road_memery_store_Plus();
            flash_road_memery_store();
            
            flash_yaw_flag = 1;
      
            sprintf(txt, "FLASH STORE DONE\r\n");
            wireless_uart_send_string(txt);
        }

        //==================================================
        // 01left：0-> 2读取Flash
        //==================================================
        if(remote_left_01_last_flag == 0 && remote_left_01_now_flag == 2)
        {
            sprintf(txt, "FLASH LOAD...\r\n");
            wireless_uart_send_string(txt);
        
            
            // 读取Flash
            flash_road_memery_get_Plus();
            flash_road_memery_get();
            flash_yaw_flag = 2;

            // 从头开始显示
            i = 0;

            sprintf(txt, "FLASH LOAD DONE\r\n");
            wireless_uart_send_string(txt);
        }

        //==========================
        // 更新按键历史状态
        //==========================
        remote_left_01_last_flag = remote_left_01_now_flag;

        //==================================================
        // 打印读取后的数据
        //==================================================
        if(flash_yaw_flag == 2)
        {
            if(i < sizeof(X_load) / sizeof(X_load[0]))
            {
//                sprintf(txt,
//                        "LOAD:(%.3f,%.3f),%d\r\n",
//                        
//                        X_load[i],
//                        Y_load[i],
//                        path_index);
////              
                sprintf(txt,
                        "path:%d)\r\n",path_index);

                wireless_uart_send_string(txt);
                i++;
            }
            else 
              i = 0;
        }
        else if(flash_yaw_flag == 0)
        {
          
          sprintf(txt,"REMENBER:(x,y):(%.3f,%.3f)，%3f,%3f\r\n",
                    X_remember[j],Y_remember[j],target_yaw,yaw_angle);
          j++;          
          wireless_uart_send_string(txt);
          if(j >= sizeof(X_remember)/sizeof(X_remember[0]))
      {
          j = 0;
      }
        }
        system_delay_ms(20);
        sprintf(txt, "target|now:%.6f,%.6f,%.6f\r\n",target_yaw,yaw_angle,yaw_error);
        wireless_uart_send_string(txt);
    }
} 

