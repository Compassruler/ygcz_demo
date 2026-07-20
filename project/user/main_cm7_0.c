#include "zf_common_headfile.h"
#define LED1                    (P19_0)     // SPI 涓插彛 SPI 涓わ拷?锟藉睆 杩欓噷瀹忓畾涔夊～锟?? IPS200_TYPE_SPI
#define BUZZER_PIN              (P19_4)    

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

char txt[128];


volatile uint8 is_jump_from_core1 = 0;              // 核1发送是否跳跃标志位
volatile uint8 is_jump_updated = 0;                 // 跳跃更新标志位
volatile uint8 bridge_valid_from_core1 = 0;         // 单边桥是否识别标志位
volatile int16 bridge_control_from_core1 = 0;       // 单边桥控制航向角数据
volatile uint8 bridge_control_updated = 0;          // 单边桥控制更新标志位
uint32 jump_count = 0;                              // 跳跃计数

// 接收核心1发送的数据
static void appipc_callback(uint32 data)
{
    if(vision_detect_mode == 2)
    {
        is_jump_from_core1 = (uint8)(data & 0x01);
        is_jump_updated = 1;
    }
    else if(vision_detect_mode == 1)
    {
        appipc_bridge_data_t bridge_data;

        if(appipc_decode_bridge_data(data, &bridge_data))
        {
            bridge_valid_from_core1 = bridge_data.valid;
            bridge_control_from_core1 = bridge_data.angle_d10;
        }
        else
        {
            bridge_valid_from_core1 = 0;
            bridge_control_from_core1 = 0;
        }

        bridge_control_updated = 1;
    }
}


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
  screen_init();                                        // 屏幕初始化
  flash_init();
  remote_control_init();                                // 遥控器初始化
  button_init();                                        // 按键初始化
  buzzer_init();                                        // 蜂鸣器初始化

  appipc_rx_init(appipc_callback);                      // IPC 初始化
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
    {"pause_flag",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"right_now",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"vy",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"segment_num ",  SCREEN_DATA_UINT, {.uint_value = 0}, 0},
    {"path_index",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"distance",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"num", SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"current_seg", SCREEN_DATA_INT, {.int_value = 0}, 0},
};
    //==========================
    // 主循环
    //==========================
    while(true)
    {
        button_update();                // 按键状态更新
        remote_update();                // 遥控器状态更新
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
        remote_table[3].value.int_value = pause_flag;
        remote_table[4].value.int_value = remote_right_01_now_flag;
        remote_table[5].value.float_value = vy;
        remote_table[6].value.uint_value = record_header.segment_num;
        remote_table[7].value.float_value = path_index;
        remote_table[8].value.float_value = distance_recover;
        remote_table[9].value.int_value = record_header.total_point_num;
        remote_table[10].value.int_value = current_segment;//vision_detect_mode
        screen_show_data_table(remote_table, 11);
        //==================================================
        // 01left: 0->1写入flash
        //==================================================
        if(remote_left_01_last_flag == 0 && remote_left_01_now_flag == 1)
//        if(button_flag[1]==1)
        {
          
//            sprintf(txt, "FLASH STORE...\r\n");
//            wireless_uart_send_string(txt);
            system_delay_ms(30); // 此处延时是为等待写入头信息（未试过去掉效果）
            // 写入Flash
            flash_path_store();          
            buzzer_beep(1,100);
            sprintf(txt, "FLASH STORE DONE\r\n");
            wireless_uart_send_string(txt);
        }

        //==================================================
        // 01left：0-> 2读取Flash
        //==================================================
//        if(remote_left_01_last_flag == 0 && remote_left_01_now_flag == 2)
        if(button_flag[2]==1)
        {
//            sprintf(txt, "FLASH LOAD...\r\n");
            wireless_uart_send_string(txt);
        
            
            // 读取Flash
            flash_path_load();
            flash_yaw_flag = 2;
            
            track_init();
            buzzer_beep(2,100);
            // 从头开始显示
            i = 0;
            
            sprintf(txt, "FLASH LOAD DONE\r\n");
            wireless_uart_send_string(txt);
        }

        //==========================
        // 更新按键历史状态
        //==========================
        remote_left_01_last_flag = remote_left_01_now_flag;

        system_delay_ms(20);
        
            // 元素通过检测（目前用延时做测试）
    if(!pause_flag)
    {
//        element_recover_check();
      pause_time ++;
      x = 0;
      y = 0;
      x_last = 0;
      y_last = 0;
      if (pause_time > 100)
      {
        pause_flag = true;
        pause_time = 0;
      }
        
    }
        
        
        if(!replay_point[i].x)
          i=0;
//        sprintf(txt, "tar|now:(%.3f,%.3f),(%.3f,%.3f)\r\n",target_x,target_y,x,y); 
        
        sprintf(txt, "x,y,yaw:%3f,%3f,%3f\r\n",replay_point[i].x,replay_point[i].y,replay_point[i].yaw); 
        
        i++;
        wireless_uart_send_string(txt);

    
// =========================================================== 视觉部分 ==========================================================

    appipc_send_speed_u32((uint32)fabsf(car_speed));  // 发送小车速度到核1
    
    if (pause_flag == false)
    {
        vision_detect_mode = 1;
    }
    else
    {
        vision_detect_mode = 0;
    }
    
    //=========================== 跳跃模式 ===========================
        if(vision_detect_mode == 2)
        {   
            // 速度控制
            switch (jump_count)
            {
                case 0:
                    vision_target_speed = 160;  // 第1次跳跃前的速度
                    break;

                case 1:
                    vision_target_speed = 130;  // 第2次跳跃前的速度
                    break;

                case 2:
                    vision_target_speed = 130;  // 第3次跳跃前的速度
                    break;
            }

            // 跳跃控制
            if(is_jump_updated)
            {
                is_jump_updated = 0;
                if(jump_count >= 2) continue;  // 限定跳跃次数
                
                // 执行跳跃
                if(is_jump_from_core1)
                {
                    jump_flag = 1;
                    jump_count++;
                }
            } 
        }


        //=========================== 单边桥模式 =========================
        else if(vision_detect_mode == 1)
        {
            if(bridge_control_updated)
            {
                bridge_control_updated = 0;

                if(bridge_valid_from_core1)
                {
                    vision_target_speed = 60;                             // 识别有效：固定低速前进
                    vision_target_yaw   = bridge_control_from_core1;      // 将核心1计算的视觉偏差转换成航向偏转加权量
                }
                else
                {
                    vision_target_speed = 0;  // 识别丢失或者进入单边桥
                    vision_target_yaw = 0;
                }
            }
        }


    }
} 
