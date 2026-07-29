#include "zf_common_headfile.h"
#define LED1                    (P19_0)     // SPI 涓插彛 SPI 涓わ拷?锟藉睆 杩欓噷瀹忓畾涔夊～锟?? IPS200_TYPE_SPI
#define BUZZER_PIN              (P19_4)    

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

#define BRIDGE_ALIGNED_CONFIRM_COUNT    (2)        // 连续对齐确认次数
#define BRIDGE_ALIGN_START_Y            (35)       // 进入低速精确对齐区域的画面纵坐标

#define BRIDGE_ALIGN_SPEED              (100)        // 距离较近且未对齐时的细调速度
#define BRIDGE_BLIND_RECOVERY_SPEED     (-80)        // 盲转超调后倒车寻找赛道的速度
#define BRIDGE_CROSS_SPEED              (-400)       // 对齐后冲过单边桥的速度
#define BUMP_CROSS_SPEED                (400)        // 通过颠簸路段时的固定速度


char txt[128];


volatile uint8 is_jump_from_core1 = 0;              // 核1发送是否跳跃标志位
volatile uint8 is_jump_updated = 0;                 // 跳跃更新标志位
volatile uint8 bridge_valid_from_core1 = 0;         // 单边桥是否识别标志位
volatile uint8 bridge_aligned_from_core1 = 0;       // 单边桥是否已经对齐标志位
volatile uint8 bridge_force_blind_from_core1 = 0;   // 核1是否请求使用 IMU 强制完成大角度盲转
volatile uint8 bridge_blind_release_from_core1 = 0; // 核1是否请求提前结束强制盲转
volatile uint8 bridge_fresh_target_from_core1 = 0;   // 核1是否连续识别到新鲜赛道目标
volatile uint8 bridge_forced_blind_active = 0;       // 核0当前是否正在执行强制盲转
volatile uint8 bridge_blind_recovery_active = 0;     // 核0当前是否正在倒车寻找赛道
volatile uint8 bridge_forced_blind_fault = 0;        // 强制盲转异常停止标志位
volatile uint8 bridge_bottom_y_from_core1 = 0;      // 单边桥最下端纵坐标，用于估算前向距离
volatile int16 bridge_control_from_core1 = 0;       // 单边桥控制航向角数据
volatile uint8 bridge_control_updated = 0;          // 单边桥控制更新标志位
volatile uint8 bridge_aligned_count = 0;            // 连续收到识别有效且已经对齐的次数
volatile uint8 phase_exited_from_core1 = 0;          // 核1确认已经离开当前阶段路段的标志位
volatile uint8 vision_phase_bab = VISION_PHASE_BAB_BRIDGE_ALIGN; // 单边桥与颠簸路段子状态，由核0统一控制
uint32 jump_count = 0;                              // 跳跃计数
volatile uint8 vision_steps = 0;                             // 视觉步骤 | 0 单边桥 | 1 跳跃 | 2 跳跃返回
volatile uint8 vision_phase_done_flag = 0;                   // 视觉单步骤完成标志位

// 接收核心1发送的数据
static void appipc_callback(uint32 data)
{
    if(vision_detect_mode == VISION_JUMP)
    {
        is_jump_from_core1 = (uint8)(data & 0x01);
        is_jump_updated = 1;
    }
    else if(vision_detect_mode == VISION_BRIDGE_BUMP)
    {
        appipc_bridge_data_t bridge_data;

        if(appipc_decode_bridge_data(data, &bridge_data))
        {
            bridge_valid_from_core1 = bridge_data.valid;
            bridge_aligned_from_core1 = bridge_data.aligned;
            bridge_force_blind_from_core1 = bridge_data.force_blind;
            bridge_blind_release_from_core1 = bridge_data.blind_release;
            bridge_fresh_target_from_core1 = bridge_data.fresh_target;
            phase_exited_from_core1 = bridge_data.exited;
            bridge_bottom_y_from_core1 = bridge_data.bottom_y;
            bridge_control_from_core1 = bridge_data.control_value;
        }
        else
        {
            bridge_valid_from_core1 = 0;
            bridge_aligned_from_core1 = 0;
            bridge_force_blind_from_core1 = 0;
            bridge_blind_release_from_core1 = 0;
            bridge_fresh_target_from_core1 = 0;
            phase_exited_from_core1 = 0;
            bridge_bottom_y_from_core1 = 0;
            bridge_control_from_core1 = 0;
        }

        // 对齐阶段连续确认成功后，进入冲桥和离桥检测阶段
        if(vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN)
        {
            if(bridge_force_blind_from_core1 ||
               bridge_blind_release_from_core1 ||
               bridge_forced_blind_active ||
               bridge_blind_recovery_active ||
               bridge_forced_blind_fault)
            {
                bridge_aligned_count = 0;
            }
            else if(bridge_valid_from_core1 && bridge_aligned_from_core1)
            {
                if(bridge_aligned_count < BRIDGE_ALIGNED_CONFIRM_COUNT)
                {
                    bridge_aligned_count++;
                }

                if(bridge_aligned_count >= BRIDGE_ALIGNED_CONFIRM_COUNT)
                {
                    vision_phase_bab = VISION_PHASE_BAB_BRIDGE_EXIT_CHECK;
                }
            }
            else
            {
                bridge_aligned_count = 0;
            }
        }
        else if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_EXIT_CHECK) && phase_exited_from_core1)
        {
            phase_exited_from_core1 = 0;
            vision_phase_bab = VISION_PHASE_BAB_BUMP_EXIT_CHECK;
        }
        else if((vision_phase_bab == VISION_PHASE_BAB_BUMP_EXIT_CHECK) && phase_exited_from_core1)
        {
            phase_exited_from_core1 = 0;
            vision_phase_bab = VISION_PHASE_BAB_COMPLETE;
            vision_phase_done_flag = 1;
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
  // screen_init();                                     // TFT180 由核心1负责，核心0暂不初始化屏幕
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
    int pause_time = 0; // 便于调试惯导打断恢复逻辑
    uint8 remote_ch9_value = 100;  // 通道9映射值，遥控器掉线时保持上一次有效值

    screen_data_item_t remote_table[] =
{
    {"angle",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"vx",  SCREEN_DATA_FLOAT, {.float_value = 0}, 0},
    {"road_memery_flag",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"pause_flag",  SCREEN_DATA_INT, {.int_value = 0}, 0},
    {"course_load_flag",  SCREEN_DATA_INT, {.int_value = 0}, 0},
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
        if(remote_is_online())
        {
            remote_ch9_value = remote_left_knob_ctrl();
        }
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
        remote_table[0].value.float_value = path_yaw_change;
        remote_table[1].value.float_value = vx;
        remote_table[2].value.int_value = road_memery_flag;
        remote_table[3].value.int_value = pause_flag;
        remote_table[4].value.int_value = course_load_flag;
        remote_table[5].value.float_value = vy;
        remote_table[6].value.uint_value = mul_header.segment_num;
        remote_table[7].value.float_value = path_index;
        remote_table[8].value.float_value = distance_recover;
        remote_table[9].value.int_value = replay_point_num;
        remote_table[10].value.int_value = current_segment;//vision_detect_mode
        // screen_show_data_table(remote_table, 11);     // TFT180 暂不显示通用数据表
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
        
        if(button_flag[0]||button_flag[1]||button_flag[2])
        {
            sprintf(txt, "FLASH LOAD...\r\n");
            wireless_uart_send_string(txt);      
             // 读取Flash
            flash_path_load();
            flash_yaw_flag = 2;
            
//            track_init();
        }
        //==================================================
        // 01left：0-> 2读取Flash
        //==================================================
//        if(remote_left_01_last_flag == 0 && remote_left_01_now_flag == 2)
//        if(button_flag[2]==1)
//        {
////            sprintf(txt, "FLASH LOAD...\r\n");
//            wireless_uart_send_string(txt);
//        
//            
            // 读取Flash
//            flash_path_load();
//            flash_yaw_flag = 2;
//            
//            track_init();
//            buzzer_beep(2,100);
//            // 从头开始显示
//            i = 0;
//            
//            sprintf(txt, "FLASH LOAD DONE\r\n");
//            wireless_uart_send_string(txt);
//        }

        //==========================
        // 更新按键历史状态
        //==========================
        remote_left_01_last_flag = remote_left_01_now_flag;

        system_delay_ms(20);
        
        // 视觉阶段 - 将控制从视觉交接给惯导
        if(!pause_flag)
        {
            // element_recover_check();
            x = 0;
            y = 0;
            x_last = 0;
            y_last = 0;
            pause_time ++;
            
            if (vision_phase_done_flag)
            {
                vision_phase_done_flag = 0;  // 跳跃完成标志位置 0
                vision_steps++;  // 进入下一视觉阶段，等待 pause_flag 再次为 false

                vision_target_speed = 0;
                vision_target_yaw = 0;

                //Y_left = 0.0f;
                //Y_right = 0.0f;
                
                pause_flag = true;
                pause_time = 0;
                
            }
            
        }
        
        
        if(!replay_point[i].x)
          i=0;
//        sprintf(txt, "tar|now:(%.3f,%.3f),(%.3f,%.3f)\r\n",target_x,target_y,x,y); 
        
//        sprintf(txt, "x,y,yaw:%3f,%3f,%3f\r\n",replay_point[i].x,replay_point[i].y,replay_point[i].yaw); 
        sprintf(txt, "angle:%3f\r\n",get_path_turn_angle(path_index)); 
        
        i++;
        wireless_uart_send_string(txt);

    
// =========================================================== 视觉部分 ==========================================================

     if (pause_flag == false)
    {
        // 视觉步骤切换
        switch (vision_steps)
        {
        case 0:
            vision_detect_mode = VISION_BRIDGE_BUMP;  // 单边桥和颠簸路段
            break;

        case 1:
            vision_detect_mode = VISION_JUMP;  // 跳跃
            break;
        
        case 2:
            vision_detect_mode = VISION_BACK;  // 三级台阶返回
            break;

        }
    }
    else
    {
        vision_detect_mode = VISION_IDLE;  // 空闲
    }

    appipc_send_core0_data((uint16)fabsf(car_speed), (uint8)vision_detect_mode, vision_phase_bab, remote_ch9_value);  // 发送车速、视觉状态和通道9数据到核1
    
    //=========================== 跳跃模式 ===========================
        if(vision_detect_mode == VISION_JUMP)
        {   
            // 速度控制
            switch (jump_count)
            {
                case 0:
                    vision_target_speed = 220;  // 第1次跳跃前的速度
                    break;

                case 1:
                    vision_target_speed = 150;  // 第2次跳跃前的速度
                    break;

                case 2:
                    vision_target_speed = 150;  // 第3次跳跃前的速度
                    break;
            }

            // 跳跃控制
            if(is_jump_updated)
            {
                is_jump_updated = 0;
                if(jump_count >= 3) continue;  // 限定跳跃次数
                
                // 执行跳跃
                if(is_jump_from_core1)
                {
                    jump_flag = 1;
                    jump_count++;

                    if (jump_count == 3)
                    {
                        vision_phase_done_flag = 1;
                    }
                    
                }
            } 
        }


        //=========================== 单边桥与颠簸路段模式 =========================
        else if(vision_detect_mode == VISION_BRIDGE_BUMP)
        {
            // 设置腿重心为 70度
            Y_left = 30.0f;
            Y_right = 30.0f;
            
            // 如果完全通过单边桥和颠簸路段
            if(vision_phase_bab == VISION_PHASE_BAB_COMPLETE)
            {

                // sprintf(txt, "单边桥和颠簸路段全部完成\n");
                // wireless_uart_send_string(txt);

                bridge_control_updated = 0;
                vision_target_speed = 0;             // 完成后立即清除视觉速度，等待控制交接
                vision_target_yaw = 0;               // 清除 yaw 锁定
                vision_detect_mode = VISION_IDLE;
            }
            //如果状态为 离开单边桥检查，即冲刺过单边桥阶段
            else if(vision_phase_bab == VISION_PHASE_BAB_BRIDGE_EXIT_CHECK)
            {
                // sprintf(txt, "冲刺单边桥\n");
                // wireless_uart_send_string(txt);

                bridge_control_updated = 0;
                vision_target_speed = BRIDGE_CROSS_SPEED;  // 保持对齐后的航向并冲过单边桥
                vision_target_yaw = 0;
            }
            // 如果状态为 离开颠簸路段检查阶段， 即冲刺过颠簸路段
            else if(vision_phase_bab == VISION_PHASE_BAB_BUMP_EXIT_CHECK)
            {
                // sprintf(txt, "冲刺颠簸路段\n");
                // wireless_uart_send_string(txt);

                bridge_control_updated = 0;
                vision_target_speed = BUMP_CROSS_SPEED;    // 保持单边桥出口航向通过颠簸路段
                vision_target_yaw = 0;
            }
            // 强制盲转异常时停车，等待离开当前视觉阶段后复位
            else if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN) &&
                    bridge_forced_blind_fault)
            {
                bridge_control_updated = 0;
                vision_target_speed = 0;
                vision_target_yaw = 0;
            }
            // 盲转达到角度上限后仍未找到赛道，倒车并使用 IMU 进行小角度反向修正
            else if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN) &&
                    bridge_blind_recovery_active)
            {
                bridge_control_updated = 0;
                vision_target_speed = BRIDGE_BLIND_RECOVERY_SPEED;
                vision_target_yaw = 0;
            }
            // 强制盲转期间忽略中途视觉结果，航向角由核心0的 IMU 闭环控制
            else if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN) &&
                    (bridge_force_blind_from_core1 || bridge_forced_blind_active))
            {
                bridge_control_updated = 0;
                vision_target_speed = BRIDGE_ALIGN_SPEED;
                vision_target_yaw = 0;
            }
            // 如果状态为 单边桥对齐阶段 并且 数据已经更新
            else if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN) && bridge_control_updated)
            {
                bridge_control_updated = 0;

                if(bridge_valid_from_core1)
                {
                    // 尚未完成整条边线对准时，保持低速前进并执行视觉转向
                    if(!bridge_aligned_from_core1)
                    {
                        // 太靠近，往回退
                        if (100 < bridge_bottom_y_from_core1)
                        {
                            vision_target_speed = -120;  // 回退速度
                            bridge_control_from_core1 = -bridge_control_from_core1;  // 角度反向
                        }
                        else
                        {
                            vision_target_speed = BRIDGE_ALIGN_SPEED;  // 前进对准速度
                        }
                        
                    }
                    else
                    {
                        vision_target_speed = 0;  // 等待连续对准确认后进入冲桥阶段
                    }

                    vision_target_yaw = bridge_control_from_core1;
                }
                else
                {
                    vision_target_speed = BRIDGE_ALIGN_SPEED;  // 识别丢失：慢慢向前寻找目标
                    vision_target_yaw = 0;
                }
            }
        }


    }
} 
