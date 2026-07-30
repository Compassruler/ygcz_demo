#include "zf_common_headfile.h"
#include "imu.h"

#define LED1                    (P19_0)                                         // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SP
#define VISION_ALIGN_TARGET_YAW_LIMIT_DEG   (55.0f)                             // 视觉对准允许设置的最大相对航向角
#define VISION_ALIGN_ABSOLUTE_YAW_LIMIT_DEG (90.0f)                             // 视觉对准允许转动的最大实际航向角
#define VISION_FORCE_BLIND_TIMEOUT_MS       (4000u)                             // 强制盲转最长执行时间
#define VISION_FORCE_BLIND_DIRECTION        (-1.0f)                             // 强制盲转方向修正，方向相反时在 1.0f 和 -1.0f 之间切换
#define VISION_FORCE_BLIND_YAW_STEP_DEG     (0.09f)                             // 每 5ms 增加的目标航向角，数值越小转向越慢
#define VISION_BLIND_RECOVERY_YAW_DEG       (15.0f)                             // 以当前航向为起点，向盲转反方向最多修正 15 度
#define VISION_BLIND_RECOVERY_YAW_STEP_DEG  (0.025f)                            // 每 5ms 增加的恢复目标角，降低恢复阶段转向速度
#define VISION_BLIND_RECOVERY_MIN_TIME_MS   (100u)                              // 倒车后允许重新前进的最短时间
#define VISION_BLIND_RECOVERY_MAX_TIME_MS   (4000u)                             // 允许缓慢完成 15 度修正，并预留实际航向跟随时间
#define VISION_BLIND_RECOVERY_COOLDOWN_MS   (300u)                              // 找回赛道后禁止再次强制盲转的时间
extern volatile uint8 vision_phase_bab;                                         // 核心0当前的单边桥与颠簸路段子状态
extern volatile uint8 bridge_force_blind_from_core1;                            // 核心1发送的强制盲转请求
extern volatile uint8 bridge_blind_release_from_core1;                          // 核心1发送的提前结束盲转请求
extern volatile uint8 bridge_fresh_target_from_core1;                           // 核心1发送的新鲜赛道目标标志位
extern volatile uint8 bridge_forced_blind_active;                               // 核心0当前是否正在执行强制盲转
extern volatile uint8 bridge_blind_recovery_active;                             // 核心0当前是否正在倒车寻找赛道
extern volatile uint8 bridge_forced_blind_fault;                                // 强制盲转异常停止标志位
extern volatile uint8 bridge_valid_from_core1;
extern volatile uint8 bridge_aligned_from_core1;
extern volatile uint8 bridge_control_updated;
extern volatile uint8 bridge_aligned_count;
extern volatile int16 bridge_control_from_core1;

static uint8_t yaw_lock_init = 0; // 后续放在flag里面

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    static uint32 system_time = 0;
    static uint8 vision_yaw_guard_active = 0;   // 盲转保护激活
    static uint8 vision_yaw_guard_reached = 0;  // 盲转保护触发
    static float vision_yaw_guard_start = 0.0f; // 盲转保护激活时航向角
    static uint8 force_blind_request_latched = 0; // 当前强制盲转请求是否已经锁存
    static uint32 force_blind_start_time = 0;     // 强制盲转开始时间
    static float force_blind_start_yaw = 0.0f;    // 强制盲转开始时的实际航向角
    static float force_blind_target_yaw = 0.0f;   // 强制盲转的目标航向角
    static float force_blind_command_yaw = 0.0f;  // 逐步发送给航向环的目标航向角
    static float force_blind_direction = 1.0f;    // 强制盲转方向
    static uint32 blind_recovery_start_time = 0;  // 倒车恢复开始时间
    static uint32 force_blind_cooldown_until = 0; // 下一次允许强制盲转的时间
    static float blind_recovery_target_yaw = 0.0f;  // 倒车恢复的最终目标航向角
    static float blind_recovery_command_yaw = 0.0f; // 倒车恢复逐步使用的目标航向角
    static float blind_recovery_direction = 1.0f;   // 倒车恢复的航向修正方向
    float force_blind_progress = 0.0f;
    system_time ++;
    remote_update();
    imu_data_get();               // 原始数据
    imu_data_transition();        // 转换后数据

    // 边线识别对准阶段大角度盲转保护
    // 只限制单边桥对准过程，离桥和颠簸路段继续沿用已经锁定的航向
    if((vision_detect_mode == VISION_BRIDGE_BUMP) && (vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN))
    {
      if(!vision_yaw_guard_active)
      {
        vision_yaw_guard_active = 1;
        vision_yaw_guard_reached = 0;
        vision_yaw_guard_start = yaw_angle;
        target_yaw_remote = yaw_angle;
      }
      
      // 普通视觉对准不超过允许的最大实际航向角，强制盲转使用自己的起点单独计角
      if(!bridge_forced_blind_active &&
         !bridge_blind_recovery_active &&
         (fabsf(yaw_angle - vision_yaw_guard_start) >= VISION_ALIGN_ABSOLUTE_YAW_LIMIT_DEG))
      {
        vision_yaw_guard_reached = 1;  // 触发保护
      }

      // 触发保护后
      if(vision_yaw_guard_reached)
      {
        vision_target_speed = 0;
        vision_target_yaw = 0;
      }
    }
    else
    {
      vision_yaw_guard_active = 0;
      vision_yaw_guard_reached = 0;
      bridge_forced_blind_active = 0;
      bridge_blind_recovery_active = 0;
      bridge_forced_blind_fault = 0;
      bridge_force_blind_from_core1 = 0;
      bridge_blind_release_from_core1 = 0;
      bridge_fresh_target_from_core1 = 0;
      force_blind_request_latched = 0;
    }

    if(!bridge_force_blind_from_core1 && !bridge_forced_blind_active)
    {
      force_blind_request_latched = 0;
    }

     // 速度环 
    if(system_time % 20 == 0)
    {
      small_driver_get_speed(&small_driver_value);
      car_speed = ((-small_driver_value.receive_left_speed_data) + small_driver_value.receive_right_speed_data) / 2;
      true_speed = rpmtotrue(car_speed); 
      if (track_flag && pause_flag)
      {
        Track_update();
        pid_pos_calc(&banlance.speed_pid, target_speed, car_speed);
        
      }
      else if (vision_detect_mode != VISION_IDLE)  // 视觉控制模式
      {
        pid_pos_calc(&banlance.speed_pid, vision_target_speed, car_speed);
      }
      else if(road_memery_flag == 2)pid_pos_calc(&banlance.speed_pid, 0 , car_speed); // 回放结束速度给0
      else // 遥控器 
      {
        pid_pos_calc(&banlance.speed_pid, remote_front_rear_ctrl() , car_speed);
      }

      
      if ((road_memery_flag == 1 && pause_flag) || remote_right_01_now_flag == 0)
      {
        ins_update();  // ins数据更新       
      }      
    }

    // 角度环
    if(system_time % 5 == 0)
    {
      pitch_acc2angle =  imu_acc2angle(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);            // 角速度转角度 俯仰角
      roll_acc2angle  =  imu_acc2angle(imu_data.acc_y, imu_data.acc_x, imu_data.acc_z);            // 角速度转角度 横滚角
      yaw_angle += imu_data.gyro_z * 0.005f;                                                          // 直接对角速度做积分，yaw角的加速度不能得到yaw角(角度这是)
//      while(yaw_angle > 180.0f) yaw_angle -= 360.0f;
//      while(yaw_angle < -180.0f) yaw_angle += 360.0f;
      first_order_complementary_filtering(&pitch_filter, imu_data.gyro_y, pitch_acc2angle);          // 一阶互补滤波处理，这里输出pitch_filter.filtering_angle
      first_order_complementary_filtering(&roll_filter, imu_data.gyro_x, roll_acc2angle);            // 输出roll_filter.filtering_angle
      pid_pos_calc(&banlance.pitch_angle_pid, 0, pitch_filter.filtering_angle);
      pid_inc_calc(&banlance.roll_angle_pid, 0, roll_filter.filtering_angle);

      if (track_flag  && pause_flag) pid_pos_calc(&banlance.yaw_angle_pid, target_yaw, yaw_angle);
      
      else if (vision_detect_mode == VISION_BRIDGE_BUMP) // 需要视觉转向控制的模式
      {
        // 强制盲转请求只在上升沿锁存一次，中途忽略视觉识别结果
        if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN) &&
           bridge_force_blind_from_core1 &&
           !force_blind_request_latched &&
           !bridge_forced_blind_active &&
           !bridge_blind_recovery_active &&
           !bridge_forced_blind_fault &&
           (system_time >= force_blind_cooldown_until) &&
           (0 != bridge_control_from_core1))
        {
          force_blind_request_latched = 1;
          bridge_forced_blind_active = 1;
          bridge_forced_blind_fault = 0;
          force_blind_start_time = system_time;
          force_blind_start_yaw = yaw_angle;
          force_blind_direction =
              ((bridge_control_from_core1 > 0) ? 1.0f : -1.0f) *
              VISION_FORCE_BLIND_DIRECTION;
          force_blind_target_yaw =
              force_blind_start_yaw +
              force_blind_direction * VISION_ALIGN_TARGET_YAW_LIMIT_DEG;
          force_blind_command_yaw = force_blind_start_yaw;
          vision_yaw_guard_start = force_blind_start_yaw;
          vision_yaw_guard_reached = 0;
          bridge_aligned_count = 0;
          bridge_fresh_target_from_core1 = 0;
          target_yaw_remote = force_blind_command_yaw;
        }

        if(bridge_blind_recovery_active)
        {
          // 找回连续可靠的新鲜赛道目标后，锁定当前航向并恢复普通视觉控制
          if(((system_time - blind_recovery_start_time) >=
              VISION_BLIND_RECOVERY_MIN_TIME_MS) &&
             bridge_fresh_target_from_core1)
          {
            bridge_blind_recovery_active = 0;
            force_blind_cooldown_until =
                system_time + VISION_BLIND_RECOVERY_COOLDOWN_MS;
            vision_yaw_guard_start = yaw_angle;
            vision_yaw_guard_reached = 0;
            target_yaw_remote = yaw_angle;
            vision_target_yaw = 0;
            if(vision_target_speed < 0)
            {
              vision_target_speed = -vision_target_speed;
            }
            bridge_control_updated = 1;
            bridge_aligned_count = 0;
          }
          // 倒车超过最长时间仍找不到赛道，停车并保持故障状态
          else if((system_time - blind_recovery_start_time) >=
                  VISION_BLIND_RECOVERY_MAX_TIME_MS)
          {
            bridge_blind_recovery_active = 0;
            bridge_forced_blind_fault = 1;
            vision_yaw_guard_reached = 1;
            target_yaw_remote = yaw_angle;
            vision_target_speed = 0;
            vision_target_yaw = 0;
            bridge_control_updated = 0;
            bridge_aligned_count = 0;
          }
          else
          {
            blind_recovery_command_yaw +=
                blind_recovery_direction * VISION_BLIND_RECOVERY_YAW_STEP_DEG;

            if(((blind_recovery_direction > 0.0f) &&
                (blind_recovery_command_yaw > blind_recovery_target_yaw)) ||
               ((blind_recovery_direction < 0.0f) &&
                (blind_recovery_command_yaw < blind_recovery_target_yaw)))
            {
              blind_recovery_command_yaw = blind_recovery_target_yaw;
            }

            target_yaw_remote = blind_recovery_command_yaw;
          }
        }
        else if(bridge_forced_blind_active)
        {
          force_blind_progress =
              (yaw_angle - force_blind_start_yaw) * force_blind_direction;

          // 可靠视觉重新出现且夹角足够小时，锁定当前航向并提前退出盲转
          if(bridge_blind_release_from_core1)
          {
            bridge_forced_blind_active = 0;
            bridge_forced_blind_fault = 0;
            bridge_blind_release_from_core1 = 0;
            force_blind_cooldown_until =
                system_time + VISION_BLIND_RECOVERY_COOLDOWN_MS;
            vision_yaw_guard_start = yaw_angle;
            vision_yaw_guard_reached = 0;
            target_yaw_remote = yaw_angle;
            vision_target_yaw = 0;
            bridge_valid_from_core1 = 0;
            bridge_aligned_from_core1 = 0;
            bridge_control_updated = 0;
            bridge_aligned_count = 0;
          }
          // 达到盲转角度上限后，没有找到赛道则进入倒车恢复
          else if(force_blind_progress >= VISION_ALIGN_TARGET_YAW_LIMIT_DEG)
          {
            bridge_forced_blind_active = 0;
            bridge_forced_blind_fault = 0;
            vision_yaw_guard_start = yaw_angle;
            vision_yaw_guard_reached = 0;
            target_yaw_remote = yaw_angle;
            vision_target_yaw = 0;
            bridge_valid_from_core1 = 0;
            bridge_aligned_from_core1 = 0;
            bridge_control_updated = 0;
            bridge_aligned_count = 0;

            if(!bridge_fresh_target_from_core1)
            {
              bridge_blind_recovery_active = 1;
              blind_recovery_start_time = system_time;
              // 恢复方向始终与刚才的盲转方向相反
              blind_recovery_direction = -force_blind_direction;
              blind_recovery_target_yaw =
                  yaw_angle +
                  blind_recovery_direction * VISION_BLIND_RECOVERY_YAW_DEG;
              blind_recovery_command_yaw = yaw_angle;
            }
            else
            {
              force_blind_cooldown_until =
                  system_time + VISION_BLIND_RECOVERY_COOLDOWN_MS;
            }
          }
          // 转向方向异常、超过 90 度或者执行超时，立即停止并保持保护状态
          else if((fabsf(yaw_angle - force_blind_start_yaw) >=
                   VISION_ALIGN_ABSOLUTE_YAW_LIMIT_DEG) ||
                  ((system_time - force_blind_start_time) >=
                   VISION_FORCE_BLIND_TIMEOUT_MS))
          {
            bridge_forced_blind_active = 0;
            bridge_forced_blind_fault = 1;
            vision_yaw_guard_reached = 1;
            target_yaw_remote = yaw_angle;
            vision_target_speed = 0;
            vision_target_yaw = 0;
            bridge_control_updated = 0;
            bridge_aligned_count = 0;
          }
          else
          {
            // 目标航向角逐步变化，避免一次跳到 85 度导致航向环满输出
            force_blind_command_yaw +=
                force_blind_direction * VISION_FORCE_BLIND_YAW_STEP_DEG;

            if(((force_blind_direction > 0.0f) &&
                (force_blind_command_yaw > force_blind_target_yaw)) ||
               ((force_blind_direction < 0.0f) &&
                (force_blind_command_yaw < force_blind_target_yaw)))
            {
              force_blind_command_yaw = force_blind_target_yaw;
            }

            target_yaw_remote = force_blind_command_yaw;
          }
        }
        // 保护模式激活
        else if(vision_yaw_guard_active)
        {
          // 保护模式达到
          if(vision_yaw_guard_reached)
          {
            target_yaw_remote = vision_yaw_guard_start + ((yaw_angle >= vision_yaw_guard_start) ?
                 VISION_ALIGN_TARGET_YAW_LIMIT_DEG : -VISION_ALIGN_TARGET_YAW_LIMIT_DEG);
          }
          else
          {
            target_yaw_remote += vision_target_yaw * 0.003f;  // 先正常处理
 
            // 限制最大角度 VISION_ALIGN_TARGET_YAW_LIMIT_DEG
            if(target_yaw_remote > vision_yaw_guard_start + VISION_ALIGN_TARGET_YAW_LIMIT_DEG)
            {
              target_yaw_remote = vision_yaw_guard_start + VISION_ALIGN_TARGET_YAW_LIMIT_DEG;
            }
            else if(target_yaw_remote < vision_yaw_guard_start - VISION_ALIGN_TARGET_YAW_LIMIT_DEG)
            {
              target_yaw_remote = vision_yaw_guard_start - VISION_ALIGN_TARGET_YAW_LIMIT_DEG;
            }
          }
        }
        else
        {
          if (vision_phase_bab == VISION_PHASE_BAB_BRIDGE_EXIT_CHECK)
          {
            target_yaw_remote = 0;  // 对齐之后航向角和发车相同
          }
          else
          {
            target_yaw_remote += vision_target_yaw * 0.003f;  // 正常处理方式
          }
        }


        pid_pos_calc(&banlance.yaw_angle_pid, target_yaw_remote, yaw_angle);
      }
      else if (vision_detect_mode == VISION_JUMP) // 跳跃
      {
        // 跳跃暂时不用改变航向角
        pid_pos_calc(&banlance.yaw_angle_pid, 0, yaw_angle);
      }
      else 
      {
        if(remote_lock_yaw_ctrl() <= -500)  yaw_lock_ctrl = 1; // 遥控器在线锁航向角
//        if(remote_lock_yaw_ctrl() >= 500)   yaw_lock_ctrl = 0;   // 遥控器在线解航向角
        
        if(yaw_lock_ctrl ==1)
        {
          if(yaw_lock_init == 0)
            {
                yaw_angle = 0;        //只执行一次
                target_yaw_remote = 0;
                yaw_lock_init = 1;
             }
          
           target_yaw_remote += remote_left_right_ctrl() * 0.002f;
           pid_pos_calc(&banlance.yaw_angle_pid, target_yaw_remote, yaw_angle);
         }

      }
        //      pid_pos_calc(&banlance.yaw_angle_pid, 0, yaw_angle);
      leg_control(); // 5ms调用一次      
    }


    // 角速度环
    pid_pos_calc(&banlance.pitch_gyro_pid,banlance.pitch_angle_pid.output, imu_data.gyro_y); // 俯仰角
    
    pid_pos_calc(&banlance.yaw_gyro_pid,  banlance.yaw_angle_pid.output, imu_data.gyro_z);

    int balance_out = (int)banlance.pitch_gyro_pid.output;
    int yaw_gyro_out = (int)banlance.yaw_gyro_pid.output;

    if(fabs(pitch_filter.filtering_angle) > 70.0f || fabs(true_speed) >=8.0f) // 自动保护
//      if(fabs(pitch_filter.filtering_angle) > 75.0f) // 自动保护
      {
        auto_protect_flag = 1;
      }
    
    if(auto_protect_flag || manual_protect_flag == 1)
      {
        small_driver_set_duty(&small_driver_value,0, 0); 
      }
    else
    {
      jump_control();
      small_driver_set_duty(&small_driver_value,-(balance_out + yaw_gyro_out),(balance_out - yaw_gyro_out)); 
    }
}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH1);
   
}

void pit0_ch2_isr()                     // 定时器通道 2 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH2);
    
}

void pit0_ch10_isr()                    // 定时器通道 10 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // 定时器通道 11 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // 定时器通道 12 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // 定时器通道 13 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // 定时器通道 14 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // 定时器通道 15 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // 定时器通道 16 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // 定时器通道 17 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // 定时器通道 18 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // 定时器通道 19 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // 定时器通道 20 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // 定时器通道 21 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 串口中断
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
      
    }
    else                                 // 串口0发送中断
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收中断
    {
        
      wireless_module_uart_handler();  // 无线模块统一回调函数
        
      
    }
    else                                // 串口1发送中断
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {
              
        uart_receiver_handler() ; // 遥控器
//        gnss_uart_callback();            // GPS模块回调函数      
        
    }
    else                                // 串口2发送中断
    {
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收中断
    {
        
        
        
    }
    else                                // 串口3发送中断
    {
      
        
        
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4接收中断
    {
//      gpio_toggle_level(LED1);
      small_driver_control_callback(&small_driver_value);
//        uart_receiver_handler();                                                                // 串口接收机回调函数
       
    }
    else                                // 串口4发送中断
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5接收中断
    {
     
      
      
    }
    else                                // 串口5发送中断
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6接收中断
    {

        
       
    }
    else                                // 串口6发送中断
    {
      
        
        
    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断服务函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断服务函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0端口外部中断判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断服务函数     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断服务函数     
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断服务函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断服务函数     
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断服务函数     
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断服务函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断服务函数     
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断服务函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断服务函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断服务函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断服务函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断服务函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断服务函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断服务函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断服务函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断服务函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断服务函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断服务函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断服务函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断服务函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断服务函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断服务函数     
{



}
// **************************** 外部中断函数 ****************************
