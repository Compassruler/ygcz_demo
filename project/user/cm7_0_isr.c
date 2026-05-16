#include "zf_common_headfile.h"
#include "imu.h"

#define LED1                    (P19_0)                                         // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SP
// **************************** PIT中断函数 ****************************
void pit0_ch0_isr()
{
    pit_isr_flag_clear(PIT_CH0);
    static uint32 system_time = 0;
    int16 car_speed;
    system_time ++;
    imu_data_get();               // 原始数据
    imu_data_transition();        // 转换后数据

     // 速度环 
    if(system_time % 20 == 0)
    {
      small_driver_get_speed(&small_driver_value);
      // (-small_driver_value.receive_left_speed_data) (small_driver_value.receive_right_speed_data) 向前数值为正 向后数值为负
      car_speed = ((-small_driver_value.receive_left_speed_data) + small_driver_value.receive_right_speed_data) / 2;
      pid_pos_calc(&speed_pid, 0 , (float)car_speed);
    }

    // 角度环
    if(system_time % 5 == 0)
    {
      pitch_acc2angle =  imu_acc2angle(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);            // 角速度转角度 俯仰角
      roll_acc2angle  =  imu_acc2angle(imu_data.acc_y, imu_data.acc_x, imu_data.acc_z);            // 角速度转角度 横滚角
      yaw_angle += imu_data.gyro_z * 0.005f;                                                       // 直接对角速度做积分，yaw角的加速度不能得到yaw角
      
      first_order_complementary_filtering(&pitch_filter, imu_data.gyro_y, pitch_acc2angle);          // 一阶互补滤波处理，这里输出pitch_filter.filtering_angle
      first_order_complementary_filtering(&roll_filter, imu_data.gyro_x, roll_acc2angle);            // 输出roll_filter.filtering_angle
      
      pid_pos_calc(&pitch_angle_pid, 0, pitch_filter.filtering_angle);
      pid_inc_calc(&roll_angle_pid, 0, roll_filter.filtering_angle);
      pid_pos_calc(&yaw_angle_pid, 0, yaw_angle);
      
      leg_control(); // 5ms调用一次
    }
    
    // 角速度环
    pid_pos_calc(&gyro_pid,pitch_angle_pid.output, imu_data.gyro_y);
    int balance_out = (int)gyro_pid.output;
    int yaw_out     = (int)yaw_angle_pid.output;

    small_driver_set_duty(&small_driver_value,-(balance_out + yaw_out), (balance_out - yaw_out)); // 未测试
//    small_driver_set_duty(&small_driver_value, -(int)gyro_pid.output, (int)gyro_pid.output );
    
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
//        uart_receiver_handler() ; // 遥控器
      
    }
    else                                // 串口1发送中断
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收中断
    {
        
        gnss_uart_callback();            // GPS模块回调函数      
        
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