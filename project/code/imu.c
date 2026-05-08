#include "zf_common_headfile.h"

// 2026.5.8 明天把数值转换为角度，接着做一节互补滤波
void imu_data_get()
{
  imu660rb_get_acc();                                                     // 获取 imu660rb 的加速度测量数值
  imu660rb_get_gyro();  
  imu660rb_acc_x -= 166;
  imu660rb_acc_y -= 426;
  imu660rb_acc_z += 4028;
  imu660rb_gyro_x-= 5;
  imu660rb_gyro_y+= 6;
  imu660rb_gyro_z+= 5;
  
  if(func_abs(imu660rb_acc_x) <= 5)
  {
    imu660rb_acc_x = 0;
  }
  if(func_abs(imu660rb_acc_y) <= 5)
  {
    imu660rb_acc_y = 0;
  }
  if(func_abs(imu660rb_acc_z) <= 5)
  {
    imu660rb_acc_z = 0;
  }
  if(func_abs(imu660rb_gyro_x) <= 5)
  {
    imu660rb_gyro_x = 0;
  }
  if(func_abs(imu660rb_gyro_y) <= 5)
  {
    imu660rb_gyro_y = 0;
  }
  if(func_abs(imu660rb_gyro_z) <= 5)
  {
    imu660rb_gyro_z = 0;
  }
}

//void first_order_complementary_filtering (cascade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data)
//{
//    float gyro_temp;          // 角速度计算临时变量
//    
//    float acc_temp;           // 加速度计算临时变量
//
//    gyro_temp = gyro_raw_data * filter_value->gyro_ration;                              // 角速度数据 * 角速度置信度(一般给4)
//
//    acc_temp = (acc_raw_data - filter_value->angle_temp) * filter_value->acc_ration;    // 加速度微分数据 * 加速度置信度(一般给4)
//
//    filter_value->angle_temp += ((gyro_temp + acc_temp) * filter_value->call_cycle);    // 两数之和 * 调用周期 并积分到角度输出
//
//    filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero;  // 最终滤波角度减去零点位置即可
//}


// 调试代码
//#define LED1                    (P19_0)                                         // SPI 串口 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI
//char txt[128];
//
//int main(void)
//{
//    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
//    debug_init();                          // 调试串口信息初始化
//    servo_init();
//    
//    // 此处编写用户代码 例如外设初始化代码等
//    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
//    
//    if(wireless_uart_init())                                                    // 判断是否通过初始化
//    {
//        while(1)                                                                // 初始化失败就在这进入死循环
//        {
//            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
//            system_delay_ms(100);                                               // 短延时快速闪灯表示异常
//        }
//    }
//    
//    wireless_uart_send_byte('\r');
//    wireless_uart_send_byte('\n');
//    wireless_uart_send_string(" WirelessUart is ok.\r\n");              // 初始化正常 输出测试信息
//
//    while(1)
//    {
//        if(imu660rb_init())
//        {
//           wireless_uart_send_string("imu660rb init error.\r\n");                                 // imu660rb 初始化失败
//        }
//        else
//        {
//           break;
//        }
//        gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
//    }
//
//    // 此处编写用户代码 例如外设初始化代码等
//    while(true)
//    {
//        // 此处编写需要循环执行的代码
//        imu_data_get();
//        sprintf(txt,
//                "ACCandGYRO:%d,%d,%d,%d,%d,%d\n",
//                imu660rb_acc_x,
//                imu660rb_acc_y,
//                imu660rb_acc_z,
//                imu660rb_gyro_x,
//                imu660rb_gyro_y,
//                imu660rb_gyro_z);
//
//        wireless_uart_send_string(txt);
//
//        gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭
//        system_delay_ms(20);
//        // 此处编写需要循环执行的代码
//    }
//}