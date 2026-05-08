#include "zf_common_headfile.h"

// ================= 舵机PWM引脚 =================
// 上0°下180°
#define SERVO1_PWM     (TCPWM_CH09_P05_0) //右上
#define SERVO2_PWM     (TCPWM_CH10_P05_1) //左上
#define SERVO3_PWM     (TCPWM_CH11_P05_2) //右下
#define SERVO4_PWM     (TCPWM_CH12_P05_3) //左下

// ================= 舵机参数 =================
#define SERVO_FREQ     (50)

#define SERVO_LEFT     (0)
#define SERVO_RIGHT    (180)

// 舵机角度转PWM占空比
#define SERVO_DUTY(x) \
((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_FREQ)*(0.5+(float)(x)/90.0))

float angle = 90;
uint8 dir = 1;

//int main(void)
//{
//    // 系统初始化
//    clock_init(SYSTEM_CLOCK_250M);
//    debug_init();
//
//    // PWM初始化
////    pwm_init(SERVO1_PWM, SERVO_FREQ, 0);
////    pwm_init(SERVO2_PWM, SERVO_FREQ, 0);
////    pwm_init(SERVO3_PWM, SERVO_FREQ, 0);
//    pwm_init(SERVO4_PWM, SERVO_FREQ, 0);
//
//    while(true)
//    {
//        // 来回摆动
//        if(dir)
//        {
//            angle++;
//
//            if(angle >= SERVO_RIGHT)
//            {
//                dir = 0;
//            }
//        }
//        else
//        {
//            angle--;
//
//            if(angle <= SERVO_LEFT)
//            {
//                dir = 1;
//            }
//        }
//
//        // 输出PWM
////        pwm_set_duty(SERVO1_PWM, (uint16)SERVO_DUTY(angle));
////        pwm_set_duty(SERVO2_PWM, (uint16)SERVO_DUTY(angle));
////        pwm_set_duty(SERVO3_PWM, (uint16)SERVO_DUTY(angle));
//        pwm_set_duty(SERVO4_PWM, (uint16)SERVO_DUTY(angle));
//
//        system_delay_ms(10);
//    }
//}