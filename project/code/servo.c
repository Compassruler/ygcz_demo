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

void servo_init()
{
  // pwm初始化
  pwm_init(SERVO1_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO2_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO3_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO4_PWM, SERVO_FREQ, 0); 
  
  // 舵机为0°
  pwm_set_duty(SERVO1_PWM, (uint16)SERVO_DUTY(90));
  pwm_set_duty(SERVO2_PWM, (uint16)SERVO_DUTY(90));
  pwm_set_duty(SERVO3_PWM, (uint16)SERVO_DUTY(90));
  pwm_set_duty(SERVO4_PWM, (uint16)SERVO_DUTY(90));
}