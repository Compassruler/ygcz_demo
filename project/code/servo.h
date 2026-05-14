#ifndef _SERVO_H_
#define _SERVO_H_

// 舵机PWM引脚
#define SERVO1_PWM     (TCPWM_CH09_P05_0) // 左前 上0°  
#define SERVO2_PWM     (TCPWM_CH10_P05_1) // 右后 上0°
#define SERVO3_PWM     (TCPWM_CH11_P05_2) // 左后 下0°
#define SERVO4_PWM     (TCPWM_CH12_P05_3) // 右前 下0°

// 舵机参数
#define SERVO_FREQ     (50)
#define SERVO_LEFT     (0)
#define SERVO_RIGHT    (180)

// 舵机角度转PWM占空比
#define SERVO_DUTY(x) ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_FREQ)*(0.5+(float)(x)/90.0))

// 偏移量
#define X_OFFSET    0.0f       // X轴偏移量18.5f 
#define Y_OFFSET    25.0f       // Y轴偏移量

// 步进值
#define SERVO_STEP   0.5f

// 五连杆参数 （mm）
extern float L1, L2, L3, L4, L5;

// 当前坐标
extern float X_L, Y_L;
extern float X_R, Y_R;

// 初始角度（第一次为90°，后续角度变量做为步进输出）
extern float servoLeftFront_now;
extern float servoLeftRear_now;
extern float servoRightFront_now;
extern float servoRightRear_now;

// 动态偏移量（速度环输出）
extern float speed_to_x_offset;
extern float balance_to_y_offset;

extern float X_left;
extern float X_right;
extern float Y_left;
extern float Y_right;

// 目标坐标
extern float XLeft, YLeft;
extern float XRight, YRight;

// 逆运动学中间变量
extern float aLeft, bLeft, cLeft, dLeft, eLeft, fLeft;
extern float aRight, bRight, cRight, dRight, eRight, fRight;

// 关节角
extern float alphaLeft, betaLeft, alphaRight, betaRight;
extern float alpha1, alpha2, beta1, beta2;

// 舵机角度
extern float servoLeftFront, servoLeftRear;
extern float servoRightFront, servoRightRear;

// 滤波参数
extern float leg_x_left_filter;
extern float leg_y_left_filter;
extern float leg_x_right_filter;
extern float leg_y_right_filter;

// 函数简介： 舵机初始化
void servo_init(void);

// 函数简介： 舵机步进
// 参数说明：now    舵机当前角度
// 参数说明：target 舵机目标角度
// 参数说明：step   步进值
float servo_step(float now, float target, float step);

// 函数简介： 设置舵机角度
// 参数说明：angle1  servoLeftFront
// 参数说明：angle2  servoLeftRear
// 参数说明：angle3  servoRightFront
// 参数说明：angle4  servoRightRear
void Set_angle(float angle1, float angle2, float angle3, float angle4);

// 函数简介： 腿部保护
void leg_disable(void);

// 函数简介： 五连杆运动学解算
// 参数说明： x           X坐标 前正后负 以芯片方向为前
// 参数说明： y           Y坐标 下正上负
// 参数说明： a, b, c     左侧运动学参数
// 参数说明： d, e, f     右侧运动学参数
// 参数说明： alpha       输出角度α
// 参数说明： beta        输出角度β
void solve_inverse_kinematics(float x, float y, float *a, float *b, float *c, float *d, float *e, float *f, float *alpha, float *beta);

// 函数简介: 弧度转角度
// 参数说明: rad         弧度值
// 返回类型:   角度值
float rad_to_deg(float rad);

// 函数名称: 计算舵机角度
// 参数说明: alpha       关节角α（弧度）
// 参数说明: beta        关节角β（弧度）
// 参数说明: front       前舵机角度输出
// 参数说明: rear        后舵机角度输出
void calculate_servo_angle(float alpha, float beta, float *front, float *rear);

// 函数简介： 腿部控制（速度环）
// 参数说明:  X_left      左侧X轴目标位置
// 参数说明:  X_right     右侧X轴目标位置
// 参数说明:  Y_left      左侧Y轴目标位置
// 参数说明:  Y_right     右侧Y轴目标位置
void leg_control(void);

#endif