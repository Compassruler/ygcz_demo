#ifndef _SERVO_H_
#define _SERVO_H_

// 舵机 PWM 通道定义
#define SERVO1_PWM     (TCPWM_CH09_P05_0) // 前左 通道 0
#define SERVO2_PWM     (TCPWM_CH10_P05_1) // 后右 通道 0
#define SERVO3_PWM     (TCPWM_CH11_P05_2) // 后左 通道 0
#define SERVO4_PWM     (TCPWM_CH12_P05_3) // 前右 通道 0

// 舵机参数
#define SERVO_FREQ     (50)
#define SERVO_LEFT     (0)
#define SERVO_RIGHT    (180)

// 舵机占空比计算PWM输出
#define SERVO_DUTY(x) ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_FREQ)*(0.5+(float)(x)/90.0))

// 偏移量
#define X_OFFSET    18.5f       // X 轴偏移量
#define Y_OFFSET    25.0f       // Y 轴偏移量

// 舵机步进量
#define SERVO_STEP   2.0f

// 腿长参数，单位：cm
extern float L1, L2, L3, L4, L5;

// 当前坐标
extern float X_L, Y_L, X_R, Y_R;

// 当前舵机角度，初始均为 90 度，对应后续角度输出变量
extern float servoLeftFront_now, servoLeftRear_now, servoRightFront_now, servoRightRear_now;

// 速度转X偏移，平衡转Y偏移
extern float speed_to_x_offset;
extern float balance_to_y_offset;

extern float X_left, X_right, Y_left, Y_right;

// 边界范围参数
extern float XLeft, YLeft, XRight, YRight;

// 运动方程中间变量
extern float aLeft, bLeft, cLeft, dLeft, eLeft, fLeft;
extern float aRight, bRight, cRight, dRight, eRight, fRight;

// 控制角度
extern float alphaLeft, betaLeft, alphaRight, betaRight;
extern float alpha1, alpha2, beta1, beta2;

// 舵机角度
extern float servoLeftFront, servoLeftRear, servoRightFront, servoRightRear;

// 滤波参数
extern float leg_x_left_filter, leg_y_left_filter, leg_x_right_filter, leg_y_right_filter;

typedef void (*HandlerFunc)(int step_num);

typedef struct
{
    int min;                  // 执行时间起点
    int max;                  // 执行时间结束
    HandlerFunc handler;        // 执行内容
    const char *description;   // 行为描述

}jump_control_struct;

extern float servoLeftFront_jump , servoLeftRear_jump , servoRightFront_jump , servoRightRear_jump ;
extern float servoLeftFront_jump_now , servoLeftRear_jump_now , servoRightFront_jump_now , servoRightRear_jump_now;

// 函数功能 舵机初始化
void servo_init(void);

// 函数名称     servo_step
// 功能说明     舵机角度过渡
// 参数说明     now    当前舵机角度
// 参数说明     target 目标舵机角度
// 参数说明     step   过渡步长
float servo_step(float now, float target, float step);

// 函数功能 设置舵机角度
// 参数说明 angle1  servoLeftFront
// 参数说明 angle2  servoLeftRear
// 参数说明 angle3  servoRightFront
// 参数说明 angle4  servoRightRear
void Set_angle(float angle1, float angle2, float angle3, float angle4);

// 函数名称     leg_disable
// 功能说明     关闭舵机输出
void leg_disable(void);

// 函数名称     solve_inverse_kinematics
// 功能说明     计算逆运动学参数
// 参数说明     x           X 坐标，前后方向偏移值
// 参数说明     y           Y 坐标，上下方向偏移值
// 参数说明     a, b, c     左腿平移系数
// 参数说明     d, e, f     右腿平移系数
// 参数说明     alpha       输出前段角度
// 参数说明     beta        输出后段角度
void solve_inverse_kinematics(float x, float y, float *a, float *b, float *c, float *d, float *e, float *f, float *alpha, float *beta);

// 函数名称     rad_to_deg
// 功能说明     将弧度转换为角度
// 参数说明     rad         弧度值
// 返回值       角度值
float rad_to_deg(float rad);

// 函数名称     calculate_servo_angle
// 功能说明     计算舵机前后角度
// 参数说明     alpha       前段弧度
// 参数说明     beta        后段弧度
// 参数说明     front       前段舵机角度输出
// 参数说明     rear        后段舵机角度输出
void calculate_servo_angle(float alpha, float beta, float *front, float *rear);

// 函数名称     leg_control
// 功能说明     控制腿部舵机动作
// 参数说明     X_left      左腿目标 X 坐标
// 参数说明     X_right     右腿目标 X 坐标
// 参数说明     Y_left      左腿目标 Y 坐标
// 参数说明     Y_right     右腿目标 Y 坐标
void leg_control(void);

void jump_step_a(int step_num);

void jump_control(void);

extern int jump_flag;
extern int jump_time;

#endif