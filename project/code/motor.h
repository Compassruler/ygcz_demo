#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"


#define SERVO_MOTOR_DUTY(x) \ ((float)10000 / (1000.0f / 300.0f) * (0.5f + (float)(x) / 90.0f)) // 舵机PWM占空比换算

// PID结构体
typedef struct
{
    // ================= PID参数 =================
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float kd;                 // 微分系数

    // ================= 误差 =================
    float error;              // 当前误差
    float lastError;          // 上一次误差
    float last_last_Error;    // 上上次误差

    // ================= 积分 =================
    float integral;           // 积分累计值
    float maxIntegral;        // 积分限幅

    // ================= 输出 =================
    float output;             // PID输出
    float maxOutput;          // 输出限幅

    // ================= 反馈值 =================
    float now_feedback;       // 当前反馈值
    float last_feedback;      // 上次反馈值

    // ================= 缩放系数 =================
    float K;                  // PID整体缩放系数

} PID;

typedef struct // 暂时还没用
{
  PID gyro_cycle;          // 角速度环
  PID angle_cycle;         // 角度环
  PID speed_cycle;         // 速度环
  PID leg_x_left_pid;
  PID leg_y_left_pid;
  PID leg_x_right_pid;
  PID leg_y_right_pid;

  float call_cycle;

} BANLANCE;

extern BANLANCE banlance;     // 三环集合结构体
extern PID gyro_pid;          // 角速度环pid结构体
extern PID pitch_angle_pid;  // 俯仰角度环pid结构体
extern PID roll_angle_pid;   // 横滚角度环pid结构体
extern PID speed_pid;       // 速度环pid结构体


// 函数简介：  三串级pid参数初始化
void banlance_init(void);

// 函数简介：   PID初始化
// 参数说明：   pid         PID结构体指针
// 参数说明：   p           比例系数
// 参数说明：   i           积分系数
// 参数说明：   d           微分系数
// 参数说明：   maxI        积分限幅
// 参数说明：   maxOut      输出限幅
// 参数说明：   K           缩放系数
// 返回值：     void
void pid_init(PID *pid, float p, float i, float d, float maxI, float maxOut, float K);

// 函数简介：       PID计算
// 参数说明：       pid         PID结构体指针
// 参数说明：       reference   目标值
// 参数说明：       feedback    当前反馈值
// 返回值：         void
void pid_pos_calc(PID *pid, float reference, float feedback);
void pid_inc_calc(PID *pid, float reference, float feedback);
#endif