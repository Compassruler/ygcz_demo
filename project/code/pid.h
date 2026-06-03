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

typedef struct
{
    PID gyro_pid;          // 角速度环pid结构体
    PID pitch_angle_pid;  // 俯仰角度环pid结构体
    PID roll_angle_pid;   // 横滚角度环pid结构体
    PID yaw_angle_pid;    // 速度环
    PID speed_pid;       // 速度环pid结构体
} BANLANCE;

extern BANLANCE banlance;

// 函数简介：  pid参数初始化
void banlance_init(void);

void pid_init(PID *pid,
              float kp,
              float ki,
              float kd,
              float maxI,
              float maxOut,
              float K);

// 函数简介：       PID计算
// 参数说明：       pid         PID结构体指针
// 参数说明：       reference   目标值
// 参数说明：       feedback    当前反馈值
// 返回值：         void
void pid_pos_calc(PID *pid, float reference, float feedback);
void pid_inc_calc(PID *pid, float reference, float feedback);

// rpm转为真实线速度单位 m/s
float rpmtotrue(short int rpm);
// 真实线速度(m/s) 转为prm 转/min
int truetorpm(float v);
#endif