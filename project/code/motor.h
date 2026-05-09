#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_headfile.h"

// PID 控制器结构体
typedef struct {
    // PID 基本系数
    float kp;          // 比例系数
    float ki;          // 积分系数
    float kd;          // 微分系数

    // 高阶或串级扩展系数（可选）
    float kd3;         // 高阶微分系数，可用于三点微分或特殊场景
    float kp3;         // 高阶比例系数，可用于快速响应场景

    // 误差值
    float error;           // 当前误差
    float last_error;      // 上一次误差
    float last_last_error; // 上上次误差（高阶 PID 可用）
    
    // 辅助误差（可用于二阶积分/预测）
    float error1;
    float error2;
    float last_error2;

    // 积分相关
    float integral;        // 积分累计
    float max_integral;    // 积分限幅

    // 输出相关
    float output;          // PID 输出
    float max_output;      // 输出限幅

    // 反馈值
    float now_feedback;    // 当前测量值
    float last_feedback;   // 上一次测量值

    // 额外增益系数，可用于自适应调节
    float K;
} PID_t;


void PID_Init(PID_t *pid,float kp, float ki, float kd,
              float kp3, float kd3,
              float max_integral, float max_output,
              float K);

float PID_Calc(PID_t *pid, float setpoint, float feedback);

void Balance_Control(float angle_ref, float angle_fb, float gyro_fb);
void PID_Init_All(void);
#endif