#include "zf_common_headfile.h"
#define wheel_radius 0.03206    // 轮子半径，单位 m

BANLANCE banlance;

void banlance_init(void)
{
    // 角速度环 PID 初始化
    banlance.gyro_pid.kp = 2.5f;           // 比例系数
    banlance.gyro_pid.ki = 0.0f;           // 积分系数
    banlance.gyro_pid.kd = 0.0f;           // 微分系数
    banlance.gyro_pid.maxIntegral = 0.0f;  // 积分限幅
    banlance.gyro_pid.maxOutput = 10000;   // 输出限幅
    banlance.gyro_pid.K = 1.0f;            // 缩放系数

    // 俯仰角度环 PID 初始化
    banlance.pitch_angle_pid.kp = 61.0f;           // 比例系数
    banlance.pitch_angle_pid.ki = 0.4f;           // 积分系数
    banlance.pitch_angle_pid.kd = 5.0f;           // 微分系数
    banlance.pitch_angle_pid.maxIntegral = 0;     // 积分限幅
    banlance.pitch_angle_pid.maxOutput = 10000;   // 输出限幅
    banlance.pitch_angle_pid.K = 1.0f;            // 缩放系数

    // 横滚角度环 PID 初始化
    banlance.roll_angle_pid.kp = 0.2f;            // 比例系数
    banlance.roll_angle_pid.ki = 0.0f;            // 积分系数
    banlance.roll_angle_pid.kd = 0.1f;            // 微分系数
    banlance.roll_angle_pid.maxIntegral = 10;      // 积分限幅
    banlance.roll_angle_pid.maxOutput = 10000;    // 输出限幅
    banlance.roll_angle_pid.K = 1.0f;             // 缩放系数

    // 偏航角度环 PID 初始化
    banlance.yaw_angle_pid.kp = 8.0f;             // 比例系数
    banlance.yaw_angle_pid.ki = 0.0f;             // 积分系数
    banlance.yaw_angle_pid.kd = 0.0f;             // 微分系数
    banlance.yaw_angle_pid.maxIntegral = 0;       // 积分限幅
    banlance.yaw_angle_pid.maxOutput = 10000;     // 输出限幅
    banlance.yaw_angle_pid.K = 1.0f;              // 缩放系数

    // 速度环 PID 初始化
    banlance.speed_pid.kp = 0.5f;            // 比例系数
    banlance.speed_pid.ki = 0.0f;            // 积分系数
    banlance.speed_pid.kd = 0.1f;            // 微分系数
    banlance.speed_pid.maxIntegral = 0;      // 积分限幅
    banlance.speed_pid.maxOutput = 10000;    // 输出限幅
    banlance.speed_pid.K = 1.0f;             // 缩放系数
}

void pid_pos_calc(PID *pid, float reference, float feedback)
{
    pid->lastError = pid->error;                                        // 保存上一次误差
    pid->error = reference - feedback;                                  // 计算当前误差
    pid->integral += pid->error;                                        // 积分累加

    // 积分限幅，防止积分过大
    if(pid->integral > pid->maxIntegral)
    {
        pid->integral = pid->maxIntegral;
    }
    else if(pid->integral < -pid->maxIntegral)
    {
        pid->integral = -pid->maxIntegral;
    }

    float pout = pid->K * pid->error * pid->kp;                         // P 项输出
    float iout = pid->K * pid->integral * pid->ki;                      // I 项输出
    float dout = pid->K * (pid->error - pid->lastError) * pid->kd;      // D 项输出

    pid->output = pout + iout + dout;                                   // PID 输出 = P + I + D

    // 输出限幅
    if(pid->output > pid->maxOutput)
    {
        pid->output = pid->maxOutput;
    }
    else if(pid->output < -pid->maxOutput)
    {
        pid->output = -pid->maxOutput;
    }
}

void pid_inc_calc(PID *pid, float reference, float feedback)
{
    // 更新反馈值
    pid->last_feedback = pid->now_feedback;
    pid->now_feedback = feedback;

    // 更新误差
    pid->last_last_Error = pid->lastError;
    pid->lastError = pid->error;
    pid->error = reference - pid->now_feedback;

    // 计算增量输出
    float deltaOutput = pid->K * (
        pid->kp * (pid->error - pid->lastError) + 
        pid->ki * pid->error + 
        pid->kd * (pid->error - 2.0f * pid->lastError + pid->last_last_Error)
    );

    // 输出增量累加
    pid->output += deltaOutput;

    // 输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}

// rpm 到线速度的转换，单位 m/s
float rpmtotrue(short int rpm)
{
    float omega = rpm * 2.0f * 3.14159265f / 60.0f;       // 角速度 rad/s
    float v = omega * wheel_radius;
    return v;
}