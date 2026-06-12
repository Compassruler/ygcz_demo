#include "zf_common_headfile.h"
#define wheel_radius 0.04550    // 轮子半径，单位 m orin 0 .03206

BANLANCE banlance;

void banlance_init(void)
{
    // 俯仰角速度环 PID 初始化
    // 原始参数: kp=2.5, ki=0.0, kd=0.0, maxIntegral=0.0, maxOutput=10000, K=1.0
    banlance.pitch_gyro_pid.kp = 6.0f;           // 比例系数
    banlance.pitch_gyro_pid.ki = 0.0f;           // 积分系数
    banlance.pitch_gyro_pid.kd = 0.0f;           // 微分系数
    banlance.pitch_gyro_pid.maxIntegral = 0.0f;  // 积分限幅
    banlance.pitch_gyro_pid.maxOutput = 10000;       // 输出限幅
    banlance.pitch_gyro_pid.K = 1.0f;            // 缩放系数

    // 俯仰角度环 PID 初始化
    // 原始参数: kp=60.0, ki=0.4, kd=5.0, maxIntegral=0, maxOutput=10000, K=1.0
    banlance.pitch_angle_pid.kp = 22.0f;           // 比例系数
    banlance.pitch_angle_pid.ki = 1.0f;           // 积分系数
    banlance.pitch_angle_pid.kd = 5.0f;           // 微分系数
    banlance.pitch_angle_pid.maxIntegral = 100;     // 积分限幅
    banlance.pitch_angle_pid.maxOutput = 10000;       // 输出限幅
    banlance.pitch_angle_pid.K = 1.0f;            // 缩放系数

    // 横滚角度环 PID 初始化
    // 原始参数: kp=0.2, ki=0.0, kd=0.1, maxIntegral=10, maxOutput=10000, K=1.0
    banlance.roll_angle_pid.kp = 0.2f;            // 比例系数
    banlance.roll_angle_pid.ki = 0.0f;            // 积分系数
    banlance.roll_angle_pid.kd = 0.0f;            // 微分系数
    banlance.roll_angle_pid.maxIntegral = 0;      // 积分限幅
    banlance.roll_angle_pid.maxOutput = 10000;        // 输出限幅
    banlance.roll_angle_pid.K = 1.0f;             // 缩放系数

    // 偏航角度环 PID 初始化
    // 原始参数: kp=20.0, ki=0.0, kd=0.0, maxIntegral=0, maxOutput=100, K=1.0
    banlance.yaw_angle_pid.kp = 15.0f;             // 比例系数
    banlance.yaw_angle_pid.ki = 0.0f;             // 积分系数
    banlance.yaw_angle_pid.kd = 0.2f;             // 微分系数
    banlance.yaw_angle_pid.maxIntegral = 0;       // 积分限幅
    banlance.yaw_angle_pid.maxOutput = 1000;         // 输出限幅
    banlance.yaw_angle_pid.K = 1.0f;              // 缩放系数
    
    // 偏航角速度环 PID 初始化
    // 原始参数: kp=10.0, ki=0.0, kd=0.0, maxIntegral=0, maxOutput=100, K=1.0
    banlance.yaw_gyro_pid.kp = 10.0f;             // 比例系数
    banlance.yaw_gyro_pid.ki = 0.0f;             // 积分系数
    banlance.yaw_gyro_pid.kd = 0.0f;             // 微分系数
    banlance.yaw_gyro_pid.maxIntegral = 0;       // 积分限幅
    banlance.yaw_gyro_pid.maxOutput = 1000;         // 输出限幅
    banlance.yaw_gyro_pid.K = 1.0f;              // 缩放系数
    
    // 速度环 PID 初始化
    // 原始参数: kp=0.5, ki=0.0, kd=0.1, maxIntegral=0, maxOutput=10000, K=1.0
    banlance.speed_pid.kp = 0.6f;            // 比例系数
    banlance.speed_pid.ki = 0.0f;            // 积分系数
    banlance.speed_pid.kd = 0.0f;            // 微分系数
    banlance.speed_pid.maxIntegral = 0;      // 积分限幅
    banlance.speed_pid.maxOutput = 10000;        // 输出限幅
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
float rpmtotrue(float rpm)
{
    float omega = rpm * 2.0f * PI / 60.0f;       // 角速度 rad/s
    float v = omega * wheel_radius;
    return v;
}

int truetorpm(float v)
{
    float rpm_middle;
    int rpm;
    rpm_middle = v / (2.0f * PI * wheel_radius) * 60.0f;
    rpm = (int)rpm_middle;
    return rpm;

}