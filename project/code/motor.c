#include "zf_common_headfile.h"

PID gyro_pid;
PID angle_pid;
PID speed_pid;

// 这个是在servo.c里的pd控制，后续整合到一起
PID leg_x_left_pid;
PID leg_y_left_pid;
PID leg_x_right_pid;
PID leg_y_right_pid;


void banlance_init(void)
{
  pid_init(&gyro_pid, 3.0f, 0.0f, 0.0f, 0, 10000, 1.0f);   // 角速度环PID初始化
  pid_init(&angle_pid, 60.0f, 0.5f, 5.0f, 0, 10000, 1.0f); // 角度环PID初始化
  pid_init(&speed_pid, 0.0f, 0.0f, 0.0f, 0, 10000, 1.0f); // 速度环PID初始化
  
  pid_init(&leg_x_left_pid, 1.0f, 0.0f, 0.0f, 0, 10000, 1.0f); 
  pid_init(&leg_x_right_pid, 1.0f, 0.0f, 0.0f, 0, 10000, 1.0f); 
  pid_init(&leg_y_left_pid, 1.0f, 0.0f, 0.0f, 0, 10000, 1.0f); 
  pid_init(&leg_y_right_pid, 1.0f, 0.0f, 0.0f, 0, 10000, 1.0f); 
}

void pid_init(PID *pid, float p, float i, float d, float maxI, float maxOut, float K)
{
    pid->kp = p;                    // 比例系数
    pid->ki = i;                    // 积分系数
    pid->kd = d;                    // 微分系数
    pid->maxIntegral = maxI;        // 积分最大值限制
    pid->maxOutput = maxOut;        // 输出最大值限制
    pid->K = K;                     // PID整体缩放系数
    pid->error = 0;
    pid->lastError = 0;  
    pid->output = 0;
}

void pid_pos_calc(PID *pid, float reference, float feedback)
{
    pid->lastError = pid->error;                                        // 保存上一次误差
    pid->error = reference - feedback;                                  // 计算当前误差
    pid->integral += pid->error;                                        // 积分累加

    // 积分限幅 防止积分过大
    if(pid->integral > pid->maxIntegral)
    {
        pid->integral = pid->maxIntegral;
    }
    else if(pid->integral < -pid->maxIntegral)
    {
        pid->integral = -pid->maxIntegral;
    }

    float pout = pid->K * pid->error * pid->kp;                         // 比例项 P
    float iout = pid->K * pid->integral * pid->ki;                      // 积分项 I
    float dout = pid->K * (pid->error - pid->lastError) * pid->kd;      // 微分项 D

    pid->output = pout + iout + dout;                                   // PID输出 = P + I + D

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
    // 更新反馈
    pid->last_feedback = pid->now_feedback;
    pid->now_feedback = feedback;

    // 更新误差
    pid->last_last_Error = pid->lastError;
    pid->lastError = pid->error;
    pid->error = reference - pid->now_feedback;

    // 计算增量
    float deltaOutput = pid->K * (
        pid->kp * (pid->error - pid->lastError) + 
        pid->ki * pid->error + 
        pid->kd * (pid->error - 2.0f * pid->lastError + pid->last_last_Error)
    );

    // 输出累加
    pid->output += deltaOutput;

    // 输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}