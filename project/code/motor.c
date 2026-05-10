#include "zf_common_headfile.h"



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

void pid_calc(PID *pid, float reference, float feedback)
{
    pid->lastError = pid->error;                                        // 保存上一次误差
    pid->error = reference - feedback;                                  // 计算当前误差
    float pout =pid->K * pid->error * pid->kp;                          // 比例项 P 
    float dout =pid->K * (pid->error - pid->lastError) * pid->kd;       // 微分项 D
    pid->output = pout + dout;                                          // PD输出 = P + D

    if(pid->output > pid->maxOutput)                                    // 防止输出超过电机允许范围
    {
        pid->output = pid->maxOutput;
    }
    else if(pid->output < -pid->maxOutput)
    {
        pid->output = -pid->maxOutput;
    }
}