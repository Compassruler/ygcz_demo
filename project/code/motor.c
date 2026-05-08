/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 tc264 V2.6主板 按照下述方式进行接线
//      模块引脚    单片机引脚
//      RX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_TX  宏定义 默认 P10_1
//      TX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_RX  宏定义 默认 P10_0
//      GND         GND

// ======================
// PID 初始化函数
// ======================
void PID_Init(PID_t *pid,
              float kp, float ki, float kd,
              float kp3, float kd3,
              float max_integral, float max_output,
              float K)
{
    if (!pid) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kp3 = kp3;
    pid->kd3 = kd3;

    // 清零历史误差
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->error1 = 0.0f;
    pid->error2 = 0.0f;
    pid->last_error2 = 0.0f;

    // 积分清零并设置上限
    pid->integral = 0.0f;
    pid->max_integral = max_integral;

    // 输出清零并设置上限
    pid->output = 0.0f;
    pid->max_output = max_output;

    // 反馈值清零
    pid->now_feedback = 0.0f;
    pid->last_feedback = 0.0f;

    pid->K = K;
}

// ======================
// PID 计算函数
// ======================
float PID_Calc(PID_t *pid, float setpoint, float feedback)
{
    pid->now_feedback = feedback;
    pid->error = setpoint - feedback;

    // 积分累加
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    // PID 输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * (pid->error - pid->last_error);

    // 高阶 PID 可选
    pid->output += pid->kp3 * pid->error + pid->kd3 * (pid->error - 2*pid->last_error + pid->last_last_error);

    // 输出限幅
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    // 更新历史误差
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->last_feedback = feedback;

    return pid->output;
}

// ======================
// 串级 PID 控制函数（使用滤波后的 IMU 数据）
// ======================
void PID_Cascade_Update(PID_t *speed_pid, PID_t *angle_pid, PID_t *gyro_pid,
                        float speed_ref, float speed_fb,
                        float angle_fb, float gyro_fb,
                        int16_t *PWM_left, int16_t *PWM_right,
                        float PWM_base)
{
    // 1. 外环：速度 PID → 输出目标角度
    float angle_ref = PID_Calc(speed_pid, speed_ref, speed_fb);

    // 2. 中环：角度 PID → 输出目标角速度
    float omega_ref = PID_Calc(angle_pid, angle_ref, angle_fb);

    // 3. 内环：角速度 PID → 输出 PWM 调整量
    float PWM_delta = PID_Calc(gyro_pid, omega_ref, gyro_fb);

    // 4. 左右轮 PWM 分配
    *PWM_left  = (int16_t)(PWM_base - PWM_delta);
    *PWM_right = (int16_t)(PWM_base + PWM_delta);
}

// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看逐飞助手上位机打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 英飞凌TriCore 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看逐飞助手上位机设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：无刷电机无反应
//      检查Rx信号引脚是否接对，信号线是否松动
// 问题4：无刷电机转动但转速显示无速度
//      检查Tx信号引脚是否接对，信号线是否松动