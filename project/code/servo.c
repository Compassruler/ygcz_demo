#include "zf_common_headfile.h"

float L1 = 60.0f;
float L2 = 90.0f;
float L3 = 90.0f;
float L4 = 60.0f;
float L5 = 37.0f;

// 当前坐标
float X_L = 0.0f, Y_L = 0.0f;    // 左腿当前坐标
float X_R = 0.0f, Y_R = 0.0f;    // 右腿当前坐标

// 当前舵机角度
float servoLeftFront_now  = 110.0f;
float servoLeftRear_now   = 110.0f;
float servoRightFront_now = 110.0f;
float servoRightRear_now  = 110.0f;

float speed_to_x_offset, balance_to_y_offset;

// 目标舵机坐标 20（90度）
float X_left = 0.0f;
float Y_left = 50.0f;
float X_right = 0.0f;
float Y_right = 50.0f;

// 当前实际坐标（0为初始值）
float XLeft = 0.0f, YLeft = 0.0f;
float XRight = 0.0f, YRight = 0.0f;

// 一阶方程计算变量
float aLeft, bLeft, cLeft, dLeft, eLeft, fLeft;
float aRight, bRight, cRight, dRight, eRight, fRight;
float alpha1, alpha2, beta1, beta2;
float alphaLeft, betaLeft, alphaRight, betaRight;

// 角度中间变量
float alphaLeftToAngle, betaLeftToAngle;
float alphaRightToAngle, betaRightToAngle;
float servoLeftFront, servoLeftRear;    // 左右舵机角度
float servoRightFront, servoRightRear;  // 前后舵机角度


// jump部分
const jump_control_struct jump_control_config[] =
{
    {  0, 70, jump_step_a, "起跳"     },
    {70, 250, jump_step_a, "收脚"     },
    {250, 330, jump_step_a, "准备缓冲" },
    {330, 430, jump_step_a, "执行缓冲" },
};

const uint8 jump_step = sizeof(jump_control_config) / sizeof(jump_control_struct);

int jump_flag = 0;
int jump_time = 0;

float servoLeftFront_jump = 0, servoLeftRear_jump = 0, servoRightFront_jump = 0, servoRightRear_jump = 0;
float servoLeftFront_jump_now = 0, servoLeftRear_jump_now = 0, servoRightFront_jump_now = 0, servoRightRear_jump_now= 0;

void servo_init()
{
  // PWM 初始化
  pwm_init(SERVO1_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO2_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO3_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO4_PWM, SERVO_FREQ, 0); 
  
  Set_angle(servoLeftFront_now, servoLeftRear_now, servoRightFront_now, servoRightRear_now);
}

float servo_step(float now, float target, float step)
{
    if(target > now + step)
    {
        now += step;
    }
    else if(target < now - step)
    {
        now -= step;
    }
    else
    {
        now = target;
    }
    return now;
}

void Set_angle(float angle1, float angle2, float angle3, float angle4)
{
  // 右侧舵机方向校正，右侧后置角度取 180 - 角度
    angle2 = 180.0f - angle2;
    angle3 = 180.0f - angle3;

    // 角度限幅
    if(angle1 > 175) angle1 = 175;
    if(angle1 < 5)   angle1 = 5;

    if(angle2 > 175) angle2 = 175;
    if(angle2 < 5)   angle2 = 5;

    if(angle3 > 175) angle3 = 175;
    if(angle3 < 5)   angle3 = 5;

    if(angle4 > 175) angle4 = 175;
    if(angle4 < 5)   angle4 = 5;
    
    // PWM 输出
    pwm_set_duty(SERVO1_PWM, (uint16)SERVO_DUTY(angle1));
    pwm_set_duty(SERVO3_PWM, (uint16)SERVO_DUTY(angle2));
    pwm_set_duty(SERVO4_PWM, (uint16)SERVO_DUTY(angle3));
    pwm_set_duty(SERVO2_PWM, (uint16)SERVO_DUTY(angle4));
}

void leg_disable(void)
{
  Set_angle(0, 0, 0, 0);
}

void solve_inverse_kinematics(float x, float y, float *a, float *b, float *c, float *d, float *e, float *f, float *alpha, float *beta)
{
    // 计算移动矩阵系数
    *a = 2.0f * x * L1;
    *b = 2.0f * y * L1;
    *c = x * x + y * y + L1 * L1 - L2 * L2;
    *d = 2.0f * L4 * (x - L5);
    *e = 2.0f * L4 * y;
    *f = (x - L5) * (x - L5) + L4 * L4 + y * y - L3 * L3;
    
    // 计算前腿两组角度判别式
    float discriminant_alpha = (*a) * (*a) + (*b) * (*b) - (*c) * (*c);
    if (discriminant_alpha < 0)
    {
      *alpha = 0;
      *beta  = 0; // 计算失败时设为 0
      return;
    }
    
    float denominator1 = (*a) + (*c);
    
    if(fabs(denominator1) < 0.0001f)denominator1 = 0.0001f;

    alpha1 = 2.0f * atan(((*b) + sqrt(discriminant_alpha)) / denominator1);
    alpha2 = 2.0f * atan(((*b) - sqrt(discriminant_alpha)) / denominator1);
    
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2.0f * PI);               // 归一化到 [0, 2π]
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2.0f * PI);
    
    *alpha = (alpha1 >= PI / 4.0f) ? alpha1 : alpha2;                     // 选择合适的角度，优先选择 [π/4, 2π] 区间
    
    // 璁＄畻尾鐨勪袱涓?瑙?
    float discriminant_beta = (*d) * (*d) + (*e) * (*e) - (*f) * (*f);
    if (discriminant_beta < 0)
    {
      *alpha = 0;
      *beta  = 0; // 计算失败时设为 0
      return;
    }
    
    float denominator2 = (*d) + (*f);

    if(fabs(denominator2) < 0.0001f) denominator2 = 0.0001f;
    
    beta1 = 2.0f * atan(((*e) + sqrt(discriminant_beta)) / denominator2);
    beta2 = 2.0f * atan(((*e) - sqrt(discriminant_beta)) / denominator2);
    
    *beta = (beta1 >= 0 && beta1 <= PI / 4.0f) ? beta1 : beta2;         // 选择合适的角度，优先选择 [0, π/4] 区间
}

float rad_to_deg(float rad)
{
  return (rad / (2.0f * PI)) * 360.0f;
}

void calculate_servo_angle(float alpha, float beta, float *front, float *rear)
{
    float alpha_deg = rad_to_deg(alpha);
    float beta_deg = rad_to_deg(beta);
    
    *front = 180.0f - (alpha_deg - 90.0f);
    *rear = 180.0f - (90.0f - beta_deg);
}

// 速度偏移和舵机角度坐标对应
void leg_control(void)
{
  if(auto_protect_flag|| manual_protect_flag == 1)
    {
      leg_disable();
    }
  else
    {
    static float speed_offset_filter = 0;
    static float roll_offset_filter  = 0;

    // 跳跃时减弱 PID 影响
    float pid_scale = (jump_flag == 1) ? 0.2f : 1.0f;
    
    // X 方向：俯仰偏移计算
    float target_offset = banlance.speed_pid.output * 0.1f * pid_scale;
    speed_offset_filter = (speed_offset_filter * 19.0f + target_offset) / 20.0f;
    speed_to_x_offset = func_limit_ab(speed_offset_filter, -45.0f, 45.0f);
    
    // Y 方向：横滚偏移计算
//    float roll_target_offset = 0;
    float roll_target_offset = banlance.roll_angle_pid.output * 5.0f * pid_scale;
    if(fabs(roll_filter.filtering_angle) < 1.0f) roll_target_offset = 0;
    roll_offset_filter = (roll_offset_filter * 19.0f + roll_target_offset) / 20.0f;
    balance_to_y_offset = func_limit_ab(roll_offset_filter, -100.0f, 100.0f);
  
    // 输入限幅
    if(Y_left < 10) Y_left = 10;
    if(Y_right < 10) Y_right = 10;

    if(Y_left > 130) Y_left = 130;
    if(Y_right > 130) Y_right = 130;
     
    // 目标坐标计算
    float target_X_left = X_left + X_OFFSET - speed_to_x_offset;
    float target_X_right = X_right + X_OFFSET - speed_to_x_offset;
  
    // Y 方向：只做伸长
//    float target_Y_left  = Y_left  + Y_OFFSET + (balance_to_y_offset > 0.0f ? balance_to_y_offset : 0.0f);
//    float target_Y_right = Y_right + Y_OFFSET - (balance_to_y_offset > 0.0f ? 0.0f : balance_to_y_offset);
    
    // Y 方向：一边伸长 一边收缩（效果好）
    float target_Y_left  = Y_left  + Y_OFFSET + balance_to_y_offset;
    float target_Y_right = Y_right + Y_OFFSET - balance_to_y_offset;
                                                 
    // 更新实际坐标
    XLeft  = target_X_left;
    YLeft  = target_Y_left;
    XRight = target_X_right;
    YRight = target_Y_right;
  
    // 计算左腿逆运动学角度
    solve_inverse_kinematics(XLeft, YLeft, &aLeft, &bLeft, &cLeft, &dLeft, &eLeft, &fLeft, &alphaLeft, &betaLeft);
    
    // 计算右腿逆运动学角度
    solve_inverse_kinematics(XRight, YRight, &aRight, &bRight, &cRight, &dRight, &eRight, &fRight, &alphaRight, &betaRight);
    
    // 计算舵机角度
    calculate_servo_angle(alphaLeft, betaLeft, &servoLeftFront, &servoLeftRear);
    calculate_servo_angle(alphaRight, betaRight, &servoRightFront, &servoRightRear);
  
    // 舵机步进
    servoLeftFront_now  = servo_step(servoLeftFront_now, servoLeftFront,  SERVO_STEP);
    servoLeftRear_now   = servo_step(servoLeftRear_now, servoLeftRear,   SERVO_STEP);
    servoRightFront_now = servo_step(servoRightFront_now, servoRightFront, SERVO_STEP);
    servoRightRear_now  = servo_step(servoRightRear_now, servoRightRear,  SERVO_STEP);

    if(jump_flag == 0)
    {
      Set_angle(servoLeftFront_now, servoLeftRear_now, servoRightFront_now, servoRightRear_now);        // 输出舵机角度
    }
    }
}

void jump_step_a(int step_num)
{
  switch(step_num)
  {
    case 0: // 起跳蓄力
    {
      Set_angle(140, 140, 140, 140);
    }break;

    case 1: // 收腿
    {
      Set_angle(90, 90, 90, 90);
    }break;

    case 2:  // 准备缓冲
    {
//      Set_angle(90, 90, 90, 90);
      // 初始化当前值
//      servoLeftFront_jump  = 90;
//      servoLeftRear_jump   = 90;
//      servoRightFront_jump = 90;
//      servoRightRear_jump  = 90;
    }break;

    case 3: // 执行缓冲（平滑）
    {
//      servoLeftFront_jump  = servo_step(servoLeftFront_jump, 90, 2);
//      servoLeftRear_jump   = servo_step(servoLeftRear_jump, 90, 2);
//      servoRightFront_jump = servo_step(servoRightFront_jump, 90, 2);
//      servoRightRear_jump  = servo_step(servoRightRear_jump, 90, 2);
//
//      Set_angle(servoLeftFront_jump, servoLeftRear_jump, servoRightFront_jump, servoRightRear_jump);
    }break;

    default: break;
  }
}

void jump_control(void)
{
  if (jump_flag)
  {
    jump_time++;

    if (jump_time < jump_control_config[jump_step - 1].max)
    {
      for (int i = 0; i < jump_step; i++)
      {
        if (jump_time >= jump_control_config[i].min && jump_time <= jump_control_config[i].max)
        {
          jump_control_config[i].handler(i);
          break;
        }
      }
    }
    else
    {
      jump_flag = 0;
      jump_time = 0;
    }
  }
}