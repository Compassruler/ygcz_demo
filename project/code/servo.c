#include "zf_common_headfile.h"

float L1 = 60.0f;
float L2 = 90.0f;
float L3 = 90.0f;
float L4 = 60.0f;
float L5 = 37.0f;

// 当前位置
float X_L = 0.0f, Y_L = 0.0f;    // 左侧当前位置
float X_R = 0.0f, Y_R = 0.0f;    // 右侧当前位置

// 目标位置
float XLeft = 0.0f, YLeft = 0.0f;
float XRight = 0.0f, YRight = 0.0f;

// 中间计算变量
float aLeft, bLeft, cLeft, dLeft, eLeft, fLeft;
float aRight, bRight, cRight, dRight, eRight, fRight;
float alpha1, alpha2, beta1, beta2;
float alphaLeft, betaLeft, alphaRight, betaRight;

// 角度变量
float alphaLeftToAngle, betaLeftToAngle;
float alphaRightToAngle, betaRightToAngle;
float servoLeftFront, servoLeftRear;    // 左前 左后
float servoRightFront, servoRightRear;  // 右前 右后        对应后删除这个

void servo_init()
{
  // pwm初始化
  pwm_init(SERVO1_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO2_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO3_PWM, SERVO_FREQ, 0);
  pwm_init(SERVO4_PWM, SERVO_FREQ, 0); 
  
  // 舵机为0°
//  Set_angle(113, 113, 113, 113);
}

void Set_angle(float angle1, float angle2, float angle3, float angle4)
{
  // 舵机方向修正  修正后都为上0°
    angle2 = 180.0f - angle2;
    angle3 = 180.0f - angle3;

    // ================= 舵机限幅 =================

    if(angle1 > 175) angle1 = 175;
    if(angle1 < 5)   angle1 = 5;

    if(angle2 > 175) angle2 = 175;
    if(angle2 < 5)   angle2 = 5;

    if(angle3 > 175) angle3 = 175;
    if(angle3 < 5)   angle3 = 5;

    if(angle4 > 175) angle4 = 175;
    if(angle4 < 5)   angle4 = 5;

    // ================= PWM输出 =================

    pwm_set_duty(SERVO1_PWM,
                 (uint16)SERVO_DUTY(angle1));

    pwm_set_duty(SERVO3_PWM,
                 (uint16)SERVO_DUTY(angle2));

    pwm_set_duty(SERVO4_PWM,
                 (uint16)SERVO_DUTY(angle3));

    pwm_set_duty(SERVO2_PWM,
                 (uint16)SERVO_DUTY(angle4));
}

void leg_disable(void)
{
  pwm_set_duty(SERVO1_PWM, (uint16)SERVO_DUTY(0));
  pwm_set_duty(SERVO2_PWM, (uint16)SERVO_DUTY(0));
  pwm_set_duty(SERVO3_PWM, (uint16)SERVO_DUTY(0));
  pwm_set_duty(SERVO4_PWM, (uint16)SERVO_DUTY(0));
}

void solve_inverse_kinematics(float x, float y, float *a, float *b, float *c, float *d, float *e, float *f, float *alpha, float *beta)
{
    // 计算运动学参数
    *a = 2.0f * x * L1;
    *b = 2.0f * y * L1;
    *c = x * x + y * y + L1 * L1 - L2 * L2;
    *d = 2.0f * L4 * (x - L5);
    *e = 2.0f * L4 * y;
    *f = (x - L5) * (x - L5) + L4 * L4 + y * y - L3 * L3;
    
    // 计算α的两个解
    float discriminant_alpha = (*a) * (*a) + (*b) * (*b) - (*c) * (*c);
    if (discriminant_alpha < 0)
    {
      *alpha = 0;
      *beta  = 0; // 无解时返回0
      return;
    }
    
    float denominator1 = (*a) + (*c);
    
    if(fabs(denominator1) < 0.0001f)denominator1 = 0.0001f;

    alpha1 = 2.0f * atan(((*b) + sqrt(discriminant_alpha)) / denominator1);
    alpha2 = 2.0f * atan(((*b) - sqrt(discriminant_alpha)) / denominator1);
    
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2.0f * PI);               // 归一化到[0, 2π]
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2.0f * PI);
    
    *alpha = (alpha1 >= PI / 4.0f) ? alpha1 : alpha2;                     // 选择合适的解（角度在[π/4, 2π]范围内）
    
    // 计算β的两个解
    float discriminant_beta = (*d) * (*d) + (*e) * (*e) - (*f) * (*f);
    if (discriminant_beta < 0)
    {
      *alpha = 0;
      *beta  = 0; // 无解时返回0
      return;
    }
    
    beta1 = 2.0f * atan(((*e) + sqrt(discriminant_beta)) / ((*d) + (*f)));
    beta2 = 2.0f * atan(((*e) - sqrt(discriminant_beta)) / ((*d) + (*f)));
    
    *beta = (beta1 >= 0 && beta1 <= PI / 4.0f) ? beta1 : beta2;         // 选择合适的解（角度在[0, π/4]范围内）
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

void leg_control(float X_left, float X_right, float Y_left, float Y_right)
{
  // 输入限幅 
  if(Y_left<10)Y_left=10;
  if(Y_right<10)Y_right=10;

  if(Y_left>130)Y_left=130;
  if(Y_right>130)Y_right=130;
     
  // 目标位置
  float target_X_left = X_left + X_OFFSET;
  float target_Y_left = Y_left + Y_OFFSET;
  float target_X_right = X_right + X_OFFSET;
  float target_Y_right = Y_right + Y_OFFSET;
  
 // 左腿PID
  pid_pos_calc(&leg_x_left_pid, target_X_left, X_L);
  pid_pos_calc(&leg_y_left_pid, target_Y_left, Y_L);

  // 右腿PID
  pid_pos_calc(&leg_x_right_pid, target_X_right, X_R);
  pid_pos_calc(&leg_y_right_pid, target_Y_right, Y_R);

  // 更新目标位置
  XLeft = X_L + leg_x_left_pid.output;
  YLeft = Y_L + leg_y_left_pid.output;
  XRight = X_R + leg_x_right_pid.output;
  YRight = Y_R + leg_y_right_pid.output;
  
  // 更新当前位置
  X_L = XLeft;
  Y_L = YLeft;
  X_R = XRight;
  Y_R = YRight;
    
  // 左侧逆运动学求解
  solve_inverse_kinematics(XLeft, YLeft, &aLeft, &bLeft, &cLeft, &dLeft, &eLeft, &fLeft, &alphaLeft, &betaLeft);
    
  // 右侧逆运动学求解 
  solve_inverse_kinematics(XRight, YRight,&aRight, &bRight, &cRight, &dRight, &eRight, &fRight, &alphaRight, &betaRight);
    
  // 计算舵机角度
  calculate_servo_angle(alphaLeft, betaLeft, &servoLeftFront, &servoLeftRear);
  calculate_servo_angle(alphaRight, betaRight, &servoRightFront, &servoRightRear);
    
  // 设置舵机角度
  Set_angle(servoLeftFront, servoLeftRear, servoRightFront, servoRightRear);

}
