#include "zf_common_headfile.h"

float car_speed;
float true_speed;

int auto_protect_flag = 0;
int manual_protect_flag = 0;
int roll_control_flag = 0;

float target_yaw_remote = 0;
float target_roll = 0;  //暂时没用

uint8 remote_left_01_last_flag = 0;
uint8 remote_right_01_last_flag = 0;
uint8 remote_left_01_now_flag = 0;
uint8 remote_right_01_now_flag = 0;
int flash_task_flag = 0;

float KP_DIS = 4.0f;
int MAX_SPEED = 400;
float DIST_TH = 0.05f;
int MIN_SPEED = 0;
float YAW_TH = 1.0f; 

