#include "zf_common_headfile.h"

float car_speed;
float true_speed;

int auto_protect_flag = 0;
int manual_protect_flag = 0;
int roll_control_flag = 0;


float target_yaw_remote = 0;
float target_roll = 0;  //‘› ±√ª”√



uint8 remote_left_01_last_flag = 0;
uint8 remote_right_01_last_flag = 0;
uint8 remote_left_01_now_flag = 0;
uint8 remote_right_01_now_flag = 0;
int flash_task_flag = 0;
bool pause_flag = true;

int vision_detect_mode = 0;
int32 vision_target_speed = 0;
int32 vision_target_yaw = 0;

float KP_DIS = 10.0f;
int MAX_SPEED = 3000;

int MIN_SPEED = 0;
float YAW_TH = 1.0f; 

bool track_flag = false;