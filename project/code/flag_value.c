#include "zf_common_headfile.h"
float car_speed;
float true_speed;



int auto_protect_flag = 0;
int manual_protect_flag = 0;

uint8 remote_left_01_last_flag = 0;
uint8 remote_right_01_last_flag = 0;
uint8 remote_left_01_now_flag = 0;
uint8 remote_right_01_now_flag = 0;

float target_yaw = 0;