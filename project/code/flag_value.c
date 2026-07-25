#include "zf_common_headfile.h"

float car_speed;
float true_speed;

int auto_protect_flag = 0;
int manual_protect_flag = 0;
int roll_control_flag = 0;


float target_yaw_remote = 0;
float target_roll = 0;  //暂时没用



uint8 remote_left_01_last_flag = 0;            // 左三档上次遥控器标志位
uint8 remote_right_01_last_flag = 0;           // 右三档上次遥控器标志位
uint8 remote_left_01_now_flag = 0;             // 左三档现在标志位
uint8 remote_right_01_now_flag = 0;            // 右三档现在标志位
uint8 remote_right_02_last_flag = 0;           // 右二档上次标志位
uint8 remote_right_02_now_flag = 0;            // 右二档现在标志位

int flash_task_flag = 0;
bool pause_flag = true;           // 中断回放标志位（仅用于科目三）右边二档拨钮控制，检测跳变

int vision_detect_mode = 0;
int32 vision_target_speed = 0;
int32 vision_target_yaw = 0;

float KP_DIS = 10.0f;
int MAX_SPEED = 3000;

int MIN_SPEED = 0;
float YAW_TH = 1.0f; 

bool track_flag = false;  // 回放标志位（按钮四控制）true为开启回放
int course_record_flag;          // 记录标志位(由遥控器控制，最右边的拨钮，上为科目一（0），中为科目二（1），下为科目三（2）)
int course_load_flag = 0;          //  回放标志位(按钮控制)