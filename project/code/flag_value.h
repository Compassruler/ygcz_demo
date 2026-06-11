#ifndef _FLAG_VALUE_H_
#define _FLAG_VALUE_H_
#define FLASH_IDLE    (0);
#define FLASH_STORE   (1);
#define FLASH_LOAD    (2);

extern float car_speed;         // 车速（原始值）
extern float true_speed;        // 车速（实际值m/s）

extern int auto_protect_flag;       // 自动保护标志位
extern int manual_protect_flag;     // 手动保护标志位
extern int roll_control_flag;       // 横滚角运动模式标志位（0为运动模式，1为单边桥模式）

extern uint8 remote_left_01_last_flag;
extern uint8 remote_right_01_last_flag;
extern uint8 remote_left_01_now_flag;
extern uint8 remote_right_01_now_flag;

extern int flash_task_flag;

extern float KP_DIS;
extern float DIST_TH;
extern int MAX_SPEED;
extern int MIN_SPEED;
extern float target_yaw_remote; // 目标航向角 （遥控用）
extern float target_roll;       // 目标横滚角 
extern float YAW_TH;

#endif