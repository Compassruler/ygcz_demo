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
extern int yaw_lock_ctrl;
// 遥控器标志位
extern uint8 remote_left_01_last_flag;
extern uint8 remote_right_01_last_flag;
extern uint8 remote_left_01_now_flag;
extern uint8 remote_right_01_now_flag;
extern uint8 remote_right_02_last_flag;           // 右二档上次标志位
extern uint8 remote_right_02_now_flag;            // 右二档现在标志位



extern int flash_task_flag;
extern bool pause_flag;            // 惯导中断flag

extern volatile int vision_detect_mode;     // 视觉识别类型 | 0 空状态 | 1 单边桥与颠簸路段 | 2 跳跃 |
extern volatile int32 vision_target_speed;  // 视觉速度
extern volatile int32 vision_target_yaw;    // 视觉航向角

extern volatile uint8 vision_bump_start;    // 核1确认离桥后置1，激活核0颠簸路段距离积分
extern volatile uint8 vision_bump_finish;   // 核0距离积分到位后置1，通知核1完成视觉阶段

extern float KP_DIS;

extern int MAX_SPEED;
extern int MIN_SPEED;
extern float target_yaw_remote; // 目标航向角 （遥控用）
extern float target_roll;       // 目标横滚角 
extern float YAW_TH;

extern int course_record_flag;    // 记录标志位(由遥控器控制，最右边的拨钮，上为科目一（0），中为科目二（1），下为科目三（2）)
extern int course_load_flag;   // 回放标志位(按钮控制)
extern bool track_flag; // 跟踪flag true为开启跟踪
#endif