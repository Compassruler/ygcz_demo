#ifndef _FLAG_VALUE_H_
#define _FLAG_VALUE_H_

extern float car_speed;         // 车速（原始值）
extern float true_speed;        // 车速（实际值m/s）

extern int auto_protect_flag;
extern int manual_protect_flag;

extern uint8 remote_left_01_last_flag;
extern uint8 remote_right_01_last_flag;
extern uint8 remote_left_01_now_flag;
extern uint8 remote_right_01_now_flag;

extern float target_yaw; // 目标航向角 （遥控用）


#endif