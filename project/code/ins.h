#ifndef _INS_H_
#define _INS_H_

#include "zf_common_headfile.h"
#include "zf_driver_flash.h"
#define MAX_PATH_POINTS 2000   // 最大记录点数，可根据 MCU 内存调整




#define Use_page         (4) // 使用flash的页数


extern float X_remember[FLASH_PAGE_LENGTH * Use_page];
extern float Y_remember[FLASH_PAGE_LENGTH * Use_page];
extern float Yaw_remember[FLASH_PAGE_LENGTH * Use_page];

extern float X_load[FLASH_PAGE_LENGTH * Use_page];
extern float Y_load[FLASH_PAGE_LENGTH * Use_page];
extern float Yaw_load[FLASH_PAGE_LENGTH * Use_page];

extern uint8 road_memery_flag;   // 路径记忆完成标志位
extern  uint16 num_index;
extern  uint16 safe_index;
extern uint16_t road_destination;
extern int target_speed;
extern float target_yaw;
extern int path_index;
extern float x;
extern float y;
extern float yaw;
extern float yaw_error;

extern float vx;
extern float vy;

extern float distance;
extern float x_now;
extern float y_now;
extern float target_x;
extern float target_y;
extern float target_v;
extern float dt;  // ins调用周期（s）

extern uint16_t element_index[];
extern float distance_recover;
extern bool pause_flag;
// 初始化
void ins_init(void);
void ins_enable(bool on_off);
void ins_clear(void);

// dt 秒，yaw 已滤波，v_enc 编码器线速度 m/s
void ins_update(void);

// 轨迹复现更新
void Track_update(void);

// 检测打断点
void path_element_check(void);

// 检测是否该恢复了
void element_recover_check(void);
#endif