#ifndef _INS_H_
#define _INS_H_

#include "zf_common_headfile.h"
#include "zf_driver_flash.h"
#define MAX_PATH_POINTS 2000   // 最大记录点数，可根据 MCU 内存调整

typedef struct {
    bool enabled;
    float x;    // 世界坐标 X
    float y;    // 世界坐标 Y
    float yaw;  // 偏航角

    float vx;   // 世界坐标速度 X
    float vy;   // 世界坐标速度 Y
} INS_t;



#define Use_page         (2) // 使用flash的页数


extern float X_remember[FLASH_PAGE_LENGTH * Use_page];
extern float Y_remember[FLASH_PAGE_LENGTH * Use_page];
extern float Yaw_remember[FLASH_PAGE_LENGTH * Use_page];

extern float X_load[FLASH_PAGE_LENGTH * Use_page];
extern float Y_load[FLASH_PAGE_LENGTH * Use_page];
extern float Yaw_load[FLASH_PAGE_LENGTH * Use_page];

extern uint8 road_memery_flag;   // 路径记忆完成标志位
extern INS_t ins;
extern  uint16 num_index;
extern  uint16 safe_index;
extern uint16_t road_destination;
extern int target_speed;
extern float target_yaw;
extern int path_index;
extern float x;
extern float y;
extern float yaw;

extern float vx;
extern float vy;

extern float distance;
extern float x_now;
extern float y_now;
extern float target_x;
extern float target_y;
extern float dt;  // ins调用周期（s）

// 初始化
void ins_init(void);
void ins_enable(bool on_off);
void ins_clear(void);

// dt 秒，yaw 已滤波，v_enc 编码器线速度 m/s
void ins_update(void);

// 轨迹复现更新
void Track_update(void);
#endif