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

typedef struct {
    float x[FLASH_PAGE_LENGTH * 6];
    float y[FLASH_PAGE_LENGTH * 6];
    float yaw[FLASH_PAGE_LENGTH];
} PathPoint_t;



extern float X_remenber[FLASH_PAGE_LENGTH * 6];
extern float Y_remenber[FLASH_PAGE_LENGTH * 6];
extern PathPoint_t path;
extern uint8 road_memery_finish_flag;   // 路径记忆完成标志位
extern uint8 road_memery_start_flag;    //路径记忆开始标志位
extern uint8 road_recurrent_flag;       // 路径复现标志位
extern INS_t ins;

extern volatile uint16 safe_index;

// 初始化
void ins_init(void);
void ins_enable(bool on_off);
void ins_clear(void);

// dt 秒，yaw 已滤波，v_enc 编码器线速度 m/s
void ins_update(float dt, float yaw, float v_enc);

// 获取数据
void ins_get_pos(float *x, float *y, float *yaw);
void ins_get_vel(float *vx, float *vy);

#endif