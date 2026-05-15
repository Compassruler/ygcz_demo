#ifndef _INS_H_
#define _INS_H_

#include "zf_common_headfile.h"


typedef struct {
    bool enabled;
    float x;    // 世界坐标 X
    float y;    // 世界坐标 Y
    float yaw;  // 偏航角

    float vx;   // 世界坐标速度 X
    float vy;   // 世界坐标速度 Y
} INS_t;

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