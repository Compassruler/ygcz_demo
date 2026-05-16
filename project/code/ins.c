#include "zf_common_headfile.h"



INS_t ins;

#define PATH_FLASH_PAGE 10    // 你要写入的 Flash 页号
#define BATCH_SIZE 100        // 每次批量写 100 个点
// ------------------ 初始化 ------------------
PathPoint_t path[MAX_PATH_POINTS];
bool record_ins = true;
int path_index = 0;

void ins_init(void)
{
    ins.enabled = false;
    ins.x = ins.y = 0;
    ins.vx = ins.vy = 0;
    ins.yaw = 0;
}

void ins_enable(bool on_off)
{
    ins.enabled = on_off;
}

void ins_clear(void)
{
    ins.x = ins.y = 0;
    ins.vx = ins.vy = 0;
    ins.yaw = 0;
}

// ----------------- 更新数据 -----------------
void ins_update(float dt, float yaw, float v_enc)
{
    if(!ins.enabled) return;
    ins.yaw = yaw;  // 姿态已滤波

    // 编码器速度投影到世界坐标系
    ins.vx = v_enc * cosf(ins.yaw);
    ins.vy = v_enc * sinf(ins.yaw);

    // 位置积分
    ins.x += ins.vx * dt;
    ins.y += ins.vy * dt;
}

void ins_get_pos(float *x, float *y, float *yaw)
{
    *x = ins.x;
    *y = ins.y;
    *yaw = ins.yaw;
}

void ins_get_vel(float *vx, float *vy)
{
    *vx = ins.vx;
    *vy = ins.vy;
}
