#include "zf_common_headfile.h"
#define PIT_CH0_PRIORITY

float X_remenber[FLASH_PAGE_LENGTH * 6] = {0};
float Y_remenber[FLASH_PAGE_LENGTH * 6] = {0};
INS_t ins;
uint8 road_memery_finish_flag = 0; // 路径记忆完成标志位
uint8 road_memery_start_flag = 0;  // 路径记忆开始标志位
uint8 road_recurrent_flag = 0;     // 路径复现标志位
volatile int index = 0;
// ------------------ 初始化 ------------------
PathPoint_t path = {0};
int path_index;
float x=0.0f,y=0.0f;
float vx,vy;
void ins_init(void)
{
}


// ----------------- 更新数据 -----------------
void ins_update(float dt, float yaw, float v_enc)
{
//  yaw = round(yaw * 100.0f) / 100.0f;
  
    // 确保不会越界访问数组
    if (index >= FLASH_PAGE_LENGTH * 6 - 2)
    {
        road_memery_finish_flag = 1; // 路径记忆完成标志位
        return;                           // 直接返回，不再记录新的点
    }
    path.yaw[index] = (int)yaw;  // 姿态已滤波
    // 编码器速度投影到世界坐标系
    yaw = yaw * 3.1415f /180;
    vx = v_enc * cosf(yaw);
    vy = v_enc * sinf(yaw);
    x += vx * dt;
    y += vy * dt;
    X_remenber[index] =x;
    Y_remenber[index] =y; 
    index ++;
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
