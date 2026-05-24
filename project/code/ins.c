#include "zf_common_headfile.h"
#define PIT_CH0_PRIORITY

float X_remenber[FLASH_PAGE_LENGTH * 6] = {0};
float Y_remenber[FLASH_PAGE_LENGTH * 6] = {0};
float Yaw_remenber[FLASH_PAGE_LENGTH * 6] = {0};

float X_load[FLASH_PAGE_LENGTH * 6] = {0};
float Y_load[FLASH_PAGE_LENGTH * 6] = {0};
float Yaw_load[FLASH_PAGE_LENGTH * 6] = {0};
INS_t ins;
uint8 road_memery_flag = 1; // 路径记忆标志位 0为初始状态 1为记录开始 2为记录完成  
uint8 road_destination = 0;        // 记录路径的终点
volatile int num_index = 0;
// ------------------ 初始化 ------------------
int path_index;
float x=0.0f,y=0.0f;
float vx,vy;
void ins_init(void)
{
  road_memery_flag = 1;
  
}


// ----------------- 更新数据 -----------------
void ins_update(float dt, float yaw, float v_enc)
{
//  yaw = round(yaw * 100.0f) / 100.0f;
  
    // 确保不会越界访问数组
    if (num_index >= FLASH_PAGE_LENGTH * 6 - 2 || flash_yaw_flag ==2)
    {
        road_memery_flag = 2; // 路径记忆完成标志位
        return;                           // 直接返回，不再记录新的点
    }
    Yaw_remenber[num_index] = yaw;  // 姿态已滤波
    // 编码器速度投影到世界坐标系
    yaw = yaw * 3.1415f /180;
    vx = v_enc * cosf(yaw);
    vy = v_enc * sinf(yaw);
    x += vx * dt;
    y += vy * dt;
    X_remenber[num_index] =x;
    Y_remenber[num_index] =y; 
    num_index ++;
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
