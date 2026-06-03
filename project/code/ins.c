#include "zf_common_headfile.h"
#define PIT_CH0_PRIORITY

float X_remember[FLASH_PAGE_LENGTH * Use_page] = {0};
float Y_remember[FLASH_PAGE_LENGTH * Use_page] = {0};
float Yaw_remember[FLASH_PAGE_LENGTH * Use_page] = {0};

float X_load[FLASH_PAGE_LENGTH * Use_page] = {0}; //不×6，先调试
float Y_load[FLASH_PAGE_LENGTH * Use_page] = {0};
float Yaw_load[FLASH_PAGE_LENGTH * Use_page] = {0};
INS_t ins;
uint8 road_memery_flag = 1; // 路径记忆标志位 0为初始状态 1为记录开始 2为记录完成  
uint16_t road_destination = 0;        // 记录路径的终点
uint16 num_index = 0;
// ------------------ 初始化 ------------------
int path_index= 0;
float x=0.0f,y=0.0f;
float x_now = 0.0f, y_now = 0.0f, yaw_now = 0.0f;
float target_x, target_y, target_yaw;
float dx,dy,dyaw;
float vx= 0.0f,vy=0.0f;
float distance, target_v;
float yaw_error;
int target_speed;
float k_yaw = 3.0f;
float dt = 0.020;  // ins调用周期（s）
uint16_t dot_num = 0;
void ins_init(void)
{
  road_memery_flag = 1;
}


// ----------------- 更新数据 -----------------
void ins_update(float yaw, float v_enc)
{
//  yaw = round(yaw * 100.0f) / 100.0f;
  
    // 确保不会越界访问数组
    if (remote_left_01_now_flag !=2 && (num_index >= FLASH_PAGE_LENGTH * 2 - 2 || flash_yaw_flag == 1))
    {
        road_memery_flag = 2; // 路径记忆完成标志位
        
        return;                           // 直接返回，不再记录新的点
    }
    
    // 编码器速度投影到世界坐标系
    yaw = yaw * PI / 180;
    Yaw_remember[num_index] = yaw;  // 姿态已滤波（弧度制）
    vx = v_enc * cosf(yaw);
    vy = v_enc * sinf(yaw);
    x += vx * dt;
    y += vy * dt;
    X_remember[num_index] =x;
    Y_remember[num_index] =y; 
    num_index ++;
}


void Track_update(void)
{
    // 当前位置
    x_now = x;
    y_now = y;

    //--------------------------------------------------
    // 终点保护
    //--------------------------------------------------
    if(path_index >= road_destination)
    {
        path_index = road_destination - 1;

        target_speed = 0;

        remote_right_01_now_flag = 2;

        return;
    }

    //--------------------------------------------------
    // 当前目标点
    //--------------------------------------------------
    target_x = X_load[path_index];
    target_y = Y_load[path_index];

    //--------------------------------------------------
    // 计算距离
    //--------------------------------------------------
    float dx = target_x - x_now;
    float dy = target_y - y_now;

    distance = sqrtf(dx * dx + dy * dy);

    //--------------------------------------------------
    // 目标角度
    //--------------------------------------------------
    target_yaw = atan2f(dy, dx) * 180.0f / PI;

    while(target_yaw > 180.0f)
        target_yaw -= 360.0f;

    while(target_yaw < -180.0f)
        target_yaw += 360.0f;

    //--------------------------------------------------
    // 计算航向误差
    //--------------------------------------------------
    yaw_error = target_yaw - yaw_angle;

    while(yaw_error > 180.0f)
        yaw_error -= 360.0f;

    while(yaw_error < -180.0f)
        yaw_error += 360.0f;

    //--------------------------------------------------
    // 距离控制
    //--------------------------------------------------
    target_v = KP_DIS * distance;

    //--------------------------------------------------
    // 后退逻辑
    //--------------------------------------------------
    if(fabsf(yaw_error) > 90.0f)
    {
        // 改为倒车

        target_v = -target_v;

        // 给角度环一个最小转角
        if(yaw_error > 0)
            target_yaw -= 180.0f;
        else
            target_yaw += 180.0f;

        // 重新归一化
        while(target_yaw > 180.0f)
            target_yaw -= 360.0f;

        while(target_yaw < -180.0f)
            target_yaw += 360.0f;
    }

    //--------------------------------------------------
    // 转rpm
    //--------------------------------------------------
    target_speed = truetorpm(target_v);

    //--------------------------------------------------
    // 限幅
    //--------------------------------------------------
    if(target_speed > MAX_SPEED)
        target_speed = MAX_SPEED;

    if(target_speed < -MAX_SPEED)
        target_speed = -MAX_SPEED;

    //--------------------------------------------------
    // 到点判断
    //--------------------------------------------------
    if(distance < DIST_TH)
    {
        if(path_index < road_destination - 1)
        {
            path_index++;
        }
    }
}
 