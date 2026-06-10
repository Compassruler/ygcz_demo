#include "zf_common_headfile.h"
#define PIT_CH0_PRIORITY
#define LOOK_AHEAD_DISTANCE 0.15f   // 前视距离m
#define NEAREST_SELECT_NUM 100       // 搜索最近点范围
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
float x=0.0f,y=0.0f,yaw = 0.0f;
float x_now = 0.0f, y_now = 0.0f, yaw_now = 0.0f;
float target_x, target_y, target_yaw;
float dx,dy,dyaw;
float vx= 0.0f,vy=0.0f;
float distance, target_v;
float yaw_error;
int target_speed;
float k_yaw = 3.0f;
float dt = 0.020;  // ins调用周期（s）

void ins_init(void)
{
  road_memery_flag = 1;
}


// ----------------- 更新数据 -----------------
void ins_update(void)
{
//  yaw = round(yaw * 100.0f) / 100.0f;
     
    // 确保不会越界访问数组
    if (remote_left_01_now_flag !=2 && (num_index >= FLASH_PAGE_LENGTH * 2 - 2 || flash_yaw_flag == 1))
    {
        road_memery_flag = 2; // 路径记忆完成标志位
        
        return;                           // 直接返回，不再记录新的点
    }
    
    // 编码器速度投影到世界坐标系
    yaw = yaw_angle * (PI / 180);
    Yaw_remember[num_index] = yaw;  // 姿态已滤波（弧度制）
    
    vx = true_speed * cosf(yaw);
    vy = true_speed * sinf(yaw);
    x += vx * dt;
    y += vy * dt;
    X_remember[num_index] =x;
    Y_remember[num_index] =y; 
    num_index ++;
    
}

// ----------------- 找到最近点 -----------------
int find_nearest_point(int start_index)
{
    float min_dist = 9999.0f;
    int nearest_index = start_index;
    for(int i = start_index; i < road_destination; i++)
    {
        dx = X_load[i] - x_now;
        dy = Y_load[i] - y_now;
        float dist = sqrtf(dx*dx + dy*dy);
        if(dist < min_dist)
        {
            min_dist = dist;
            nearest_index = i;
        }

        // 限制搜索范围，防止耗时
        if(i - start_index > NEAREST_SELECT_NUM)  
            break;
    }
    return nearest_index;
}

//----------------- 找到前视点 -----------------
void find_lookahead_point(int nearest_index)
{
    for(int i = nearest_index; i < road_destination; i++)
    {
        dx = X_load[i] - x_now;
        dy = Y_load[i] - y_now;

        distance = sqrtf(dx*dx + dy*dy);

        if(distance >= LOOK_AHEAD_DISTANCE)
        {
            target_x = X_load[i];
            target_y = Y_load[i];
            path_index = i;
            return;
        }
    }
    //没有前视点用最后一个点
    target_x = X_load[road_destination - 1];
    target_y = Y_load[road_destination - 1];
    path_index = road_destination - 1;
}

//----------------- 路径回放 ----------------- 
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
      
    
    find_lookahead_point(find_nearest_point(path_index));

    //--------------------------------------------------
    // 计算距离
    //--------------------------------------------------
    dx = target_x - x_now;
    dy = target_y - y_now;

    distance = sqrtf(dx*dx + dy*dy);

    //--------------------------------------------------
    // 目标角度
    //--------------------------------------------------
    target_yaw =  atan2f(dy, dx) * (180.0f / PI);
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
        float yaw_error_rad = yaw_error * PI / 180.0f;
        target_v = target_v * cosf(yaw_error_rad);

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

     // 到终点判断
    if(path_index >= road_destination - 2)
    {
        float dx_end = X_load[road_destination - 1] - x_now;
        float dy_end = Y_load[road_destination - 1] - y_now;
        if(sqrtf(dx_end*dx_end + dy_end*dy_end) < 0.05f)
        {
            target_speed = 0;
            remote_right_01_now_flag = 2;
        }
    }     
}
 