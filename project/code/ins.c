#include "zf_common_headfile.h"
#define PIT_CH0_PRIORITY
#define LOOK_AHEAD_DISTANCE 0.15f   // 前视距离m
#define NEAREST_SELECT_NUM 10       // 搜索最近点范围
#define DISTANCE_STEP 0.01f  // 打点间距，单位 m（2cm）
#define MAX_ELEMENT_NUM 10              // 打断点数量
#define INTERRUPT_DISTANCE 0.5         //打断惯导的判断距离
#define RECOVER_DISTANCE 0.05           // 恢复惯导的判断距离
uint16_t element_index[MAX_ELEMENT_NUM];        // 打断点索引
uint8 element_num = 0;     // 已经记录的打断点数量
uint8 current_element = 0;  // 目前到哪个打断点了 
//#define TURN_NUM        3    //
float x_last = 0.0f;
float y_last = 0.0f;

float X_remember[FLASH_PAGE_LENGTH * Use_page] = {0};   // 记录点数组
float Y_remember[FLASH_PAGE_LENGTH * Use_page] = {0};
float Yaw_remember[FLASH_PAGE_LENGTH * Use_page] = {0};


float X_load[FLASH_PAGE_LENGTH * Use_page] = {0}; // 读取点数组
float Y_load[FLASH_PAGE_LENGTH * Use_page] = {0};
float Yaw_load[FLASH_PAGE_LENGTH * Use_page] = {0};

uint8 road_memery_flag = 1; // 路径记忆标志位 0为初始状态 1为记录开始 2为记录完成  
uint16_t road_destination = 0;        // 记录路径的终点
uint16 num_index = 0;
// ------------------ 初始化 ------------------
int path_index= 0;
float x=0.0f,y=0.0f,yaw_ins = 0.0f;
float x_ins = 0.0f,y_ins = 0.0f;
float dx_ins, dy_ins;
float distance_ins;
float x_now = 0.0f, y_now = 0.0f, yaw_now = 0.0f;
float target_x, target_y, target_yaw;
bool yaw_ready = true;
float dx,dy,dyaw;
float vx= 0.0f,vy=0.0f;
float distance, target_v;
float yaw_error;
int target_speed;

float dt = 0.020;  // ins调用周期（s） 


float distance_recover = 0;   // 判断恢复点的距离(写在这里为了调试)
void ins_init(void)
{
  road_memery_flag = 1;
}


// ----------------- 更新数据 -----------------
void ins_update(void)
{
//  yaw = round(yaw * 100.0f) / 100.0f;
     
    // 确保不会越界访问数组
    if (remote_left_01_now_flag !=2 && (num_index >= FLASH_PAGE_LENGTH * 4 - 2 || flash_yaw_flag == 1))
    {
        road_memery_flag = 2; // 路径记忆完成标志位
        return;                           // 直接返回，不再记录新的点
    }
    
    // 编码器速度投影到世界坐标系
    yaw_ins = yaw_angle * (PI / 180);
    
    vx = true_speed * cosf(yaw_ins);
    vy = true_speed * sinf(yaw_ins);
    x += vx * dt;
    y += vy * dt;
        // 元素通过检测
    if(!pause_flag)
    {
        element_recover_check();
    }
    // 计算和上一个记录点的距离
     dx_ins = x - x_last;
     dy_ins = y - y_last;
     distance_ins = sqrtf(dx_ins * dx_ins + dy_ins * dy_ins);
    if (distance_ins >= DISTANCE_STEP && pause_flag && remote_right_01_now_flag != 2)
    {
        // 超过打点间距，记录点
        X_remember[num_index] = x;
        Y_remember[num_index] = y;
        Yaw_remember[num_index] = yaw_angle;

        // 更新上一个记录点
        x_last = x;
        y_last = y;

        num_index++;
    }
    
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
        if(dist < min_dist && yaw_ready)
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
    
    // 判断是否该打断点了
    if(current_element < element_num)
{ 
    if(path_index >= element_index[current_element]-10) // 减几就是提前几个点关闭
    {
        pause_flag = false;
    }
}
    //--------------------------------------------------
    // 计算距离
    //--------------------------------------------------
    dx = target_x - x_now;
    dy = target_y - y_now;

    distance = sqrtf(dx * dx + dy * dy);

    //--------------------------------------------------
    // 目标角度：直接使用记录的连续 yaw
    //--------------------------------------------------
    float angle = Yaw_load[path_index];
//    target_yaw = Yaw_load[path_index];  // 直接使用记录好的连续角度
//    banlance.yaw_angle_pid.K = (remote_right_01_now_flag == 1) ? 0.05f : 1.0f;
    //--------------------------------------------------
    // 计算航向误差（连续角度，Yaw_load 已连续累积）
    //--------------------------------------------------
    
    if(fabs(yaw_angle - angle) >= 30)
    {
      target_yaw = yaw_angle + 30;
       target_speed = 0;
       return;
    }
    target_yaw = angle;
    yaw_error = target_yaw - yaw_angle;  
      //--------------------------------------------------
    // 距离控制
    //--------------------------------------------------
    target_v = KP_DIS * distance;

    //--------------------------------------------------
    // 后退逻辑
    //--------------------------------------------------
//    if(fabsf(yaw_error) > 90.0f)
//    {
        // 改为倒车
        float yaw_error_rad = yaw_error * PI / 180.0f;
        target_v = target_v * cosf(yaw_error_rad);
//    }

    //--------------------------------------------------
    // 转 rpm
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
    // 到终点判断
    //--------------------------------------------------
    if(path_index >= road_destination - 2)
    { 

            target_speed = 0;
            remote_right_01_now_flag = 2;
            banlance.yaw_angle_pid.K = 1.0;
            road_memery_flag = 2;
            target_yaw_remote = target_yaw;
            buzzer_beep(3,100);
    }
}
 
//  检测是否该打断惯导了（查打断点）
void path_element_check(void)
{
    element_num = 0;     // 已经记录的打断点数量

    for(int i=0;i<road_destination-1;i++)
    {
        float dx_check = X_load[i+1]-X_load[i];
        float dy_check = Y_load[i+1]-Y_load[i];

        float dis_check = sqrtf(dx_check*dx_check + dy_check*dy_check);


        // 正常2cm
        // 元素停止打点产生大间隔
        if(dis_check > INTERRUPT_DISTANCE)
        {
            if(element_num < MAX_ELEMENT_NUM)
            {
                element_index[element_num]=i;
                element_num++;
                if ( element_num ==  MAX_ELEMENT_NUM)
                  return;
            }
        }
    }
}

//  检测是否该恢复惯导了
void element_recover_check(void)
{
    if(current_element >= element_num)
        return;


    uint16_t recover_index =
        element_index[current_element] + 1;


    float dx_recover =
        X_load[recover_index] - x;

    float dy_recover =
        Y_load[recover_index] - y;


     distance_recover =
        sqrtf(dx_recover*dx_recover +
              dy_recover*dy_recover);


    if(distance_recover < RECOVER_DISTANCE)
    {
        path_index = recover_index;


        current_element++;


        pause_flag = true;
    }
}
 