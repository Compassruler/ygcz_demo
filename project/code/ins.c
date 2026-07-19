#include "zf_common_headfile.h"
#define PIT_CH0_PRIORITY
#define LOOK_AHEAD_DISTANCE 0.15f   // 前视距离m
#define NEAREST_SELECT_NUM 4       // 搜索最近点范围
#define DISTANCE_STEP 0.01f  // 打点间距，单位 m（2cm）
#define MAX_ELEMENT_NUM 10              // 打断点数量
#define INTERRUPT_DISTANCE 0.5         //打断惯导的判断距离
#define RECOVER_DISTANCE 0.15           // 恢复惯导的判断距离
#define MAX_PATH_POINT          (FLASH_PAGE_LENGTH * Use_page)             //最大点数



uint16_t element_index[MAX_ELEMENT_NUM];        // 打断点索引

uint8 element_num = 0;     // 已经记录的打断点数量
uint8 current_element = 0;  // 目前到哪个打断点了 
//#define TURN_NUM        3    //
float x_last = 0.0f;
float y_last = 0.0f;
                                            
PathPoint record_path[FLASH_PAGE_LENGTH * Use_page];           //记录点结构体
PathPoint replay_point[FLASH_PAGE_LENGTH * Use_page];          //回放点结构体
PathHeader record_header;                                      //总头信息结构体
SegmentHeader segment_header[MAX_SEGMENT_NUM];  // 段的头信息结构体

uint32_t record_total_index;          //记录点总数/索引(总)
uint32_t current_segment_points;      // 记录点总数/索引（段） 
uint32_t segment_point_num[MAX_SEGMENT_NUM]; //记录每一段的点数
uint32_t segment_num = 0;                       //段数（表示有多少段）

uint8 road_memery_flag = 1; // 路径记忆标志位 0为初始状态 1为记录开始 2为记录完成  
uint8_t dir = 0;  // 0:前进  1:倒车(方向)
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

int pause_time = 0; // 恢复惯导时间

float dt = 0.020;  // ins调用周期（s） 

float distance_recover = 0;   // 判断恢复点的距离(写在这里为了调试)

void ins_init(void)
{
    road_memery_flag = 1;


    record_total_index = 0;

    current_segment_points = 0;

    segment_num = 0;

}

// 写入数据点到记录结构体数组
void path_record_add(float x,float y,float yaw)
{
    if(record_total_index >= MAX_PATH_POINT)
        return;


    record_path[record_total_index].x=x;

    record_path[record_total_index].y=y;

    record_path[record_total_index].yaw=yaw;


    record_total_index++;

    current_segment_points++;
}


// 每一段结束后重置
void path_segment_finish(void)
{
    if(current_segment_points == 0)
        return;


    if(segment_num >= MAX_SEGMENT_NUM)
        return;


    //生成这一段头信息
    segment_header[segment_num].magic = PATH_SEGMENT_MAGIC;

    segment_header[segment_num].point_num 
        = current_segment_points;


    segment_num++;


    //当前段清零
    current_segment_points = 0;


    //坐标重新建立
    x = 0;
    y = 0;

    x_last = 0;
    y_last = 0;


    //重新等待稳定（再看）
    pause_flag = false;
    pause_time = 0;

}

//写入头信息（校验位＋点数）
void path_record_finish(void)
{

    if(current_segment_points > 0)
    {
        path_segment_finish();
    }


    record_header.magic = PATH_MAGIC;

    record_header.segment_num = segment_num;

    record_header.total_point_num = record_total_index;

}

// ----------------- 更新数据 -----------------
void ins_update(void)
{
//  yaw = round(yaw * 100.0f) / 100.0f; 
  
    if(remote_right_01_now_flag == 2 &&remote_right_01_last_flag == 0) // 遥控打断时写入一段的头信息
    {
      path_segment_finish();
      return;
    }
    
    // 确保不会越界访问数组/
    if (remote_left_01_now_flag !=2 && (record_total_index >= FLASH_PAGE_LENGTH * Use_page || remote_left_01_now_flag == 1))
    {
        path_record_finish();   //写入头信息
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
//        element_recover_check();
      pause_time ++;
      if (pause_time > 100)
        pause_flag = true;
    }
    // 计算和上一个记录点的距离
     dx_ins = x - x_last;
     dy_ins = y - y_last;
     distance_ins = sqrtf(dx_ins * dx_ins + dy_ins * dy_ins);
    if (distance_ins >= DISTANCE_STEP && pause_flag && remote_right_01_now_flag != 2)
    {
        // 超过打点间距，记录点
        path_record_add(x,y,yaw_angle);
        // 更新上一个记录点
        x_last = x;
        y_last = y;      
    }   
}

// ----------------- 找到最近点 -----------------
int find_nearest_point(int start_index)
{
    float min_dist = 9999.0f;
    int nearest_index = start_index;
    for(int i = start_index; i < record_header.total_point_num; i++)
    {
        dx = replay_point[i].x - x_now;
        dy = replay_point[i].y - y_now;
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
    for(int i = nearest_index; i < record_header.total_point_num; i++)
    {
        dx = replay_point[i].x - x_now;
        dy = replay_point[i].y - y_now;

        distance = sqrtf(dx*dx + dy*dy);

        if(distance >= LOOK_AHEAD_DISTANCE)
        {
            target_x = replay_point[i].x;
            target_y = replay_point[i].y;
            path_index = i;
            return;
        }
    }
    //没有前视点用最后一个点
    target_x = replay_point[record_header.total_point_num-1].x;
    target_y = replay_point[record_header.total_point_num-1].y;
    path_index = record_header.total_point_num - 1;
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
    if(path_index >= record_header.total_point_num)
    {
        path_index = record_header.total_point_num - 1;

        target_speed = 0;

        remote_right_01_now_flag = 2;

        return;
    }

    find_lookahead_point(find_nearest_point(path_index));
    
    // 判断是否该打断点了
    if(current_element < element_num)
    { 
        if(path_index >= element_index[current_element]-6)
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
    
    float target_move_yaw = atan2f(dy, dx) * 180.0f / PI; //目标运动方向


    //--------------------------------------------------
    // 目标角度：直接使用记录的连续 yaw
    //--------------------------------------------------
    float angle = replay_point[path_index].yaw;


    //--------------------------------------------------
    // 航向误差
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
    // 后退逻辑（增加滞回区）
    //--------------------------------------------------

    // 将目标方向调整到当前连续yaw附近
    float move_error = target_move_yaw - yaw_angle;

    while(move_error > 180)
    {
        move_error -= 360;
    }

    while(move_error < -180)
    {
        move_error += 360;
    }


    float move_error_abs = fabs(move_error);


    // 当前前进状态
    if(dir == 0)
    {
        // 偏差超过120°才进入倒车
        if(move_error_abs > 120)
        {
            dir = 1;
        }
    }
    // 当前倒车状态
    else
    {
        // 偏差小于60°才退出倒车
        if(move_error_abs < 60)
        {
            dir = 0;
        }
    }


    // 根据状态决定速度方向
    if(dir)
    {
        target_v = -target_v;
    }


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
    if(path_index >= record_header.total_point_num - 2)
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

    for(int i=0;i<record_header.total_point_num-1;i++)
    {
        float dx_check = replay_point[i+1].x -replay_point[i].x;
        float dy_check = replay_point[i+1].y -replay_point[i].y;

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
        replay_point[recover_index].x - x;

    float dy_recover =
        replay_point[recover_index].y - y;


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
 