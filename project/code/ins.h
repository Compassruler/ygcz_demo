#ifndef _INS_H_
#define _INS_H_

#include "zf_common_headfile.h"
#include "zf_driver_flash.h"
#include "flash.h"
#define MAX_PATH_POINTS 2000   // 最大记录点数，可根据 MCU 内存调整
#define Use_page         (4) // 使用flash的页数
#define MAX_SEGMENT_NUM 20              //最大段数




typedef struct
{
    uint32_t check;             //校验位

    uint32_t segment_num;      //总段数

    uint32_t total_point_num;  //所有段总点数

}Course3Header;                          // 科目三的总头信息结构体

typedef struct
{
    uint32_t check;        // 科目标识校验

    uint32_t point_num;    // 总点数

} CourseSingleHeader;                                   // 科目12点头信息结构体


extern PathPoint record_path[FLASH_PAGE_LENGTH * Use_page];           //记录点结构体
extern PathPoint replay_point[FLASH_PAGE_LENGTH * Use_page];          //回放点结构体
extern SegmentHeader segment_header[MAX_SEGMENT_NUM];                   // 段头信息结构体
extern Course3Header mul_header;                                        // 科目三总头信息结构体
extern CourseSingleHeader single_header;



extern uint8 road_memery_flag;   // 路径记忆完成标志位
extern uint32_t record_total_index;          //记录点总数/索引(总)
extern uint32_t current_segment_points;      // 记录点总数/索引（段） 
extern uint32_t current_segment;                //当前段索引   
extern  uint16 safe_index;

extern int target_speed;
extern float target_yaw;
extern int path_index;          // 当前回放点索引
extern float x;                 // ins x坐标
extern float x_last;
extern float y_last;
extern float y;
extern float yaw;
extern float yaw_error;

extern float vx;
extern float vy;

extern float distance;
extern float x_now;             // 当前回放点坐标
extern float y_now;
extern float target_x;
extern float target_y;
extern float target_v;
extern float dt;  // ins调用周期（s）

extern uint16_t element_index[];
extern float distance_recover;
extern bool pause_flag;

extern uint32_t replay_point_num;                                     //回放点数量
extern uint32_t segment_end_index;                                    //当前段结束点索引（科目一二初始化别处赋值）

extern float turn_angle;
extern float path_yaw_change;


extern int TURN_CHECK_POINT;    // 提前减速的点
extern int TURN_SPEED_LIMIT;  // 速度条件，低于该速度不减速
extern float TURN_SPEED_SCALE;  // 降速比例
extern int TURN_ANGLE_LIMIT; //降速判断角度
extern float LOOK_AHEAD_DISTANCE; // 前视距离
// 初始化
void ins_init(void);
void track_init(void);

// 写入数据点到记录结构体数组
void path_record_add(float x,float y,float yaw); // dt 秒，yaw 已滤波，v_enc 编码器线速度 m/s

// 每一段打点结束后重置
void path_segment_finish(void);

// ins数据更新
void ins_update(void);

//写入头信息（校验位＋点数）
void path_record_finish(void);

//  找到最近点 
int find_nearest_point(int start_index);

//  找前视点
void find_lookahead_point(int nearest_index);
// 轨迹复现更新
void Track_update(void);

// 检测打断点
void segment_check(void);


// 当前段结束检测函数
void segment_check(void);

// 检测角度
float get_path_turn_angle(uint32_t index);

// 未来转弯检测
void check_future_turn(uint32_t index);

// 出弯检测
void check_turn_finish(void);
#endif