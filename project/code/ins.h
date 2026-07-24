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
    uint32_t magic;             //校验位

    uint32_t segment_num;      //总段数

    uint32_t total_point_num;  //所有段总点数

}PathHeader;                          // 总的头信息结构体




extern PathPoint record_path[FLASH_PAGE_LENGTH * Use_page];           //记录点结构体
extern PathPoint replay_point[FLASH_PAGE_LENGTH * Use_page];          //回放点结构体
extern SegmentHeader segment_header[MAX_SEGMENT_NUM];                   // 段头信息结构体
extern PathHeader record_header;                                        // 总头信息结构体

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



extern float turn_angle;
extern float path_yaw_change;
// 初始化
void ins_init(void);
void ins_enable(bool on_off);
void ins_clear(void);
void track_init(void);
// dt 秒，yaw 已滤波，v_enc 编码器线速度 m/s
void ins_update(void);

// 轨迹复现更新
void Track_update(void);

// 检测打断点
void segment_check(void);


// 写入数据点到记录结构体数组
void path_record_add(float x,float y,float yaw);

// 每一段结束后重置
void path_segment_finish(void);

// 写入头信息（校验位＋点数）
void path_record_finish(void);

// flash读取总头
void flash_read_path_header(void);

// flash读取段头
void flash_read_segment_headers(void);

// flash读取路径点
void flash_read_all_points(PathPoint *path,uint32_t point_num);

// 当前段结束检测函数
void segment_check(void);

// 检测角度
float get_path_turn_angle(uint32_t index);

// 未来转弯检测
uint8_t check_future_turn(uint32_t index);
#endif