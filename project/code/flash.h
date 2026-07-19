#ifndef _FLASH_H_
#define _FLASH_H_

#include "zf_common_headfile.h"

typedef struct
{
    uint32_t magic;        //校验位

    uint32_t point_num;    //这一段点数量

}SegmentHeader;        // 每一段头信息结构体


typedef struct
{
    float x;
    float y;
    float yaw;

}PathPoint;             //存点结构体

void flash_road_memery_store(void); // 存路径数据
void flash_road_memery_get(void);   // 取路径数据

void flash_road_memery_store_Plus(void); // 存xy数据
void flash_road_memery_get_Plus(void);  // 取xy数据
void flash_road_memory_clear(void);     // 清除flash
extern uint8_t flash_yaw_flag;      // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
extern uint8_t flash_xy_flag; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；

//flash写入总头信息
void flash_write_path_header(void);

// flash写入段头信息
void flash_write_segment_header(SegmentHeader *header);

// flash写入路径点
void flash_write_all_points(PathPoint *path, uint32_t point_num); //path：开始位置 point_num：该段点数

// flash写入总
void flash_path_store(void);

// flash读取总头
void flash_read_path_header(void);

// flash读取段头
void flash_read_segment_headers(void);

// flash读取路径点
void flash_read_all_points(PathPoint *path,uint32_t point_num);

//flash读取总
void flash_path_load(void);

#define FLASH_YAW_DATA_LENGTH  (FLASH_PAGE_LENGTH - 1)          // 每一页最后一个存路径点数
#define FLASH_XY_DATA_LENGTH   (FLASH_PAGE_LENGTH - 1)          // 每一页最后一个点位空出来，与yaw对应
#define PATH_MAGIC 0x12345678                                   // 头信息校验位
#define PATH_SEGMENT_MAGIC 0x1234                               // 段头信息




#define FLASH_SECTION_INDEX (0)     // 存储数据用的扇区

#define PATH_HEADER_PAGE     (1)    // 总头信息页
#define PATH_SEGMENT_HEADER_PAGE      (2)    // 段头信息页
#define PATH_START_PAGE           (3)           // 路径点页




#endif /* CODE_FLASH_H_ */
