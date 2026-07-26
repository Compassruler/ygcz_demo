#ifndef _FLASH_H_
#define _FLASH_H_

#include "zf_common_headfile.h"

typedef struct
{
    uint32_t check;        //校验位

    uint32_t point_num;    //这一段点数量

}SegmentHeader;        // 每一段头信息结构体


typedef struct
{
    float x;
    float y;
    float yaw;

}PathPoint;             //存点结构体

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


//======================
// 头信息校验位
//======================

#define COURSE1_CHECK_VALUE    0x1111

#define COURSE2_CHECK_VALUE    0x2222

#define COURSE3_CHECK_VALUE    0x3333

#define PATH_SEGMENT_VALUE     0xAAAA                               // 段头信息



#define FLASH_SECTION_INDEX (0)     // 存储数据用的扇区
//======================
// 科目1 Flash区域
//======================

#define COURSE1_HEADER_PAGE       1                     // 写入flash头信息

#define COURSE1_POINT_START_PAGE  2                     // 写入flash路径点起始页
#define COURSE1_POINT_END_PAGE    5                     // 写入flash路径点结束页

//======================
// 科目2 Flash区域
//======================

#define COURSE2_HEADER_PAGE       11                    // 写入flash头信息

#define COURSE2_POINT_START_PAGE  12                    // 写入flash路径点起始页
#define COURSE2_POINT_END_PAGE    20                    // 写入flash路径点结束页

//======================
// 科目3 Flash区域
//======================

#define COURSE3_HEADER_PAGE          26                    // 写入flash总头信息

#define COURSE3_SEGMENT_HEADER_PAGE  27                    // 写入flash段头信息

#define COURSE3_POINT_START_PAGE     28                    // 写入flash路径点起始页

#define COURSE3_POINT_END_PAGE       80                    // 写入flash路径点结束页


//======================
// 擦除信息区
//======================
#define COURSE1_ERASE_START_PAGE  COURSE1_HEADER_PAGE
#define COURSE1_ERASE_END_PAGE    COURSE1_POINT_END_PAGE

#define COURSE2_ERASE_START_PAGE  COURSE2_HEADER_PAGE
#define COURSE2_ERASE_END_PAGE    COURSE2_POINT_END_PAGE

#define COURSE3_ERASE_START_PAGE  COURSE3_HEADER_PAGE
#define COURSE3_ERASE_END_PAGE    COURSE3_POINT_END_PAGE




#endif /* CODE_FLASH_H_ */
