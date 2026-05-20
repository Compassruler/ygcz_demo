#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#include "zf_common_headfile.h"

void flash_road_memery_store(void); // 存路径数据
void flash_road_memery_get(void);   // 取路径数据

void flash_road_memery_store_Plus(void); // 存xy数据
void flash_road_memery_get_Plus(void);  // 取xy数据

extern uint8_t flash_yaw_flag;      // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
extern uint8_t flash_xy_flag; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
#define FLASH_SECTION_INDEX (0)     // 存储数据用的扇区
#define Yaw_memery_page_INDEX_11 (11)   // 路径存点专用扇区   
#define Yaw_memery_page_INDEX_14 (14)   // 路径存点专用扇区
#define Yaw_memery_page_INDEX_15 (15)   // 路径存点专用扇区
#define Yaw_memery_page_INDEX_16 (16)   // 路径存点专用扇区
#define Yaw_memery_page_INDEX_17 (17)   // 路径存点专用扇区
#define Yaw_memery_page_INDEX_18 (18)   // 路径存点专用扇区
   
#define X_memery_page_INDEX_1 (1) // x存点专用扇区
#define X_memery_page_INDEX_3 (3) // x存点专用扇区
#define X_memery_page_INDEX_5 (5) // x存点专用扇区

#define Y_memery_page_INDEX_2 (2) // y存点专用扇区
#define Y_memery_page_INDEX_4 (4) // y存点专用扇区
#define Y_memery_page_INDEX_6 (6) // y存点专用扇区

#define X_memery_page_INDEX_7 (7)   // 新增X轴存点专用扇区
#define X_memery_page_INDEX_9 (9)   // 新增X轴存点专用扇区
#define X_memery_page_INDEX_13 (13) // 新增X轴存点专用扇区

#define Y_memery_page_INDEX_8 (8)   // 新增Y轴存点专用扇区
#define Y_memery_page_INDEX_10 (10) // 新增Y轴存点专用扇区
#define Y_memery_page_INDEX_12 (12) // 新增Y轴存点专用扇区

#endif /* CODE_FLASH_H_ */
