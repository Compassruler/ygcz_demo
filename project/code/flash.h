#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#include "zf_common_headfile.h"

void flash_road_memery_store(void); // 存路径数据
void flash_road_memery_get(void);   // 取路径数据

void flash_road_memery_store_Plus(void); // 存xy数据
void flash_road_memery_get_Plus(void);  // 取xy数据
void flash_road_memory_clear(void);     // 清除flash
extern uint8_t flash_yaw_flag;      // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
extern uint8_t flash_xy_flag; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；


#define FLASH_YAW_DATA_LENGTH  (FLASH_PAGE_LENGTH - 1)          // 每一页最后一个存路径点数
#define FLASH_XY_DATA_LENGTH   (FLASH_PAGE_LENGTH - 1)          // 每一页最后一个点位空出来，与yaw对应

#define FLASH_SECTION_INDEX (0)     // 存储数据用的扇区
// yaw角存点页
#define Yaw_memery_page_INDEX_11 (11)     
#define Yaw_memery_page_INDEX_12 (12)   
#define Yaw_memery_page_INDEX_13 (13)   
#define Yaw_memery_page_INDEX_14 (14)   
#define Yaw_memery_page_INDEX_15 (15)   
//#define Yaw_memery_page_INDEX_18 (18)   

// x存点页
#define X_memery_page_INDEX_1 (1) 
#define X_memery_page_INDEX_2 (2) 
#define X_memery_page_INDEX_3 (3) 
#define X_memery_page_INDEX_4 (4)   
#define X_memery_page_INDEX_5 (5)   
//#define X_memery_page_INDEX_13 (13) 

// y存点页
#define Y_memery_page_INDEX_6 (6) 
#define Y_memery_page_INDEX_7 (7) 
#define Y_memery_page_INDEX_8 (8) 
#define Y_memery_page_INDEX_9 (9)   
#define Y_memery_page_INDEX_10 (10) 
//#define Y_memery_page_INDEX_12 (11) 



#endif /* CODE_FLASH_H_ */
