#ifndef _FLASH_H_
#define _FLASH_H_

#include "zf_driver_flash.h"
#include "ins.h"          // 你的 INS 结构体 PathPoint_t

#define PATH_FLASH_PAGE      10      // 起始 Flash 页
#define BATCH_POINTS         50      // 每批写入点数
#define POINT_UNION_COUNT    3       // 每点 3 个联合体存 x,y,yaw


typedef struct {
    uint16 last_flash_page;  // 上一次写入的起始页号
    uint16 last_point_count; // 上一次写入的总轨迹点数
} FlashMeta_t;

// 外部变量
extern bool record_done_flag;       // 结束记录标志
extern int last_flash_page;         // 最后写入的 Flash 页号

// 初始化 Flash
void flash_ins_init(void);

// 写入元信息页
void flash_write_meta(uint16 last_page, uint16 point_count);

// 读取元信息页
FlashMeta_t flash_read_meta(void);

// 将 INS 轨迹写入 Flash（批量写）
void flash_ins_save(void);

// 从 Flash 读取轨迹到 path[]
void flash_ins_load(void);

// 清空 Flash 中轨迹数据（擦除页）
void flash_ins_clear(void);

#endif  // _FLASH_H_