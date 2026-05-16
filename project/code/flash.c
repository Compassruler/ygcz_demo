#include "small_driver_uart_control.h"

// 结束记录标志
bool record_done_flag = false;
int last_flash_page = 0;  // 记录最后写入的 Flash 页号

// 初始化 Flash
void flash_ins_init(void)
{
    flash_init();        // 官方库初始化
    flash_buffer_clear(); 
}

// 写入元信息页（记录上次写入的页数）
void flash_write_meta(uint16 last_page, uint16 point_count)
{
    flash_buffer_clear();

    flash_union_buffer[0].uint16_type = last_page;
    flash_union_buffer[1].uint16_type = point_count;

    // 写入元信息页
    flash_write_page_from_buffer(0, PATH_FLASH_PAGE - 1, 2); // 用2个联合体存两个uint16
}

// 写入 Flash（直接写入 path_index 指定的轨迹点数）
void flash_ins_save(void)
{
    if(path_index == 0) return;  // 没有数据就不写

    flash_buffer_clear();

    for(int i = 0; i < path_index; i++)
    {
        flash_union_buffer[i*POINT_UNION_COUNT + 0].float_type = path[i].x;
        flash_union_buffer[i*POINT_UNION_COUNT + 1].float_type = path[i].y;
        flash_union_buffer[i*POINT_UNION_COUNT + 2].float_type = path[i].yaw;
    }

    // 写入 Flash 页
    flash_write_page_from_buffer(0, PATH_FLASH_PAGE, path_index * POINT_UNION_COUNT);

    last_flash_page = PATH_FLASH_PAGE;  // 记录最后写入的页
    path_index = 0;                     // 清空 path，准备下一次记录
}

// 从 Flash 读取轨迹到 path[]
void flash_ins_load(void)
{
    flash_buffer_clear();

    // 读取写入的页（假设上一次写入的页是 last_flash_page）
    flash_read_page_to_buffer(0, last_flash_page, path_index * POINT_UNION_COUNT);

    // 复制到 path[]
    for(int i = 0; i < path_index; i++)
    {
        path[i].x   = flash_union_buffer[i*POINT_UNION_COUNT + 0].float_type;
        path[i].y   = flash_union_buffer[i*POINT_UNION_COUNT + 1].float_type;
        path[i].yaw = flash_union_buffer[i*POINT_UNION_COUNT + 2].float_type;
    }
}

//读取元信息页
FlashMeta_t flash_read_meta(void)
{
    FlashMeta_t meta = {0, 0};

    flash_read_page_to_buffer(0, PATH_FLASH_PAGE - 1, 2);

    meta.last_flash_page = flash_union_buffer[0].uint16_type;
    meta.last_point_count = flash_union_buffer[1].uint16_type;

    flash_buffer_clear();
    return meta;
}
// 擦除 Flash 中轨迹数据（只擦除已写入页）
void flash_ins_clear(void)
{
    FlashMeta_t meta = flash_read_meta();

    uint16 pages_to_erase = (meta.last_point_count + BATCH_POINTS - 1) / BATCH_POINTS;

    for(uint16 i = 0; i < pages_to_erase; i++)
    {
        flash_erase_page(0, meta.last_flash_page + i);
    }

    // 擦除完后，把元信息页清空
    flash_erase_page(0, PATH_FLASH_PAGE - 1);
}