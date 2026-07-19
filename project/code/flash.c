#include "zf_common_headfile.h"



uint8_t flash_yaw_flag = 0;      // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
uint8_t flash_xy_flag = 0; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
uint8_t flash_turn_flag = 0; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志
int path_write_page = PATH_START_PAGE; //写路径点的页数
//// 存路径数据
//void flash_road_memery_store(void)
//{
//    uint32 page_offset = 0;
//
//
//    //================ page11 =================
//    flash_buffer_clear();
//
//    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
//    }
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Yaw_memery_page_INDEX_11,
//        FLASH_YAW_DATA_LENGTH
//    );
//
//    page_offset += FLASH_YAW_DATA_LENGTH;
//
//
//
//    //================ page12 =================
//    flash_buffer_clear();
//
//    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
//    }
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Yaw_memery_page_INDEX_12,
//        FLASH_YAW_DATA_LENGTH
//    );
//
//    page_offset += FLASH_YAW_DATA_LENGTH;
//
//
//
//    //================ page13 =================
//    flash_buffer_clear();
//
//    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
//    }
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Yaw_memery_page_INDEX_13,
//        FLASH_YAW_DATA_LENGTH
//    );
//
//    page_offset += FLASH_YAW_DATA_LENGTH;
//
//
//
//    //================ page14 =================
//    flash_buffer_clear();
//
//    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
//    }
//
//
////    // 最后一个位置存路径长度
////    flash_union_buffer[FLASH_PAGE_LENGTH-1].uint16_type = record_total_index;
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Yaw_memery_page_INDEX_14,
//        FLASH_PAGE_LENGTH
//    );
//
//
//    flash_yaw_flag = 3;
//}
//// 取路径数据
//void flash_road_memery_get(void)
//{
//    uint32 page_offset = 0;
//
//
//    //================ page11 =================
//    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_11))
//    {
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Yaw_memery_page_INDEX_11,
//            FLASH_YAW_DATA_LENGTH
//        );
//
//
//        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//        {
//            Yaw_load[page_offset + i] =
//                flash_union_buffer[i].float_type;
//        }
//
//        page_offset += FLASH_YAW_DATA_LENGTH;
//    }
//
//
//
//    //================ page12 =================
//    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_12))
//    {
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Yaw_memery_page_INDEX_12,
//            FLASH_YAW_DATA_LENGTH
//        );
//
//
//        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//        {
//            Yaw_load[page_offset + i] =
//                flash_union_buffer[i].float_type;
//        }
//
//        page_offset += FLASH_YAW_DATA_LENGTH;
//    }
//
//
//
//    //================ page13 =================
//    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_13))
//    {
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Yaw_memery_page_INDEX_13,
//            FLASH_YAW_DATA_LENGTH
//        );
//
//
//        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//        {
//            Yaw_load[page_offset + i] =
//                flash_union_buffer[i].float_type;
//        }
//
//        page_offset += FLASH_YAW_DATA_LENGTH;
//    }
//
//
//
//    //================ page14 =================
//    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_14))
//    {
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Yaw_memery_page_INDEX_14,
//            FLASH_PAGE_LENGTH
//        );
//
//
//        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
//        {
//            Yaw_load[page_offset + i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        //读取最后一个数据
//        
//    }
//
//
//
//    flash_yaw_flag = 4;
//}
//
//// 存xy数据
//void flash_road_memery_store_Plus(void)
//{
//
//    uint32 page_offset = 0;
//
//
//    /***********************
//     *       X轴存储
//     ***********************/
//
//
//    //================ X page1 ================
//    flash_buffer_clear();
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            X_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        X_memery_page_INDEX_1,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    page_offset += FLASH_XY_DATA_LENGTH;
//
//
//
//    //================ X page2 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            X_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        X_memery_page_INDEX_2,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    page_offset += FLASH_XY_DATA_LENGTH;
//
//
//
//    //================ X page3 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            X_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        X_memery_page_INDEX_3,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    page_offset += FLASH_XY_DATA_LENGTH;
//
//
//
//    //================ X page4 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            X_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        X_memery_page_INDEX_4,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//
//    /***********************
//     *       Y轴存储
//     ***********************/
//
//
//    page_offset = 0;
//
//
//
//    //================ Y page6 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            Y_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Y_memery_page_INDEX_6,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    page_offset += FLASH_XY_DATA_LENGTH;
//
//
//
//    //================ Y page7 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            Y_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Y_memery_page_INDEX_7,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    page_offset += FLASH_XY_DATA_LENGTH;
//
//
//
//    //================ Y page8 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            Y_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Y_memery_page_INDEX_8,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    page_offset += FLASH_XY_DATA_LENGTH;
//
//
//
//    //================ Y page9 ================
//    flash_buffer_clear();
//
//
//    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
//    {
//        flash_union_buffer[i].float_type =
//            Y_remember[page_offset+i];
//    }
//
//
//    flash_write_page_from_buffer(
//        FLASH_SECTION_INDEX,
//        Y_memery_page_INDEX_9,
//        FLASH_XY_DATA_LENGTH
//    );
//
//
//    flash_xy_flag = 3;
//
//}
//
//
//// 读xy数据
//void flash_road_memery_get_Plus(void)
//{
//    uint32 page_offset = 0;
//
//
//    /***********************
//     *       X轴读取
//     ***********************/
//
//
//    //================ X page1 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   X_memery_page_INDEX_1))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            X_memery_page_INDEX_1,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            X_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        page_offset += FLASH_XY_DATA_LENGTH;
//    }
//
//
//
//    //================ X page2 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   X_memery_page_INDEX_2))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            X_memery_page_INDEX_2,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            X_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        page_offset += FLASH_XY_DATA_LENGTH;
//    }
//
//
//
//    //================ X page3 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   X_memery_page_INDEX_3))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            X_memery_page_INDEX_3,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            X_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        page_offset += FLASH_XY_DATA_LENGTH;
//    }
//
//
//
//    //================ X page4 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   X_memery_page_INDEX_4))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            X_memery_page_INDEX_4,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            X_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//    }
//
//
//
//
//    /***********************
//     *       Y轴读取
//     ***********************/
//
//
//    page_offset = 0;
//
//
//    //================ Y page6 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   Y_memery_page_INDEX_6))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Y_memery_page_INDEX_6,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            Y_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        page_offset += FLASH_XY_DATA_LENGTH;
//    }
//
//
//
//    //================ Y page7 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   Y_memery_page_INDEX_7))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Y_memery_page_INDEX_7,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            Y_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        page_offset += FLASH_XY_DATA_LENGTH;
//    }
//
//
//
//    //================ Y page8 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   Y_memery_page_INDEX_8))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Y_memery_page_INDEX_8,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            Y_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//
//        page_offset += FLASH_XY_DATA_LENGTH;
//    }
//
//
//
//    //================ Y page9 ================
//    if(flash_check(FLASH_SECTION_INDEX,
//                   Y_memery_page_INDEX_9))
//    {
//
//        flash_read_page_to_buffer(
//            FLASH_SECTION_INDEX,
//            Y_memery_page_INDEX_9,
//            FLASH_XY_DATA_LENGTH
//        );
//
//
//        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
//        {
//            Y_load[page_offset+i] =
//                flash_union_buffer[i].float_type;
//        }
//
//    }
//
//
//    flash_xy_flag = 4;
//
//}


/**
 * @brief 清除路径存储Flash区域
 * @note 上电初始化时调用
 *       清除 X/Y/Yaw 所使用的Flash页
 */
void flash_path_memory_clear(void)
{

    //清除总header
    flash_erase_page(
        FLASH_SECTION_INDEX,
        PATH_HEADER_PAGE
    );


    //清除segment header
    flash_erase_page(
        FLASH_SECTION_INDEX,
        PATH_SEGMENT_HEADER_PAGE
    );



    //清除路径点区域(改清除的页数)

    for(uint32 page = PATH_START_PAGE;
        page < FLASH_PAGE_NUM;
        page++)
    {

        flash_erase_page(
            FLASH_SECTION_INDEX,
            page
        );

    }

}

//flash写入总头信息
void flash_write_path_header(void)
{       
  flash_buffer_clear();
  flash_union_buffer[0].uint32_type =
        record_header.magic;


flash_union_buffer[1].uint32_type =
        record_header.segment_num;


flash_union_buffer[2].uint32_type =
        record_header.total_point_num;


flash_write_page_from_buffer(
    0,
    PATH_HEADER_PAGE,
    3
);
  
}

// flash写入段头信息
void flash_write_segment_headers(void)
{

    if(record_header.segment_num > MAX_SEGMENT_NUM)
        return;


    flash_buffer_clear();


    for(uint32_t i=0;i<record_header.segment_num;i++)
    {
        flash_union_buffer[i*2].uint32_type =
            segment_header[i].magic;

        flash_union_buffer[i*2+1].uint32_type =
            segment_header[i].point_num;
    }


    flash_write_page_from_buffer(
        0,
        PATH_SEGMENT_HEADER_PAGE,
        record_header.segment_num*2
    );

}

// flash写入路径点
void flash_write_all_points(PathPoint *path,uint32_t point_num)
{

    uint32_t point_index = 0;


    while(point_index < point_num)
    {
      
        // 越界保护
        if(path_write_page >= FLASH_PAGE_NUM)
        return;
        flash_buffer_clear();


        uint32_t write_num =
            point_num - point_index;


        if(write_num > PATH_POINT_PER_PAGE)
        {
            write_num = PATH_POINT_PER_PAGE;
        }



        for(uint32_t i=0;i<write_num;i++)
        {

            uint32_t index=i*3;


            flash_union_buffer[index].float_type =
                path[point_index+i].x;


            flash_union_buffer[index+1].float_type =
                path[point_index+i].y;


            flash_union_buffer[index+2].float_type =
                path[point_index+i].yaw;

        }



        flash_write_page_from_buffer(
            0,
            path_write_page,
            write_num*3
        );


        path_write_page++;


        point_index += write_num;

    }

}
// flash写入总
void flash_path_store(void)
{

    //路径点起始页
    path_write_page = PATH_START_PAGE;



    //1 总头
    flash_write_path_header();



    //2 段头
    flash_write_segment_headers();



    //3 点
    flash_write_all_points(
        record_path,
        record_header.total_point_num
    );

}

// flash读取总头
void flash_read_path_header(void)
{
    flash_read_page_to_buffer(
        0,
        PATH_HEADER_PAGE,
        3
    );

    record_header.magic = flash_union_buffer[0].uint32_type;

    record_header.segment_num = flash_union_buffer[1].uint32_type;

    record_header.total_point_num = flash_union_buffer[2].uint32_type;

    if(record_header.magic != PATH_MAGIC)
    {
        record_header.total_point_num = 0;
        return;
    }
}

// flash读取段头
void flash_read_segment_headers(void)
{

    if(record_header.segment_num > MAX_SEGMENT_NUM)
    {
        return;
    }


    flash_read_page_to_buffer(
        0,
        PATH_SEGMENT_HEADER_PAGE,
        record_header.segment_num * 2
    );


    for(uint32_t i=0;i<record_header.segment_num;i++)
    {

        segment_header[i].magic =
            flash_union_buffer[i*2].uint32_type;


        segment_header[i].point_num =
            flash_union_buffer[i*2+1].uint32_type;


        // 校验段头
        if(segment_header[i].magic != PATH_SEGMENT_MAGIC)
        {
            segment_header[i].point_num = 0;
        }


        // 防止异常点数
        if(segment_header[i].point_num > FLASH_PAGE_LENGTH * Use_page)
        {
            segment_header[i].point_num = 0;
        }

    }

}

// flash读取路径点
void flash_read_all_points(PathPoint *path,uint32_t point_num)
{

    uint32_t point_index = 0;


    uint32_t page = PATH_START_PAGE;


    while(point_index < point_num)
    {

        uint32_t read_num =
            point_num-point_index;


        if(read_num > PATH_POINT_PER_PAGE)
        {
            read_num = PATH_POINT_PER_PAGE;
        }


        flash_read_page_to_buffer(
            0,
            page,
            read_num*3
        );


        for(uint32_t i=0;i<read_num;i++)
        {

            uint32_t index=i*3;


            path[point_index+i].x =
                flash_union_buffer[index].float_type;


            path[point_index+i].y =
                flash_union_buffer[index+1].float_type;


            path[point_index+i].yaw =
                flash_union_buffer[index+2].float_type;


        }


        point_index += read_num;


        page++;

    }

}

// flash读取总
void flash_path_load(void)
{

    //1.读取总头
    flash_read_path_header();

    if(record_header.magic != PATH_MAGIC)
    {
        return;
    }

    //2.读取段头
    flash_read_segment_headers();

    //3.读取所有路径点
    flash_read_all_points(replay_point,record_header.total_point_num);
}

void flash_turn_memery_store()
{


}
//
//void flash_task()
//{
//    switch(flash_task_flag)
//    {
//    case FLASH_IDLE:
//      break;
//    case FLASH_STORE:  
//    flash_road_memery_store();
//    flash_road_memery_store_Plus();    
//  break;
//    case FLASH_LOAD:
//      flash_road_memery_get();
//        flash_road_memery_get_Plus();
//          break;
//}
//}