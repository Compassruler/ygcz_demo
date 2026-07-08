#include "zf_common_headfile.h"



uint8_t flash_yaw_flag = 0;      // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
uint8_t flash_xy_flag = 0; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
uint8_t flash_turn_flag = 0; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志
// 存路径数据
void flash_road_memery_store(void)
{
    uint32 page_offset = 0;


    //================ page11 =================
    flash_buffer_clear();

    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
    }

    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Yaw_memery_page_INDEX_11,
        FLASH_YAW_DATA_LENGTH
    );

    page_offset += FLASH_YAW_DATA_LENGTH;



    //================ page12 =================
    flash_buffer_clear();

    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
    }

    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Yaw_memery_page_INDEX_12,
        FLASH_YAW_DATA_LENGTH
    );

    page_offset += FLASH_YAW_DATA_LENGTH;



    //================ page13 =================
    flash_buffer_clear();

    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
    }

    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Yaw_memery_page_INDEX_13,
        FLASH_YAW_DATA_LENGTH
    );

    page_offset += FLASH_YAW_DATA_LENGTH;



    //================ page14 =================
    flash_buffer_clear();

    for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type = Yaw_remember[page_offset + i];
    }


    // 最后一个位置存路径长度
    flash_union_buffer[FLASH_PAGE_LENGTH-1].uint16_type = num_index;


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Yaw_memery_page_INDEX_14,
        FLASH_PAGE_LENGTH
    );


    flash_yaw_flag = 3;
}
// 取路径数据
void flash_road_memery_get(void)
{
    uint32 page_offset = 0;


    //================ page11 =================
    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_11))
    {
        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Yaw_memery_page_INDEX_11,
            FLASH_YAW_DATA_LENGTH
        );


        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
        {
            Yaw_load[page_offset + i] =
                flash_union_buffer[i].float_type;
        }

        page_offset += FLASH_YAW_DATA_LENGTH;
    }



    //================ page12 =================
    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_12))
    {
        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Yaw_memery_page_INDEX_12,
            FLASH_YAW_DATA_LENGTH
        );


        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
        {
            Yaw_load[page_offset + i] =
                flash_union_buffer[i].float_type;
        }

        page_offset += FLASH_YAW_DATA_LENGTH;
    }



    //================ page13 =================
    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_13))
    {
        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Yaw_memery_page_INDEX_13,
            FLASH_YAW_DATA_LENGTH
        );


        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
        {
            Yaw_load[page_offset + i] =
                flash_union_buffer[i].float_type;
        }

        page_offset += FLASH_YAW_DATA_LENGTH;
    }



    //================ page14 =================
    if (flash_check(FLASH_SECTION_INDEX, Yaw_memery_page_INDEX_14))
    {
        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Yaw_memery_page_INDEX_14,
            FLASH_PAGE_LENGTH
        );


        for(size_t i = 0; i < FLASH_YAW_DATA_LENGTH; i++)
        {
            Yaw_load[page_offset + i] =
                flash_union_buffer[i].float_type;
        }


        //读取最后一个数据
        road_destination =
            flash_union_buffer[FLASH_PAGE_LENGTH-1].uint16_type;
    }



    flash_yaw_flag = 4;
}

// 存xy数据
void flash_road_memery_store_Plus(void)
{

    uint32 page_offset = 0;


    /***********************
     *       X轴存储
     ***********************/


    //================ X page1 ================
    flash_buffer_clear();

    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            X_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        X_memery_page_INDEX_1,
        FLASH_XY_DATA_LENGTH
    );


    page_offset += FLASH_XY_DATA_LENGTH;



    //================ X page2 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            X_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        X_memery_page_INDEX_2,
        FLASH_XY_DATA_LENGTH
    );


    page_offset += FLASH_XY_DATA_LENGTH;



    //================ X page3 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            X_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        X_memery_page_INDEX_3,
        FLASH_XY_DATA_LENGTH
    );


    page_offset += FLASH_XY_DATA_LENGTH;



    //================ X page4 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            X_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        X_memery_page_INDEX_4,
        FLASH_XY_DATA_LENGTH
    );



    /***********************
     *       Y轴存储
     ***********************/


    page_offset = 0;



    //================ Y page6 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            Y_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Y_memery_page_INDEX_6,
        FLASH_XY_DATA_LENGTH
    );


    page_offset += FLASH_XY_DATA_LENGTH;



    //================ Y page7 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            Y_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Y_memery_page_INDEX_7,
        FLASH_XY_DATA_LENGTH
    );


    page_offset += FLASH_XY_DATA_LENGTH;



    //================ Y page8 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            Y_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Y_memery_page_INDEX_8,
        FLASH_XY_DATA_LENGTH
    );


    page_offset += FLASH_XY_DATA_LENGTH;



    //================ Y page9 ================
    flash_buffer_clear();


    for(size_t i = 0; i < FLASH_XY_DATA_LENGTH; i++)
    {
        flash_union_buffer[i].float_type =
            Y_remember[page_offset+i];
    }


    flash_write_page_from_buffer(
        FLASH_SECTION_INDEX,
        Y_memery_page_INDEX_9,
        FLASH_XY_DATA_LENGTH
    );


    flash_xy_flag = 3;

}


// 读xy数据
void flash_road_memery_get_Plus(void)
{
    uint32 page_offset = 0;


    /***********************
     *       X轴读取
     ***********************/


    //================ X page1 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   X_memery_page_INDEX_1))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            X_memery_page_INDEX_1,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            X_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }


        page_offset += FLASH_XY_DATA_LENGTH;
    }



    //================ X page2 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   X_memery_page_INDEX_2))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            X_memery_page_INDEX_2,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            X_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }


        page_offset += FLASH_XY_DATA_LENGTH;
    }



    //================ X page3 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   X_memery_page_INDEX_3))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            X_memery_page_INDEX_3,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            X_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }


        page_offset += FLASH_XY_DATA_LENGTH;
    }



    //================ X page4 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   X_memery_page_INDEX_4))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            X_memery_page_INDEX_4,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            X_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }
    }




    /***********************
     *       Y轴读取
     ***********************/


    page_offset = 0;


    //================ Y page6 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   Y_memery_page_INDEX_6))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Y_memery_page_INDEX_6,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            Y_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }


        page_offset += FLASH_XY_DATA_LENGTH;
    }



    //================ Y page7 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   Y_memery_page_INDEX_7))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Y_memery_page_INDEX_7,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            Y_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }


        page_offset += FLASH_XY_DATA_LENGTH;
    }



    //================ Y page8 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   Y_memery_page_INDEX_8))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Y_memery_page_INDEX_8,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            Y_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }


        page_offset += FLASH_XY_DATA_LENGTH;
    }



    //================ Y page9 ================
    if(flash_check(FLASH_SECTION_INDEX,
                   Y_memery_page_INDEX_9))
    {

        flash_read_page_to_buffer(
            FLASH_SECTION_INDEX,
            Y_memery_page_INDEX_9,
            FLASH_XY_DATA_LENGTH
        );


        for(size_t i=0;i<FLASH_XY_DATA_LENGTH;i++)
        {
            Y_load[page_offset+i] =
                flash_union_buffer[i].float_type;
        }

    }


    flash_xy_flag = 4;

}


/**
 * @brief 清除路径存储Flash区域
 * @note 上电初始化时调用
 *       清除 X/Y/Yaw 所使用的Flash页
 */
void flash_road_memory_clear(void)
{
    // 清除X轴数据页 1~5
    for(uint32 page = X_memery_page_INDEX_1;
        page <= X_memery_page_INDEX_5;
        page++)
    {
        flash_erase_page(FLASH_SECTION_INDEX, page);
    }


    // 清除Y轴数据页 6~10
    for(uint32 page = Y_memery_page_INDEX_6;
        page <= Y_memery_page_INDEX_10;
        page++)
    {
        flash_erase_page(FLASH_SECTION_INDEX, page);
    }


    // 清除Yaw数据页 11~15
    for(uint32 page = Yaw_memery_page_INDEX_11;
        page <= Yaw_memery_page_INDEX_15;
        page++)
    {
        flash_erase_page(FLASH_SECTION_INDEX, page);
    }
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