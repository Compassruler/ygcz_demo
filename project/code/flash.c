#include "zf_common_headfile.h"



uint8_t flash_yaw_flag = 0;      // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
uint8_t flash_xy_flag = 0; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志；
uint8_t flash_turn_flag = 0; // 0为初始状态，1为开始存，2为开始取，3为存完标志，4为取完标志
uint32_t path_write_page; //写路径点的起始页/对应页
uint32_t path_read_page; // 读取flash的起始页/对应页
/**
 * @brief 清除路径存储Flash区域
 * @note 上电初始化时调用
 *       清除 X/Y/Yaw 所使用的Flash页
 */
void flash_path_memory_clear(void)
{

    //清除总header
    flash_erase_page(FLASH_SECTION_INDEX, COURSE3_HEADER_PAGE);


    //清除segment header
    flash_erase_page(FLASH_SECTION_INDEX, COURSE3_SEGMENT_HEADER_PAGE);



    //清除路径点区域(改清除的页数)

    for(uint32 page = COURSE3_POINT_START_PAGE; page < FLASH_PAGE_NUM; page++)
    {
        flash_erase_page(FLASH_SECTION_INDEX,page);
    }
}


// flash写入科目1/2单段头信息
void flash_write_single_header(void)
{
    uint32_t header_page;


    //清空buffer
    flash_buffer_clear();



    //根据科目选择magic和页地址

    if(course_record_flag == 0)
    {
        //科目1

        single_header.check = COURSE1_CHECK_VALUE;

        header_page = COURSE1_HEADER_PAGE;

    }


    else if(course_record_flag == 1)
    {
        //科目2

        single_header.check = COURSE2_CHECK_VALUE;

        header_page = COURSE2_HEADER_PAGE;

    }


    else
    {
      // 无效
        return;
    }



    //记录点数量

    single_header.point_num = record_total_index;



    //写入buffer

    flash_union_buffer[0].uint32_type =
        single_header.check;


    flash_union_buffer[1].uint32_type =
        single_header.point_num;



    //写Flash

    flash_write_page_from_buffer(
        0,
        header_page,
        2
    );

}


//flash写入科三总头信息
void flash_write_path_header(void)
{       
  flash_buffer_clear();
  flash_union_buffer[0].uint32_type = mul_header.check;


flash_union_buffer[1].uint32_type = mul_header.segment_num;


flash_union_buffer[2].uint32_type = mul_header.total_point_num;


flash_write_page_from_buffer(0, COURSE3_HEADER_PAGE, 3);
  
}

// flash写入段头信息
void flash_write_segment_headers(void)
{

    if(mul_header.segment_num > MAX_SEGMENT_NUM)
        return;


    flash_buffer_clear();


    for(uint32_t i=0;i<mul_header.segment_num;i++)
    {
        flash_union_buffer[i*2].uint32_type = segment_header[i].check;
        flash_union_buffer[i*2+1].uint32_type = segment_header[i].point_num;
    }


    flash_write_page_from_buffer(0, COURSE3_SEGMENT_HEADER_PAGE, mul_header.segment_num*2);

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


        uint32_t write_num = point_num - point_index;


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
// 将统一图像二值化阈值写入 Flash
void flash_write_vision_threshold()
{
    flash_buffer_clear(); // 清除缓冲区
    flash_union_buffer[0].uint32_type = (uint32)vision_binary_threshold;
    flash_write_page_from_buffer(0, VISION_THRESHOLD_PAGE, 1);
}


// flash写入总
void flash_path_store(void)
{

    if(course_record_flag == 0)
    {
        //科目1

        path_write_page = COURSE1_POINT_START_PAGE;

        flash_write_single_header();

        flash_write_all_points(
            record_path,
            record_total_index
        );

    }


    else if(course_record_flag == 1)
    {
        //科目2

        path_write_page = COURSE2_POINT_START_PAGE;

        flash_write_single_header();

        flash_write_all_points(
            record_path,
            record_total_index
        );

    }


    else if(course_record_flag == 2)
    {
        //科目3


        path_write_page = COURSE3_POINT_START_PAGE;


        flash_write_path_header();


        flash_write_segment_headers();


        flash_write_all_points(
            record_path,
            mul_header.total_point_num
        );
//        flash_write_vision_threshold();

    }
    
    
    
}

// flash读取科目三总头
void flash_read_path_header(void)
{
    flash_read_page_to_buffer(
        0,
        COURSE3_HEADER_PAGE,
        3
    );

    mul_header.check = flash_union_buffer[0].uint32_type;

    mul_header.segment_num = flash_union_buffer[1].uint32_type;

    mul_header.total_point_num = flash_union_buffer[2].uint32_type;

    if(mul_header.check != COURSE3_CHECK_VALUE)
    {
        mul_header.total_point_num = 0;
        return;
    }
}

// flash读取单段科目头（科一科二用）
void flash_read_single_header(void)
{

    uint32_t page;


    if(course_load_flag == 0)
    {
        //科目1

        page = COURSE1_HEADER_PAGE;

    }


    else if(course_load_flag == 1)
    {
        //科目2

        page = COURSE2_HEADER_PAGE;

    }


    else
    {
        return;
    }



    flash_read_page_to_buffer(0,page,2);



    single_header.check =
        flash_union_buffer[0].uint32_type;



    single_header.point_num =
        flash_union_buffer[1].uint32_type;



    //校验

    if(course_load_flag == 0)
    {

        if(single_header.check != COURSE1_CHECK_VALUE)
        {
            single_header.point_num = 0;
            return;
        }

    }



    else if(course_load_flag == 1)
    {

        if(single_header.check != COURSE2_CHECK_VALUE)
        {
            single_header.point_num = 0;
            return;
        }

    }



    //点数量保护（过于宽泛目前）

    if(single_header.point_num >
       FLASH_PAGE_LENGTH * Use_page)
    {
        single_header.point_num = 0;
    }

}

// flash读取段头
void flash_read_segment_headers(void)
{

    if(mul_header.segment_num > MAX_SEGMENT_NUM)
    {
        return;
    }


    flash_read_page_to_buffer(
        0,
        COURSE3_SEGMENT_HEADER_PAGE,
        mul_header.segment_num * 2
    );


    for(uint32_t i=0;i<mul_header.segment_num;i++)
    {

        segment_header[i].check =
            flash_union_buffer[i*2].uint32_type;


        segment_header[i].point_num =
            flash_union_buffer[i*2+1].uint32_type;


        // 校验段头
        if(segment_header[i].check != PATH_SEGMENT_VALUE)
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


    //读取起始页由外部设置
    uint32_t page = path_read_page;



    while(point_index < point_num)
    {

        uint32_t read_num =
            point_num-point_index;


        if(read_num > PATH_POINT_PER_PAGE)
        {
            read_num = PATH_POINT_PER_PAGE;
        }



        //页越界保护

        if(page >= FLASH_PAGE_NUM)
        {
            return;
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


// 从 Flash 读取统一图像二值化阈值
void flash_read_vision_threshold(void)
{
    uint32 stored_threshold;

    flash_read_page_to_buffer(0, VISION_THRESHOLD_PAGE, 1);
    stored_threshold = flash_union_buffer[0].uint32_type;

    if((stored_threshold >= 1u) && (stored_threshold <= 255u))
    {
        vision_binary_threshold = (uint8)stored_threshold;
    }
}

// flash读取总
void flash_path_load(void)
{

    if(course_load_flag == 0)
    {
        //科目1

        path_read_page =
        COURSE1_POINT_START_PAGE;


        flash_read_single_header();


        flash_read_all_points(
            replay_point,
            single_header.point_num
        );
    replay_point_num = single_header.point_num;
    segment_end_index = replay_point_num - 1;
    KP_DIS = 13.0;
    LOOK_AHEAD_DISTANCE = 0.20;
    TURN_CHECK_POINT = 40;
    TURN_SPEED_LIMIT  = 350;
    TURN_SPEED_SCALE = 0.30f;
    MIN_SPEED = 100;
    TURN_ANGLE_LIMIT = 30;
    }


    else if(course_load_flag == 1)
    { 
        //科目2

        path_read_page =
        COURSE2_POINT_START_PAGE;


        flash_read_single_header();


        flash_read_all_points(
            replay_point,
            single_header.point_num
        );
    replay_point_num = single_header.point_num;
    segment_end_index = replay_point_num - 1;
    KP_DIS = 6.0;       
    TURN_CHECK_POINT = 30;
    TURN_SPEED_LIMIT = 100;
    TURN_SPEED_SCALE = 0.10;
    MIN_SPEED = 100;
    TURN_ANGLE_LIMIT = 10;
    }


    else if(course_load_flag == 2)
    {
        //科目3

        path_read_page =
        COURSE3_POINT_START_PAGE;


        flash_read_path_header();


        flash_read_segment_headers();


        flash_read_all_points(
            replay_point,
            mul_header.total_point_num
        );
        
    replay_point_num = mul_header.total_point_num;
    segment_end_index = segment_header[0].point_num - 1;
    KP_DIS = 4.5;
//    flash_read_vision_threshold();
    }
    yaw_angle = -3; // 发车航向角清0，防止歪了
    x = 0;
    y = 0;
    x_last = 0;
    y_last = 0;// 给惯导数据清零
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