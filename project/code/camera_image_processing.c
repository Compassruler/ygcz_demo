#include "camera_image_processing.h"
#include <string.h>

static uint32 jump_trigger_count = 0;

void vision_binary_fixed(uint8 image[MT9V03X_H][MT9V03X_W], uint8 threshold)
{
    uint16 x = 0;
    uint16 y = 0;

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            if(image[y][x] > threshold)
            {
                image[y][x] = 255;
            }
            else
            {
                image[y][x] = 0;
            }
        }
    }
}

uint8 camera_image_binary_otsu(uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 threshold_temp = 0;
    uint8 threshold = 0;
    uint32 total = MT9V03X_IMAGE_SIZE;
    uint32 sum = 0;
    uint32 sum_background = 0;
    uint32 weight_background = 0;
    uint32 weight_foreground = 0;
    uint32 histogram[256] = {0};
    double mean_background = 0;
    double mean_foreground = 0;
    double between_class_variance = 0;
    double max_between_class_variance = 0;

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            histogram[image[y][x]]++;
        }
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        sum += threshold_temp * histogram[threshold_temp];
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        weight_background += histogram[threshold_temp];
        if(weight_background == 0)
        {
            continue;
        }

        weight_foreground = total - weight_background;
        if(weight_foreground == 0)
        {
            break;
        }

        sum_background += threshold_temp * histogram[threshold_temp];
        mean_background = (double)sum_background / weight_background;
        mean_foreground = (double)(sum - sum_background) / weight_foreground;
        between_class_variance = (double)weight_background *
                                 (double)weight_foreground *
                                 (mean_background - mean_foreground) *
                                 (mean_background - mean_foreground);

        if(between_class_variance > max_between_class_variance)
        {
            max_between_class_variance = between_class_variance;
            threshold = (uint8)threshold_temp;
        }
    }

    vision_binary_fixed(image, threshold);

    return threshold;
}

uint8 camera_image_binary_otsu_roi(uint8 image[MT9V03X_H][MT9V03X_W], uint16 roi_row, uint16 roi_row_count, uint16 roi_column, uint16 roi_column_count)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 checked_rows = 0;
    uint16 checked_columns = 0;
    uint16 threshold_temp = 0;
    uint8  threshold = 0;
    uint32 total = 0;
    uint32 sum = 0;
    uint32 sum_background = 0;
    uint32 weight_background = 0;
    uint32 weight_foreground = 0;
    uint32 histogram[256] = {0};
    double mean_background = 0;
    double mean_foreground = 0;
    double between_class_variance = 0;
    double max_between_class_variance = 0;

    if((MT9V03X_H <= roi_row) || (MT9V03X_W <= roi_column))
    {
        return camera_image_binary_otsu(image);
    }

    if((0 == roi_row_count) || (0 == roi_column_count))
    {
        return camera_image_binary_otsu(image);
    }

    if((roi_row + 1) < roi_row_count)
    {
        return camera_image_binary_otsu(image);
    }

    if((MT9V03X_W - roi_column) < roi_column_count)
    {
        return camera_image_binary_otsu(image);
    }

    total = (uint32)roi_row_count * (uint32)roi_column_count;

    for(checked_rows = 0; checked_rows < roi_row_count; checked_rows++)
    {
        y = roi_row - checked_rows;

        for(checked_columns = 0; checked_columns < roi_column_count; checked_columns++)
        {
            x = roi_column + checked_columns;
            histogram[image[y][x]]++;
        }
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        sum += threshold_temp * histogram[threshold_temp];
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        weight_background += histogram[threshold_temp];
        if(weight_background == 0)
        {
            continue;
        }

        weight_foreground = total - weight_background;
        if(weight_foreground == 0)
        {
            break;
        }

        sum_background += threshold_temp * histogram[threshold_temp];
        mean_background = (double)sum_background / weight_background;
        mean_foreground = (double)(sum - sum_background) / weight_foreground;
        between_class_variance = (double)weight_background *
                                 (double)weight_foreground *
                                 (mean_background - mean_foreground) *
                                 (mean_background - mean_foreground);

        if(between_class_variance > max_between_class_variance)
        {
            max_between_class_variance = between_class_variance;
            threshold = (uint8)threshold_temp;
        }
    }

    if(0 == max_between_class_variance)
    {
        return camera_image_binary_otsu(image);
    }

    vision_binary_fixed(image, threshold);

    return threshold;
}

void camera_image_filter_isolated_black(uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 x = 0;
    uint16 y = 0;
    int16 dx = 0;
    int16 dy = 0;
    uint8 black_count = 0;
    static uint8 image_temp[MT9V03X_H][MT9V03X_W];

    memcpy(image_temp[0], image[0], MT9V03X_IMAGE_SIZE);

    for(y = 1; y < MT9V03X_H - 1; y++)
    {
        for(x = 1; x < MT9V03X_W - 1; x++)
        {
            if(image_temp[y][x] == 0)
            {
                black_count = 0;

                for(dy = -1; dy <= 1; dy++)
                {
                    for(dx = -1; dx <= 1; dx++)
                    {
                        if(image_temp[y + dy][x + dx] == 0)
                        {
                            black_count++;
                        }
                    }
                }

                if(black_count <= 2)
                {
                    image[y][x] = 255;
                }
            }
        }
    }
}

void camera_image_filter_isolated_white(uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 x = 0;
    uint16 y = 0;
    int16 dx = 0;
    int16 dy = 0;
    uint8 white_count = 0;
    static uint8 image_temp[MT9V03X_H][MT9V03X_W];

    memcpy(image_temp[0], image[0], MT9V03X_IMAGE_SIZE);

    for(y = 1; y < MT9V03X_H - 1; y++)
    {
        for(x = 1; x < MT9V03X_W - 1; x++)
        {
            if(image_temp[y][x] == 255)
            {
                white_count = 0;

                for(dy = -1; dy <= 1; dy++)
                {
                    for(dx = -1; dx <= 1; dx++)
                    {
                        if(image_temp[y + dy][x + dx] == 255)
                        {
                            white_count++;
                        }
                    }
                }

                if(white_count <= 2)
                {
                    image[y][x] = 0;
                }
            }
        }
    }
}


uint8 camera_image_check_jump_rows(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint16 black_count)
{
    uint16 x = 0;
    int16 y = 0;
    uint16 checked_columns = 0;
    uint16 current_black_count = 0;
    uint16 checked_rows = 0;

    if((MT9V03X_H <= check_row) || (MT9V03X_W <= check_column))
    {
        return 0;
    }

    if((0 == check_row_count) || (0 == check_column_count))
    {
        return 0;
    }

    for(y = (int16)check_row; (0 <= y) && (checked_rows < check_row_count); y--)
    {
        current_black_count = 0;
        checked_columns = 0;

        for(x = check_column; (x < MT9V03X_W) && (checked_columns < check_column_count); x++)
        {
            if(image[y][x] == 0)
            {
                current_black_count++;
            }

            checked_columns++;
        }

        if(checked_columns != check_column_count)
        {
            return 0;
        }

        if(current_black_count < black_count)
        {
            return 0;
        }

        checked_rows++;
    }

    return (checked_rows == check_row_count);
}

uint8 camera_image_check_jump_columns(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint16 black_count)
{
    uint16 x = 0;
    int16 y = 0;
    uint16 checked_columns = 0;
    uint16 current_black_count = 0;
    uint16 checked_rows = 0;

    if((MT9V03X_H <= check_row) || (MT9V03X_W <= check_column))
    {
        return 0;
    }

    if((0 == check_row_count) || (0 == check_column_count))
    {
        return 0;
    }

    for(x = check_column; (x < MT9V03X_W) && (checked_columns < check_column_count); x++)
    {
        current_black_count = 0;
        checked_rows = 0;

        for(y = (int16)check_row; (0 <= y) && (checked_rows < check_row_count); y--)
        {
            if(image[y][x] == 0)
            {
                current_black_count++;
            }

            checked_rows++;
        }

        if(checked_rows != check_row_count)
        {
            return 0;
        }

        if(current_black_count < black_count)
        {
            return 0;
        }

        checked_columns++;
    }

    return (checked_columns == check_column_count);
}

uint8 camera_image_check_jump_area(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint32 dot_count, uint32 dot_type)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 checked_rows = 0;
    uint16 checked_columns = 0;
    uint8 target_dot_value = 0;
    uint32 current_dot_count = 0;
    uint32 check_area_size = 0;

    if((MT9V03X_H <= check_row) || (MT9V03X_W <= check_column))
    {
        return 0;
    }

    if((0 == check_row_count) || (0 == check_column_count))
    {
        return 0;
    }

    if((check_row + 1) < check_row_count)
    {
        return 0;
    }

    if((MT9V03X_W - check_column) < check_column_count)
    {
        return 0;
    }

    if(CAMERA_IMAGE_DOT_BLACK == dot_type)
    {
        target_dot_value = 0;
    }
    else if(CAMERA_IMAGE_DOT_WHITE == dot_type)
    {
        target_dot_value = 255;
    }
    else
    {
        return 0;
    }

    check_area_size = (uint32)check_row_count * (uint32)check_column_count;
    if(check_area_size < dot_count)
    {
        return 0;
    }

    if(0 == dot_count)
    {
        return 1;
    }

    for(checked_rows = 0; checked_rows < check_row_count; checked_rows++)
    {
        y = check_row - checked_rows;

        for(checked_columns = 0; checked_columns < check_column_count; checked_columns++)
        {
            x = check_column + checked_columns;

            if(image[y][x] == target_dot_value)
            {
                current_dot_count++;

                if(current_dot_count >= dot_count)
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

uint8 camera_image_check_jump_strict(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 row_black_count, uint16 check_column, uint16 check_column_count, uint16 column_black_count)
{
    if(!camera_image_check_jump_rows(image, check_row, check_row_count, check_column, check_column_count, row_black_count))
    {
        return 0;
    }

    return camera_image_check_jump_columns(image, check_row, check_row_count, check_column, check_column_count, column_black_count);
}


// 跳跃触发冷却时间检�?
uint8 camera_image_jump_trigger_filter(uint32 time_ms, uint32 cooldown_time_ms, uint8 jump_detected)
{
    static uint32 last_jump_time = 0;
    static uint8 has_triggered = 0;

    if(!jump_detected)
    {
        return 0;
    }

    if(has_triggered && ((time_ms - last_jump_time) < cooldown_time_ms))
    {
        return 0;
    }

    has_triggered = 1;
    last_jump_time = time_ms;

    return 1;
}

uint8 camera_dot_type_switch(void)
{
    jump_trigger_count++;
    return (uint8)dot_type_list[jump_trigger_count % CAMERA_DOT_TYPE_LIST_COUNT];
}

uint32 camera_dot_type_get_steps(void)
{
    return jump_trigger_count;
}

uint8 camera_dot_type_reset(void)
{
    jump_trigger_count = 0;
    return (uint8)dot_type_list[0];
}
