#include "camera_image_processing.h"
#include <string.h>

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


uint8 camera_image_check_jump(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 black_count)
{
    uint16 x = 0;
    int16 y = 0;
    uint16 current_black_count = 0;
    uint16 checked_rows = 0;

    if(MT9V03X_H <= check_row)
    {
        return 0;
    }

    if(0 == check_row_count)
    {
        return 0;
    }

    for(y = (int16)check_row; (0 <= y) && (checked_rows < check_row_count); y--)
    {
        current_black_count = 0;

        for(x = 0; x < MT9V03X_W; x++)
        {
            if(image[y][x] == 0)
            {
                current_black_count++;
            }
        }

        if(current_black_count < black_count)
        {
            return 0;
        }

        checked_rows++;
    }

    return (checked_rows == check_row_count);
}
