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
