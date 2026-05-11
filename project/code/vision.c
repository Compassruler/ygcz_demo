#include "vision.h"

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
