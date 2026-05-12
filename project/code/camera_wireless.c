#include "camera_wireless.h"
#include "camera_image_processing.h"
#include "zf_device_mt9v03x.h"
#include <string.h>
#define LED1                    (P19_0)

static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static uint32 camera_frame_count = 0;

void camera_wireless_init(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    if(wireless_uart_init())
    {
        while(1)
        {
            gpio_toggle_level(LED1);
            system_delay_ms(100);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);

    while(mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        MT9V03X_W,
        MT9V03X_H
    );
}

uint8 camera_wireless_has_frame(void)
{
    return mt9v03x_finish_flag;
}

void camera_wireless_send_frame(void)
{
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        vision_binary_fixed(image_copy, 100);                            // 二值化处理,可以根据实际情况调整阈值(80/100/120/140/160)
        camera_image_filter_isolated_black(image_copy);                  // 去除二值图中的孤立黑色噪点
        seekfree_assistant_camera_send();
        camera_frame_count++;
    }
}

uint32 camera_wireless_get_frame_count(void)
{
    return camera_frame_count;
}

