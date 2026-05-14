#include "camera.h"
#include "camera_image_processing.h"
#include "screen.h"
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
        vision_binary_fixed(image_copy, 100);                            // äºŒå€¼åŒ–å¤„ç†,å?ä»¥æ ¹æ?å®žé™…æƒ…å†µè°ƒæ•´é˜ˆå€?(80/100/120/140/160)
        camera_image_filter_isolated_black(image_copy);                  // åŽ»é™¤äºŒå€¼å›¾ä¸?çš„å?¤ç«‹é»‘è‰²å™?ç‚?
        camera_image_filter_isolated_white(image_copy);
        seekfree_assistant_camera_send();
        camera_frame_count++;
    }
}

uint32 camera_wireless_get_frame_count(void)
{
    return camera_frame_count;
}

void camera_wireless_screen_init(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    screen_init();

    while(mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }
}

uint8 camera_show_processed_frame_on_screen(uint16 x, uint16 y, uint16 display_width, uint16 display_height, uint16 check_row, uint16 check_row_count, uint16 black_count)
{
    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    mt9v03x_finish_flag = 0;
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE); // ¸´ÖÆÍ¼Ïñ

    camera_image_binary_otsu(image_copy);  // ´ó½ò·¨´¦Àí¶þÖµ»¯
    camera_image_filter_isolated_black(image_copy);  // ÂËÈ¥ºÚÉ«Ôëµã
    camera_image_filter_isolated_white(image_copy);  // ÂËÈ¥°×É«Ôëµã
    
    screen_show_camera_image(x, y, image_copy[0], display_width, display_height);  // ·µ»ØÍ¼Ïñ

    return camera_image_check_jump(image_copy, check_row, check_row_count, black_count);  // ¼ì²éÊÇ·ñÌøÔ¾, Ä¿Ç°Óëdebug image ºÏ²¢
}

