#include "camera.h"
#include "camera_image_processing.h"
#include "screen.h"
#include "zf_device_mt9v03x.h"
#include <string.h>

#define LED1                    (P19_0)

static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static uint32 camera_frame_count = 0;
static uint8 camera_processed_image_valid = 0;

static uint8 camera_copy_and_process_frame(void)
{
    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    mt9v03x_finish_flag = 0;
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    camera_image_binary_otsu(image_copy);
    camera_image_filter_isolated_black(image_copy);
    camera_image_filter_isolated_white(image_copy);

    camera_processed_image_valid = 1;

    return 1;
}

uint8 camera_has_frame(void)
{
    return mt9v03x_finish_flag;
}

void camera_send_frame(void)
{
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

        vision_binary_fixed(image_copy, 100);
        camera_image_filter_isolated_black(image_copy);
        camera_image_filter_isolated_white(image_copy);

        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_MT9V03X,
            image_copy[0],
            MT9V03X_W,
            MT9V03X_H
        );
        seekfree_assistant_camera_send();
        camera_frame_count++;
    }
}

uint32 camera_get_frame_count(void)
{
    return camera_frame_count;
}

void camera_init(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    while(mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }
}

void camera_debug_on_screen(uint16 x, uint16 y, uint16 display_width, uint16 display_height)
{
    camera_copy_and_process_frame();

    if(!camera_processed_image_valid)
    {
        return;
    }

    screen_show_camera_image(x, y, image_copy[0], display_width, display_height);
}

uint8 camera_processing(uint16 row, uint16 row_total, uint16 colum, uint16 colum_total, uint16 black_pix_count)
{
    if(!camera_copy_and_process_frame())
    {
        return 0;
    }

    return camera_image_check_jump_area(
        image_copy,
        row,
        row_total,
        colum,
        colum_total,
        black_pix_count
    );
}
