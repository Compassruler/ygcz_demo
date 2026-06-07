#include "camera.h"
#include "camera_image_processing.h"
#include "screen.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_wifi_spi.h"
#include <string.h>

#define LED1                    (P19_0)

static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static uint32 camera_frame_count = 0;
static uint8 camera_processed_image_valid = 0;
static uint8 camera_wifi_spi_ready = 0;
static uint16 camera_wifi_frame_count = 0;

static uint8 camera_assistant_channel_to_index(uint8 channel, uint8 *index)
{
    if((0 == channel) || (SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT < channel) || (NULL == index))
    {
        return 0;
    }

    *index = channel - 1;

    return 1;
}

static float camera_limit_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

static void camera_config_assistant_image(void)
{
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        MT9V03X_W,
        MT9V03X_H
    );
}

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

static uint8 camera_copy_and_process_frame_roi(JumpDetectParams_t *jump_params)
{
    if(NULL == jump_params)
    {
        return 0;
    }

    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    mt9v03x_finish_flag = 0;
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    camera_image_binary_otsu_roi(
        image_copy,
        jump_params->otsu_roi_row,
        jump_params->otsu_roi_row_count,
        jump_params->otsu_roi_column,
        jump_params->otsu_roi_column_count
    );
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

uint8 camera_wifi_spi_init(char *wifi_ssid, char *pass_word, char *target_ip, char *target_port, char *local_port)
{
    uint8 return_state = 0;

    camera_wifi_spi_ready = 0;
    camera_wifi_frame_count = 0;

    return_state = wifi_spi_init(wifi_ssid, pass_word);
    if(return_state)
    {
        return return_state;
    }

    return_state = wifi_spi_socket_connect("UDP", target_ip, target_port, local_port);
    if(return_state)
    {
        return return_state;
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    camera_config_assistant_image();

    camera_wifi_spi_ready = 1;

    return 0;
}

uint8 camera_assistant_wifi_spi_init(char *wifi_ssid, char *pass_word, char *target_ip, char *target_port, char *local_port)
{
    return camera_wifi_spi_init(wifi_ssid, pass_word, target_ip, target_port, local_port);
}

void camera_assistant_parameter_update(void)
{
    seekfree_assistant_data_analysis();
}

uint8 camera_assistant_parameter_read_float(uint8 channel, float *value, float min_value, float max_value)
{
    uint8 index = 0;
    float parameter_value = 0.0f;

    if((NULL == value) || !camera_assistant_channel_to_index(channel, &index))
    {
        return 0;
    }

    if(!seekfree_assistant_parameter_update_flag[index])
    {
        return 0;
    }

    seekfree_assistant_parameter_update_flag[index] = 0;
    parameter_value = seekfree_assistant_parameter[index];
    *value = camera_limit_float(parameter_value, min_value, max_value);

    return 1;
}

uint8 camera_assistant_parameter_read_int16(uint8 channel, int16 *value, int16 min_value, int16 max_value)
{
    float parameter_value = 0.0f;
    int16 rounded_value = 0;

    if(NULL == value)
    {
        return 0;
    }

    if(!camera_assistant_parameter_read_float(channel, &parameter_value, (float)min_value, (float)max_value))
    {
        return 0;
    }

    rounded_value = (0.0f <= parameter_value) ? (int16)(parameter_value + 0.5f) : (int16)(parameter_value - 0.5f);
    *value = rounded_value;

    return 1;
}

uint8 camera_assistant_parameter_read_uint16(uint8 channel, uint16 *value, uint16 min_value, uint16 max_value)
{
    float parameter_value = 0.0f;

    if(NULL == value)
    {
        return 0;
    }

    if(!camera_assistant_parameter_read_float(channel, &parameter_value, (float)min_value, (float)max_value))
    {
        return 0;
    }

    *value = (uint16)(parameter_value + 0.5f);

    return 1;
}

uint8 camera_assistant_parameter_read_uint32(uint8 channel, uint32 *value, uint32 min_value, uint32 max_value)
{
    float parameter_value = 0.0f;

    if(NULL == value)
    {
        return 0;
    }

    if(!camera_assistant_parameter_read_float(channel, &parameter_value, (float)min_value, (float)max_value))
    {
        return 0;
    }

    *value = (uint32)(parameter_value + 0.5f);

    return 1;
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

void camera_debug_on_screen()  //(uint16 x, uint16 y, uint16 display_width, uint16 display_height)
{
    uint16 x              = IMAGE_X;
    uint16 y              = IMAGE_Y;
    uint16 display_width  = IMAGE_DISPLAY_WIDTH;
    uint16 display_height = IMAGE_DISPLAY_HEIGHT;

    camera_copy_and_process_frame();

    if(!camera_processed_image_valid)
    {
        return;
    }

    screen_show_camera_image(x, y, image_copy[0], display_width, display_height);
}

void camera_debug_on_wifi_spi(uint16 send_div)
{
    if(!camera_wifi_spi_ready)
    {
        return;
    }

    camera_copy_and_process_frame();

    if(!camera_processed_image_valid)
    {
        return;
    }

    if(0 == send_div)
    {
        send_div = 1;
    }

    if(0 == (camera_wifi_frame_count % send_div))
    {
        camera_config_assistant_image();
        seekfree_assistant_camera_send();
        camera_frame_count++;
    }

    camera_wifi_frame_count++;
}


uint32 calc_fps(uint32 time_ms, uint32 *frame_count, uint32 *fps)
{
    static uint32 last_1s_time = 0;
    if (time_ms - last_1s_time >= 1000)
    {
        last_1s_time = time_ms;
        *fps = *frame_count;
        *frame_count = 0;
        return *fps;
    }
    
    return *fps;
}

uint8 camera_processing(uint32 time_ms, JumpDetectParams_t *jump_params)
{
    uint8 jump_detected = 0;
    uint32 required_frame_count = 0;
    static uint32 multi_frame_count = 0;

    if(NULL == jump_params)
    {
        return 0;
    }

    if(!camera_copy_and_process_frame())
    {
        return 0;
    }

    required_frame_count = jump_params->multi_frame;
    if(0 == required_frame_count)
    {
        required_frame_count = 1;
    }

    // 严格检测：要求检测区域内每一行、每一列的黑色像素数量都达到阈值
    if(jump_params->algo_type == CAMERA_JUMP_ALGO_STRICT)
    {
        if(jump_params->dot_count > 0xFFFFu)
        {
            multi_frame_count = 0;
            return 0;
        }

        jump_detected = camera_image_check_jump_strict(
            image_copy,
            jump_params->check_row,
            jump_params->check_row_count,
            (uint16)jump_params->dot_count,
            jump_params->check_column,
            jump_params->check_column_count,
            (uint16)jump_params->dot_count
        );
    }
    else if(jump_params->algo_type == CAMERA_JUMP_ALGO_AREA)
    {   
        // 矩形检测：统计指定矩形区域内的黑色或白色像素总数
        jump_detected = camera_image_check_jump_area(
            image_copy, 
            jump_params->check_row, 
            jump_params->check_row_count, 
            jump_params->check_column, 
            jump_params->check_column_count, 
            jump_params->dot_count,
            jump_params->dot_type
        );
    }
    else
    {
        return 0;
    }

    // 多帧确认要求连续检测到目标，期间只要有一帧未检测到就重新计数
    if(!jump_detected)
    {
        multi_frame_count = 0;
        return 0;
    }

    multi_frame_count++;
    if(multi_frame_count < required_frame_count)
    {
        return 0;
    }

    multi_frame_count = 0;

    // 通过多帧确认后，再进入触发冷却过滤，避免连续重复触发
    jump_detected = camera_image_jump_trigger_filter(time_ms, jump_params->cooldown_time_ms, 1);

    if(jump_detected)
    {
        jump_params->dot_type = camera_dot_type_switch();
        jump_params->steps = camera_dot_type_get_steps();

        if (jump_params->steps == CAMERA_DOT_TYPE_LIST_COUNT)
        {
            jump_params->dot_type = camera_dot_type_reset();
            jump_params->steps = camera_dot_type_get_steps();
        }
        
        return 1;
    }
    
    return 0;
}

uint8 camera_processing_roi(uint32 time_ms, JumpDetectParams_t *jump_params)
{
    uint8 jump_detected = 0;
    uint32 required_frame_count = 0;
    static uint32 multi_frame_count = 0;

    if(NULL == jump_params)
    {
        return 0;
    }

    if(!camera_copy_and_process_frame_roi(jump_params))
    {
        return 0;
    }

    required_frame_count = jump_params->multi_frame;
    if(0 == required_frame_count)
    {
        required_frame_count = 1;
    }

    if(jump_params->algo_type == CAMERA_JUMP_ALGO_STRICT)
    {
        if(jump_params->dot_count > 0xFFFFu)
        {
            multi_frame_count = 0;
            return 0;
        }

        jump_detected = camera_image_check_jump_strict(
            image_copy,
            jump_params->check_row,
            jump_params->check_row_count,
            (uint16)jump_params->dot_count,
            jump_params->check_column,
            jump_params->check_column_count,
            (uint16)jump_params->dot_count
        );
    }
    else if(jump_params->algo_type == CAMERA_JUMP_ALGO_AREA)
    {
        jump_detected = camera_image_check_jump_area(
            image_copy,
            jump_params->check_row,
            jump_params->check_row_count,
            jump_params->check_column,
            jump_params->check_column_count,
            jump_params->dot_count,
            jump_params->dot_type
        );
    }
    else
    {
        return 0;
    }

    if(!jump_detected)
    {
        multi_frame_count = 0;
        return 0;
    }

    multi_frame_count++;
    if(multi_frame_count < required_frame_count)
    {
        return 0;
    }

    multi_frame_count = 0;

    jump_detected = camera_image_jump_trigger_filter(time_ms, jump_params->cooldown_time_ms, 1);

    if(jump_detected)
    {
        jump_params->dot_type = camera_dot_type_switch();
        jump_params->steps = camera_dot_type_get_steps();

        if (jump_params->steps == CAMERA_DOT_TYPE_LIST_COUNT)
        {
            jump_params->dot_type = camera_dot_type_reset();
            jump_params->steps = camera_dot_type_get_steps();
        }

        return 1;
    }

    return 0;
}
