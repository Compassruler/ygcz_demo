#include "camera.h"
#include "camera_image_processing.h"
#include "screen.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_wifi_spi.h"
#include <math.h>
#include <string.h>

#define LED1                    (P19_0)
#define CAMERA_BINARY_THRESHOLD_DEFAULT     (100u)
#define CAMERA_BRIDGE_CAMERA_RAD_TO_D10      (572.9577951f)

static uint8 image_copy[MT9V03X_H][MT9V03X_W];
static uint32 camera_frame_count = 0;
static uint8 camera_processed_image_valid = 0;
static uint8 camera_wifi_spi_ready = 0;
static uint16 camera_wifi_frame_count = 0;

typedef struct
{
    uint8 initialized;
    uint8 pending_count;
    uint8 invalid_count;
    uint16 reference_row;
    float slope;
    float reference_x;
    float edge_top;
    float edge_bottom;
    float pending_slope;
    float pending_reference_x;
    float pending_edge_top;
    float pending_edge_bottom;
    int16 pending_angle_d10;
} CameraBridgeFilterState_t;

static CameraBridgeFilterState_t camera_bridge_filter_state = {0};

static int32 camera_abs_int32(int32 value)
{
    return (value < 0) ? -value : value;
}

static uint16 camera_bridge_round_limit(float value, uint16 upper_limit)
{
    int32 rounded_value = 0;

    if(value >= 0.0f)
    {
        rounded_value = (int32)(value + 0.5f);
    }
    else
    {
        rounded_value = (int32)(value - 0.5f);
    }

    if(rounded_value < 0)
    {
        rounded_value = 0;
    }
    else if(rounded_value >= upper_limit)
    {
        rounded_value = upper_limit - 1u;
    }

    return (uint16)rounded_value;
}

static int16 camera_bridge_slope_to_angle_d10(float slope)
{
    float angle_d10 = atanf(slope) * CAMERA_BRIDGE_CAMERA_RAD_TO_D10;

    if(angle_d10 >= 0.0f)
    {
        return (int16)(angle_d10 + 0.5f);
    }

    return (int16)(angle_d10 - 0.5f);
}

static void camera_bridge_filter_set_raw(const CameraBridgeResult_t *raw_result)
{
    camera_bridge_filter_state.initialized = 1;
    camera_bridge_filter_state.pending_count = 0;
    camera_bridge_filter_state.invalid_count = 0;
    camera_bridge_filter_state.reference_row = raw_result->reference_row;
    camera_bridge_filter_state.slope = raw_result->edge_slope;
    camera_bridge_filter_state.reference_x =
        raw_result->edge_slope * (float)raw_result->reference_row + raw_result->edge_intercept;
    camera_bridge_filter_state.edge_top = (float)raw_result->edge_y1;
    camera_bridge_filter_state.edge_bottom = (float)raw_result->edge_y2;
}

static void camera_bridge_filter_start_pending(
    const CameraBridgeResult_t *raw_result,
    float raw_reference_x)
{
    camera_bridge_filter_state.pending_count = 1;
    camera_bridge_filter_state.pending_slope = raw_result->edge_slope;
    camera_bridge_filter_state.pending_reference_x = raw_reference_x;
    camera_bridge_filter_state.pending_edge_top = (float)raw_result->edge_y1;
    camera_bridge_filter_state.pending_edge_bottom = (float)raw_result->edge_y2;
    camera_bridge_filter_state.pending_angle_d10 = raw_result->angle_d10;
}

static void camera_bridge_filter_update_pending(
    const CameraBridgeResult_t *raw_result,
    float raw_reference_x)
{
    float count = (float)camera_bridge_filter_state.pending_count;
    float next_count = count + 1.0f;

    camera_bridge_filter_state.pending_slope =
        (camera_bridge_filter_state.pending_slope * count + raw_result->edge_slope) / next_count;
    camera_bridge_filter_state.pending_reference_x =
        (camera_bridge_filter_state.pending_reference_x * count + raw_reference_x) / next_count;
    camera_bridge_filter_state.pending_edge_top =
        (camera_bridge_filter_state.pending_edge_top * count + (float)raw_result->edge_y1) / next_count;
    camera_bridge_filter_state.pending_edge_bottom =
        (camera_bridge_filter_state.pending_edge_bottom * count + (float)raw_result->edge_y2) / next_count;
    camera_bridge_filter_state.pending_angle_d10 = camera_bridge_slope_to_angle_d10(
        camera_bridge_filter_state.pending_slope);
    camera_bridge_filter_state.pending_count++;
}

static void camera_bridge_filter_build_result(
    const CameraBridgeParams_t *params,
    const CameraBridgeResult_t *raw_result,
    CameraBridgeResult_t *filtered_result)
{
    float intercept = camera_bridge_filter_state.reference_x -
                      camera_bridge_filter_state.slope * (float)camera_bridge_filter_state.reference_row;

    *filtered_result = *raw_result;
    filtered_result->reference_row = camera_bridge_filter_state.reference_row;
    filtered_result->edge_slope = camera_bridge_filter_state.slope;
    filtered_result->edge_intercept = intercept;
    filtered_result->right_edge_x = camera_bridge_round_limit(
        camera_bridge_filter_state.reference_x,
        MT9V03X_W);
    filtered_result->distance_px = (int16)(
        (int32)filtered_result->right_edge_x - (int32)params->target_edge_x);
    filtered_result->angle_d10 = camera_bridge_slope_to_angle_d10(
        camera_bridge_filter_state.slope);
    filtered_result->edge_y1 = camera_bridge_round_limit(
        camera_bridge_filter_state.edge_top,
        MT9V03X_H);
    filtered_result->edge_y2 = camera_bridge_round_limit(
        camera_bridge_filter_state.edge_bottom,
        MT9V03X_H);

    if(filtered_result->edge_y1 >= filtered_result->edge_y2)
    {
        filtered_result->edge_y1 = raw_result->edge_y1;
        filtered_result->edge_y2 = raw_result->edge_y2;
    }

    filtered_result->edge_x1 = camera_bridge_round_limit(
        camera_bridge_filter_state.slope * (float)filtered_result->edge_y1 + intercept,
        MT9V03X_W);
    filtered_result->edge_x2 = camera_bridge_round_limit(
        camera_bridge_filter_state.slope * (float)filtered_result->edge_y2 + intercept,
        MT9V03X_W);
    filtered_result->valid = 1;
}

static void camera_bridge_filter_apply(
    const CameraBridgeParams_t *params,
    const CameraBridgeResult_t *raw_result,
    CameraBridgeResult_t *filtered_result)
{
    float raw_reference_x = 0.0f;
    int16 filtered_angle_d10 = 0;
    int32 angle_delta = 0;
    int32 position_delta = 0;
    uint8 pending_consistent = 0;

    if(!raw_result->valid)
    {
        *filtered_result = *raw_result;
        camera_bridge_filter_state.pending_count = 0;

        if(camera_bridge_filter_state.invalid_count < 0xFFu)
        {
            camera_bridge_filter_state.invalid_count++;
        }

        if(camera_bridge_filter_state.invalid_count >= CAMERA_BRIDGE_FILTER_INVALID_RESET_FRAMES)
        {
            camera_bridge_filter_reset();
        }
        return;
    }

    raw_reference_x = raw_result->edge_slope * (float)raw_result->reference_row +
                      raw_result->edge_intercept;

    if((!camera_bridge_filter_state.initialized) ||
       (camera_bridge_filter_state.reference_row != raw_result->reference_row))
    {
        camera_bridge_filter_set_raw(raw_result);
        camera_bridge_filter_build_result(params, raw_result, filtered_result);
        return;
    }

    camera_bridge_filter_state.invalid_count = 0;
    filtered_angle_d10 = camera_bridge_slope_to_angle_d10(camera_bridge_filter_state.slope);
    angle_delta = camera_abs_int32((int32)raw_result->angle_d10 - filtered_angle_d10);
    position_delta = camera_abs_int32(
        (int32)camera_bridge_round_limit(raw_reference_x, MT9V03X_W) -
        (int32)camera_bridge_round_limit(camera_bridge_filter_state.reference_x, MT9V03X_W));

    if((angle_delta > CAMERA_BRIDGE_FILTER_MAX_ANGLE_JUMP_D10) ||
       (position_delta > CAMERA_BRIDGE_FILTER_MAX_POSITION_JUMP_PX))
    {
        if(camera_bridge_filter_state.pending_count > 0u)
        {
            pending_consistent =
                (camera_abs_int32((int32)raw_result->angle_d10 -
                                  camera_bridge_filter_state.pending_angle_d10) <=
                 CAMERA_BRIDGE_FILTER_PENDING_ANGLE_TOLERANCE_D10) &&
                (camera_abs_int32((int32)camera_bridge_round_limit(raw_reference_x, MT9V03X_W) -
                                  (int32)camera_bridge_round_limit(
                                      camera_bridge_filter_state.pending_reference_x,
                                      MT9V03X_W)) <=
                 CAMERA_BRIDGE_FILTER_PENDING_POSITION_TOLERANCE_PX);
        }

        if(pending_consistent)
        {
            camera_bridge_filter_update_pending(raw_result, raw_reference_x);
        }
        else
        {
            camera_bridge_filter_start_pending(raw_result, raw_reference_x);
        }

        if(camera_bridge_filter_state.pending_count >= CAMERA_BRIDGE_FILTER_JUMP_CONFIRM_FRAMES)
        {
            camera_bridge_filter_state.slope = camera_bridge_filter_state.pending_slope;
            camera_bridge_filter_state.reference_x = camera_bridge_filter_state.pending_reference_x;
            camera_bridge_filter_state.edge_top = camera_bridge_filter_state.pending_edge_top;
            camera_bridge_filter_state.edge_bottom = camera_bridge_filter_state.pending_edge_bottom;
            camera_bridge_filter_state.pending_count = 0;
        }
    }
    else
    {
        camera_bridge_filter_state.pending_count = 0;
        camera_bridge_filter_state.slope += CAMERA_BRIDGE_FILTER_ALPHA *
            (raw_result->edge_slope - camera_bridge_filter_state.slope);
        camera_bridge_filter_state.reference_x += CAMERA_BRIDGE_FILTER_ALPHA *
            (raw_reference_x - camera_bridge_filter_state.reference_x);
        camera_bridge_filter_state.edge_top += CAMERA_BRIDGE_FILTER_ALPHA *
            ((float)raw_result->edge_y1 - camera_bridge_filter_state.edge_top);
        camera_bridge_filter_state.edge_bottom += CAMERA_BRIDGE_FILTER_ALPHA *
            ((float)raw_result->edge_y2 - camera_bridge_filter_state.edge_bottom);
    }

    camera_bridge_filter_build_result(params, raw_result, filtered_result);
}

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

static uint8 camera_copy_and_process_frame(uint8 threshold)
{
    if(!mt9v03x_finish_flag)
    {
        return 0;
    }

    mt9v03x_finish_flag = 0;
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    vision_binary_fixed(image_copy, threshold);
    // camera_image_binary_otsu(image_copy);
    // camera_image_filter_isolated_black(image_copy);
    // camera_image_filter_isolated_white(image_copy);

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

        vision_binary_fixed(image_copy, CAMERA_BINARY_THRESHOLD_DEFAULT);
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
    camera_bridge_filter_reset();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    while(mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }
}

void camera_debug_on_screen(void)
{
    uint16 x              = IMAGE_X;
    uint16 y              = IMAGE_Y;
    uint16 display_width  = IMAGE_DISPLAY_WIDTH;
    uint16 display_height = IMAGE_DISPLAY_HEIGHT;

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


void camera_fps_counter_init(fps_counter_t *counter, uint32 time_ms)
{
    if(NULL == counter)
    {
        return;
    }

    counter->last_time_ms = time_ms;
    counter->frame_count = 0;
    counter->fps = 0;
}

uint32 camera_fps_counter_update(fps_counter_t *counter, uint32 time_ms)
{
    if(NULL == counter)
    {
        return 0;
    }

    counter->frame_count++;

    if(time_ms - counter->last_time_ms >= 1000)
    {
        counter->last_time_ms = time_ms;
        counter->fps = counter->frame_count;
        counter->frame_count = 0;
    }

    return counter->fps;
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

void camera_bridge_filter_reset(void)
{
    memset(&camera_bridge_filter_state, 0, sizeof(camera_bridge_filter_state));
}

uint8 camera_bridge_processing(const CameraBridgeParams_t *bridge_params, CameraBridgeResult_t *bridge_result)
{
    CameraBridgeResult_t raw_result = {0};

    if((NULL == bridge_params) || (NULL == bridge_result))
    {
        return 0;
    }

    if(!camera_copy_and_process_frame(bridge_params->binary_threshold))
    {
        return 0;
    }

    (void)camera_image_bridge_detect(image_copy, bridge_params, &raw_result);
    camera_bridge_filter_apply(bridge_params, &raw_result, bridge_result);

    return 1;
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

    if(jump_params->steps >= CAMERA_DOT_TYPE_LIST_COUNT)
    {
        multi_frame_count = 0;
        return 0;
    }

    if(!camera_copy_and_process_frame(CAMERA_BINARY_THRESHOLD_DEFAULT))
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

    if(jump_params->steps >= CAMERA_DOT_TYPE_LIST_COUNT)
    {
        multi_frame_count = 0;
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

        return 1;
    }

    return 0;
}
