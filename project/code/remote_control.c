#include "remote_control.h"
#include "zf_common_headfile.h"

#define REMOTE_CONTROL_OUTPUT_MAX         (1792)
#define REMOTE_CONTROL_OUTPUT_MIN         (192)
#define REMOTE_CONTROL_DEAD_ZONE          (30)

static uint16 remote_channel[REMOTE_CONTROL_CHANNEL_NUM];
static uint8 remote_online = 0;

void remote_control_init(void)
{
    uart_receiver_init();
}

void remote_update(void)
{
    if(uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;

        remote_online = uart_receiver.state;

        for(uint8 i = 0; i < REMOTE_CONTROL_CHANNEL_NUM; i++)
        {
            remote_channel[i] = uart_receiver.channel[i];
        }
    }
}

uint16 remote_get_channel(uint8 ch)
{
    if(ch >= REMOTE_CONTROL_CHANNEL_NUM)
    {
        return 0;
    }

    return remote_channel[ch];
}

uint8 remote_is_online(void)
{
    return remote_online;
}

static int16 remote_control_apply_dead_zone(int16 data)
{
    int16 dead_zone = REMOTE_CONTROL_DEAD_ZONE;

    if(dead_zone < 0)
    {
        dead_zone = -dead_zone;
    }

    if(data >= -dead_zone && data <= dead_zone)
    {
        data = 0;
    }

    return data;
}

static int16 remote_control_calc_channel(uint8 channel)
{
    int16 raw_data_to_zero = (int16)(remote_get_channel(channel)) - REMOTE_CONTROL_CENTER_RAW;
    float coeff = (float)(REMOTE_CONTROL_OUTPUT_MAX - REMOTE_CONTROL_OUTPUT_MIN) / 1600.0f; //目前倍率为1
    float output_data;

    raw_data_to_zero = remote_control_apply_dead_zone(raw_data_to_zero);
    output_data = (float)raw_data_to_zero * coeff;

    return (int16)output_data;
}

int16 remote_front_rear_ctrl(void)
{
    return remote_control_calc_channel(REMOTE_CONTROL_FRONT_REAR_CH);
}

int16 remote_left_right_ctrl(void)
{
    return remote_control_calc_channel(REMOTE_CONTROL_LEFT_RIGHT_CH);
}

void remote_left_01_switch_ctrl(void)
{
    uint16 raw_data = remote_get_channel(REMOTE_CONTROL_LEFT_01_SWITCH_CH);
}

void remote_left_02_switch_ctrl(void)
{
    uint16 raw_data = remote_get_channel(REMOTE_CONTROL_LEFT_02_SWITCH_CH);

    if(raw_data < REMOTE_CONTROL_CENTER_RAW) protect_flag = 0;
    else  protect_flag = 1;
}

void remote_right_01_switch_ctrl(void)
{
    uint16 raw_data = remote_get_channel(REMOTE_CONTROL_RIGHT_01_SWITCH_CH);
}

void remote_right_02_switch_ctrl(void)
{
    static uint16 last_raw = 0;
    static uint8 has_last_raw = 0;
    uint16 raw_data = remote_get_channel(REMOTE_CONTROL_RIGHT_02_SWITCH_CH);

    if(has_last_raw && raw_data != last_raw)
    {
        jump_flag = 1;
    }

    last_raw = raw_data;
    has_last_raw = 1;
}
