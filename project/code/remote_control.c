#include "remote_control.h"
#include "zf_common_headfile.h"

static uint16 remote_channel[REMOTE_CONTROL_CHANNEL_NUM];
static uint8 remote_online = 0;

float yaw_target = 0.0f;     // 目标航向角


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


static int16 remote_control_apply_dead_zone(int16 data, int16 dead_zone)
{
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


int16 remote_front_rear_ctrl(int16 base, int16 max, int16 min, int16 dead_zone)
{
    int16 raw_data_to_zero = (int16)(remote_get_channel(REMOTE_CONTROL_FRONT_REAR_CH)) - REMOTE_CONTROL_CENTER_RAW;
    float coeff = (float)(max - min) / 1600.0f;
    float output_data;

    raw_data_to_zero = remote_control_apply_dead_zone(raw_data_to_zero, dead_zone);
    output_data = (float)base + (float)raw_data_to_zero * coeff;

    return (int16)output_data;
}

int16 remote_left_right_ctrl(int16 base, int16 max, int16 min, int16 dead_zone)
{
    int16 raw_data_to_zero = (int16)(remote_get_channel(REMOTE_CONTROL_LEFT_RIGHT_CH)) - REMOTE_CONTROL_CENTER_RAW;
    float coeff = (float)(max - min) / 1600.0f;
    float output_data;

    raw_data_to_zero = remote_control_apply_dead_zone(raw_data_to_zero, dead_zone);
    output_data = (float)base + (float)raw_data_to_zero * coeff;

    return (int16)output_data;
}


int8 remote_left_01_switch_ctrl(void)
{
    uint16 raw_data = remote_get_channel(REMOTE_CONTROL_LEFT_01_SWITCH_CH);

    if(raw_data < REMOTE_CONTROL_SWITCH_LOW_RAW)
    {
        return 1;
    }
    else if(raw_data > REMOTE_CONTROL_SWITCH_HIGH_RAW)
    {
        return -1;
    }

    return 0;
}


void remote_left_02_switch_ctrl(void)
{
    uint16 raw_data = remote_get_channel(REMOTE_CONTROL_LEFT_02_SWITCH_CH);

    if(raw_data < 1500) protect_flag = 0;
    else  protect_flag = 1;
}
