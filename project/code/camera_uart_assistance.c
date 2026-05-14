#include "camera_uart_assistance.h"

#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_wireless_uart.h"

#define CAMERA_UART_ASSISTANCE_CHECK_ROW_CHANNEL          (0)
#define CAMERA_UART_ASSISTANCE_CHECK_ROW_COUNT_CHANNEL    (1)
#define CAMERA_UART_ASSISTANCE_BLACK_COUNT_CHANNEL        (2)

#define CAMERA_UART_ASSISTANCE_CHECK_ROW_UPDATED          (1 << 0)
#define CAMERA_UART_ASSISTANCE_CHECK_ROW_COUNT_UPDATED    (1 << 1)
#define CAMERA_UART_ASSISTANCE_BLACK_COUNT_UPDATED        (1 << 2)

static void camera_uart_assistance_limit_jump_param(uint16 *check_row, uint16 *check_row_count, uint16 *black_count)
{
    if(MT9V03X_H <= *check_row)
    {
        *check_row = MT9V03X_H - 1;
    }

    if(0 == *check_row_count)
    {
        *check_row_count = 1;
    }

    if((*check_row + 1) < *check_row_count)
    {
        *check_row_count = *check_row + 1;
    }

    if(MT9V03X_W < *black_count)
    {
        *black_count = MT9V03X_W;
    }
}

void camera_uart_assistance_init(void)
{
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
}

uint8 camera_uart_assistance_update_jump_param(uint16 *check_row, uint16 *check_row_count, uint16 *black_count)
{
    uint8 update_state = 0;

    if((0 == check_row) || (0 == check_row_count) || (0 == black_count))
    {
        return 0;
    }

    seekfree_assistant_data_analysis();

    if(seekfree_assistant_parameter_update_flag[CAMERA_UART_ASSISTANCE_CHECK_ROW_CHANNEL])
    {
        seekfree_assistant_parameter_update_flag[CAMERA_UART_ASSISTANCE_CHECK_ROW_CHANNEL] = 0;
        *check_row = (uint16)seekfree_assistant_parameter[CAMERA_UART_ASSISTANCE_CHECK_ROW_CHANNEL];
        update_state |= CAMERA_UART_ASSISTANCE_CHECK_ROW_UPDATED;
    }

    if(seekfree_assistant_parameter_update_flag[CAMERA_UART_ASSISTANCE_CHECK_ROW_COUNT_CHANNEL])
    {
        seekfree_assistant_parameter_update_flag[CAMERA_UART_ASSISTANCE_CHECK_ROW_COUNT_CHANNEL] = 0;
        *check_row_count = (uint16)seekfree_assistant_parameter[CAMERA_UART_ASSISTANCE_CHECK_ROW_COUNT_CHANNEL];
        update_state |= CAMERA_UART_ASSISTANCE_CHECK_ROW_COUNT_UPDATED;
    }

    if(seekfree_assistant_parameter_update_flag[CAMERA_UART_ASSISTANCE_BLACK_COUNT_CHANNEL])
    {
        seekfree_assistant_parameter_update_flag[CAMERA_UART_ASSISTANCE_BLACK_COUNT_CHANNEL] = 0;
        *black_count = (uint16)seekfree_assistant_parameter[CAMERA_UART_ASSISTANCE_BLACK_COUNT_CHANNEL];
        update_state |= CAMERA_UART_ASSISTANCE_BLACK_COUNT_UPDATED;
    }

    camera_uart_assistance_limit_jump_param(check_row, check_row_count, black_count);

    return update_state;
}
