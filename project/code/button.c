#include "button.h"

// 按键释放时为高电平，按下时为低电平。
#define BUTTON_RELEASE_LEVEL        (GPIO_HIGH)

// 按键引脚表，顺序需要和 button_index_enum 保持一致。
static const gpio_pin_enum button_pin[BUTTON_NUMBER] =
{
    BUTTON_KEY1_PIN,
    BUTTON_KEY2_PIN,
    BUTTON_KEY3_PIN,
    BUTTON_KEY4_PIN,
};

// 记录上一次 update 时每个按键是否处于按下状态，用于判断按下边沿。
static uint8 button_last_pressed[BUTTON_NUMBER] = {0};

// 对外提供的单次按下标志位。
uint8 button_flag[BUTTON_NUMBER] = {0};

void button_init(void)
{
    uint8 i;

    for(i = 0; i < BUTTON_NUMBER; i++)
    {
        // 上拉输入：未按下为高电平，按下为低电平。
        gpio_init(button_pin[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        button_last_pressed[i] = 0;
        button_flag[i] = 0;
    }
}

void button_update(void)
{
    uint8 i;

    for(i = 0; i < BUTTON_NUMBER; i++)
    {
        uint8 now_pressed;

        // 每次 update 先清零，保证标志位只保持一个调用周期。
        button_flag[i] = 0;

        // 当前电平不等于释放电平，说明按键正在被按下。
        now_pressed = (BUTTON_RELEASE_LEVEL != gpio_get_level(button_pin[i]));

        // 只有从“未按下”变成“按下”的这一刻置 1，长按不会重复触发。
        if(now_pressed && !button_last_pressed[i])
        {
            button_flag[i] = 1;
        }

        button_last_pressed[i] = now_pressed;
    }
}
