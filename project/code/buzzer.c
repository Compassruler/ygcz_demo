#include "buzzer.h"

#define BUZZER_PIN  (P19_4)

/* 蜂鸣器初始化 */
void buzzer_init(void)
{
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
}

/* 蜂鸣器打开 */
void buzzer_on(void)
{
    gpio_set_level(BUZZER_PIN, GPIO_HIGH);
}

/* 蜂鸣器关闭 */
void buzzer_off(void)
{
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
}

/* 蜂鸣器翻转 */
void buzzer_toggle(void)
{
    gpio_toggle_level(BUZZER_PIN);
}

/* 蜂鸣器调试响声 */
void buzzer_beep(uint16 times, uint16 interval_ms)
{
    uint16 i;

    for(i = 0; i < times; i++)
    {
        buzzer_on();
        system_delay_ms(interval_ms);

        buzzer_off();
        system_delay_ms(interval_ms);
    }
}