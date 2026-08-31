#include "led.h"

typedef struct
{
    uint8_t port;
    uint16_t pin;
} led_cfg_t;

/* 引脚配置表，与枚举一一对应 */
static const led_cfg_t g_led_cfg[LED_MAX] = {
    [LED_RED]     = {GPIO_PORT_A, GPIO_PIN_10},
    [LED_GREEN]   = {GPIO_PORT_C, GPIO_PIN_09},
    [LED_BLUE]    = {GPIO_PORT_C, GPIO_PIN_06},
    [LED_BAT_LOW] = {GPIO_PORT_A, GPIO_PIN_15},
};

/* 电平逻辑自动适配 */
#if LED_COMMON_ANODE
    #define LED_OUTPUT_ON(port, pin)     GPIO_ResetPins(port, pin)
    #define LED_OUTPUT_OFF(port, pin)    GPIO_SetPins(port, pin)
#else
    #define LED_OUTPUT_ON(port, pin)     GPIO_SetPins(port, pin)
    #define LED_OUTPUT_OFF(port, pin)    GPIO_ResetPins(port, pin)
#endif

void led_init(void)
{
    uint8_t i;
    stc_gpio_init_t stcGpioCfg;

    GPIO_StructInit(&stcGpioCfg);
    stcGpioCfg.u16PinAttr = PIN_ATTR_DIGITAL;

    for (i = 0u; i < LED_MAX; i++)
    {
        GPIO_Init(g_led_cfg[i].port, g_led_cfg[i].pin, &stcGpioCfg);
        led_on((led_index_t)i);
				DDL_DelayMS(200);
				led_off((led_index_t)i);
    }
}

void led_on(led_index_t led_idx)
{
    if (led_idx >= LED_MAX) return;
    LED_OUTPUT_ON(g_led_cfg[led_idx].port, g_led_cfg[led_idx].pin);
}

void led_off(led_index_t led_idx)
{
    if (led_idx >= LED_MAX) return;
    LED_OUTPUT_OFF(g_led_cfg[led_idx].port, g_led_cfg[led_idx].pin);
}

void led_toggle(led_index_t led_idx)
{
    if (led_idx >= LED_MAX) return;
    GPIO_TogglePins(g_led_cfg[led_idx].port, g_led_cfg[led_idx].pin);
}

void led_all_off(void)
{
    uint8_t i;
    for (i = 0u; i < LED_MAX; i++) led_off((led_index_t)i);
}

void led_all_on(void)
{
    uint8_t i;
    for (i = 0u; i < LED_MAX; i++) led_on((led_index_t)i);
}




