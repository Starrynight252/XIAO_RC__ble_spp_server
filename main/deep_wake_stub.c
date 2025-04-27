#include <stdio.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

#define WAKEUP_GPIO_NUM (1ULL << 0) | (1ULL << 1) | (1ULL << 2) | (1ULL << 3)
#define WAKEUP_GPIO_LEVEL ESP_GPIO_WAKEUP_GPIO_LOW // 低电平唤醒

// 配置深度睡眠的唤醒方式（使用 GPIO 唤醒）
void configure_deep_sleep_wakeup(void)
{
    // 配置唤醒引脚为输入模式
    gpio_config_t io_conf = {
        .pin_bit_mask = WAKEUP_GPIO_NUM,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&io_conf);
    // 启用 GPIO 唤醒功能（指定引脚和电平）
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(WAKEUP_GPIO_NUM, WAKEUP_GPIO_LEVEL));
}

// 进入睡眠模式
void enter_deep_sleep(void)
{
    // 配置唤醒引脚为输入模式
    gpio_config_t io_conf = {
        .pin_bit_mask = WAKEUP_GPIO_NUM,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&io_conf);

    // 启动睡眠
    esp_deep_sleep_start();
}
