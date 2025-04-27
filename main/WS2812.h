#ifndef WS2818_H
#define WS2818_H

#include <stdint.h> 
#include <stdbool.h> 

typedef struct {
    uint8_t r;  // 红色分量
    uint8_t g;  // 绿色分量
    uint8_t b;  // 蓝色分量
} ws2812_color_t;


// 定义闪烁模式的枚举类型
typedef enum {
    WS2812_NO_FLASH,

    WS2812_FLASH_FAST_1 = 1,  // 快闪模式 1
    WS2812_FLASH_SLOW_1 = 2,  // 慢闪模式 1
    WS2812_FLASH_FAST_2 = 3,  // 快闪模式 2
    WS2812_FLASH_SLOW_2 = 4,  // 慢闪模式 2
    WS2812_FLASH_SLOW_3 = 5,  // 慢闪模式 3
    WS2812_FLASH_FAST_3 = 6   // 快闪模式 3
} ws2812_flash_mode_t;


#define WS2812_MODE_ONE     1
#define WS2812_MODE_TWO     2
#define WS2812_MODE_THREE   3

// 函数声明

/**
 * @brief 配置并初始化 RMT 控制器
 */
void ws2812_init(int gpio_num);

/**
 * @brief 设置 LED 颜色并发送到 WS2812
 * 
 * @param color LED 颜色数据
 */
void ws2812_set_colors(ws2812_color_t *led_colors, int led_count);


// 设置ws2812 颜色 返回 是否存在更改
bool ws2812_color_mode(ws2812_color_t *led_color,int8_t mods);

//控制 WS2812 LED 根据当前颜色和模式控制闪烁方式
bool ws2812_flash_mode(ws2812_color_t *led_color, ws2812_flash_mode_t mode);

#endif