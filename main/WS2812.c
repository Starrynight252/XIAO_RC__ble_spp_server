#include "ws2812.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h" // taskENTER_CRITICAL/taskEXIT_CRITICAL

#include "WS2812.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define T1H 800 // ns, high for "1"
#define T1L 450 // ns, low for "1"
#define T0H 400 // ns, high for "0"
#define T0L 850 // ns, low for "0"

static int ws2812_pin;
// 亮度
static int8_t brightness_percent = 50;
// static inline void delay_ns(uint32_t ns) {
//     // ESP32-C2 固定 160MHz，即每个 cycle = 6.25ns
//     // 所以 1ns = 0.16 cycle，约等于 cycles = ns / 6.25
//     uint32_t cycles = (ns / 5);  // 更快整数运算近似值
//     for (uint32_t i = 0; i < cycles; ++i) {
//         __asm__ __volatile__("nop");
//     }
// }

#define NOP() asm volatile("nop" ::: "memory")

IRAM_ATTR static inline void delay_exact_400ns()
{
    NOP();
    NOP();
    NOP();
    NOP();
    NOP(); // 5
    NOP();
}
IRAM_ATTR static inline void delay_exact_850ns()
{
    NOP();
    NOP();
    NOP();
    NOP();
    NOP(); // 5
    NOP();
}

static inline void delay_ns(uint32_t ns)
{
    uint32_t cycles = (ns * 120) / 1000; // = ns * (MHz) / 1000
    for (uint32_t i = 0; i < cycles; ++i)
    {
        __asm__ __volatile__("nop");
    }
}

IRAM_ATTR static inline void send_bit(bool bit_val)
{
    if (bit_val)
    {
        gpio_set_level(ws2812_pin, 1);
        delay_exact_850ns();
        gpio_set_level(ws2812_pin, 0);
        delay_exact_400ns();
    }
    else
    {
        gpio_set_level(ws2812_pin, 1);
        // delay_exact_400ns();
        delay_ns(5); // 6
        gpio_set_level(ws2812_pin, 0);
        // delay_exact_850ns();
        delay_ns(7); // 8
    }
}

/**
 * @brief 发送一个字节（高位在前）
 */
IRAM_ATTR static inline void send_byte(uint8_t byte)
{

    for (int i = 7; i >= 0; --i)
    {
        send_bit((byte >> i) & 1);
    }
}

void ws2812_init(int pin)
{
    ws2812_pin = pin;
    // gpio_reset_pin(pin);
    // gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    // gpio_set_level(pin, 0);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pin); // 选择指定 GPIO 引脚
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // 上拉电阻
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 下拉电阻
    gpio_config(&io_conf);
}

void ws2812_set_colors(ws2812_color_t *led_colors, int led_count)
{
    int i = 0;
    ;
    // portENTER_CRITICAL(&mux);

    // for (int i = 0; i < led_count; ++i) {
    send_byte(led_colors[i].g);
    send_byte(led_colors[i].r);
    send_byte(led_colors[i].b);
    //}
    // portEXIT_CRITICAL(&mux);

    // esp_rom_delay_us(1000);
}

// /**
//  * @brief 在不改变颜色的前提下调整亮度
//  * @param color 原始颜色
//  * @return 新的颜色（已调整亮度）
//  */
// ws2812_color_t adjust_brightness(ws2812_color_t color)
// {

//     ws2812_color_t result;
//     result.r = (color.r * brightness_percent) / 100;
//     result.g = (color.g * brightness_percent) / 100;
//     result.b = (color.b * brightness_percent) / 100;
//     return result;
// }

// 设置ws2812颜色，返回是否存在更改
bool ws2812_color_mode(ws2812_color_t *led_color, int8_t mods)
{
    static int8_t equipmentWorkingMode_bit = 0; // 保存上一次模式
    static ws2812_color_t saved_color = {0};    // 保存上一次的颜色

    int8_t equipment_working_mode = mods;

    if (led_color == NULL)
    {
        return false;
    }

    if (equipment_working_mode == 0)  // 传入0，恢复保存的颜色
    {
        *led_color = saved_color;
        ws2812_set_colors(led_color, 1);
        return true;
    }

    // 模式有变化才切换
    if (equipmentWorkingMode_bit != equipment_working_mode)
    {
        // 先保存当前颜色
        saved_color = *led_color;

        // 根据模式设置LED颜色
        switch (equipment_working_mode)
        {
        case WS2812_MODE_ONE:
            led_color[0].r = 8;
            led_color[0].g = 0;
            led_color[0].b = 0;
            break;
        case WS2812_MODE_TWO:
            led_color[0].r = 0;
            led_color[0].g = 8;
            led_color[0].b = 0;
            break;
        case WS2812_MODE_THREE:
            led_color[0].r = 0;
            led_color[0].g = 0;
            led_color[0].b = 8;
            break;
        default:
            return false; // 无效模式
        }

        // 更新LED
        ws2812_set_colors(led_color, 1);

        // 保存当前新的模式
        equipmentWorkingMode_bit = equipment_working_mode;

        return true; // 有修改
    }

    return false; // 无变化
}

// 该函数根据传入的颜色和模式，控制 WS2812 LED 的颜色和闪烁方式。模式不同会有不同的闪烁效果
bool ws2812_flash_mode(ws2812_color_t *led_color, ws2812_flash_mode_t mode)
{
    ws2812_color_t led_sho;

    if (led_color == NULL)
    {
        return false;
    }

    // 记录之前颜色
    led_sho = led_color[0];

    // 关闭所有 颜色
    led_color[0].r = led_color[0].g = led_color[0].b = 0;

    // 控制闪烁方式，根据模式
    switch (mode)
    {
    case WS2812_NO_FLASH: // 取消闪烁
        break;
    case WS2812_FLASH_FAST_1:
        // 快闪模式 1：修改 LED 颜色和闪烁方式
        break;

    case WS2812_FLASH_SLOW_1:
        // 慢闪模式 1
        break;

    case WS2812_FLASH_FAST_2:
        // 快闪模式 2
        delay_ns(10000000);
        ws2812_set_colors(led_color, 1);
        delay_ns(10000000);
        break;

    case WS2812_FLASH_SLOW_2:
        // 慢闪模式 2
        break;

    case WS2812_FLASH_SLOW_3:
        // 慢闪模式 3
        break;

    case WS2812_FLASH_FAST_3:
        // 快闪模式 3
        break;

    default:
        led_color[0] = led_sho;
        // 处理未知模式
        return false;
    }

    led_color[0] = led_sho;
    // 根据闪烁模式更新 LED 状态
    ws2812_set_colors(led_color, 1); // 这里的 `ws2812_set_colors` 函数需要根据你的实际控制实现

    return true;
}
