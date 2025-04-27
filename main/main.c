// #include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "spp_client_demo.h"
#include "gpio_driver.h"

#include "WS2812.h"
#include "WorkingMode\WorkingMode.h"
#include "nvs_storage.h"
#include "deep_wake_stub.h"
#include "gpio_driver.h"

static const char *TAG = "main"; // 日志 TAG

// 配对键按下保持时间，单位：微秒
#define PAIR_KEY_HOLD_TIME_US (5 * 1000000)

// 设置单个 LED 颜色
ws2812_color_t led_color[1] = {
    {.r = 16, .g = 16, .b = 16}};

const int64_t max_idle_time_us = 30 * 1000000; // 最大空闲时间

// ble 控制ws2812S闪烁方式
void ble_control_ws2812();

// 主程序入口
void app_main(void)
{
    init_nvs();
    // 配置深度睡眠的唤醒方式（使用 GPIO 唤醒）
    configure_deep_sleep_wakeup();
    // gio初始化，在configure_deep_sleep_wakeup后面调用
    gpio_init();
    // 初始化 RMT 控制器
    ws2812_init(10);

    // 控制 LED
    ws2812_set_colors(led_color, 1);

    ble_client_init();

    // 获取设备的蓝牙地址
    const uint8_t *mac_address = ble_get_mac_address();
    // 输出 MAC 地址
    ESP_LOGI(TAG, "设备 MAC 地址: %02x:%02x:%02x:%02x:%02x:%02x",
             mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);

    // BaseType_t xTaskCreate(
    //     Task_keypad_scan, // 任务函数指针
    //     "key_scan",       // 任务名称（用于调试）
    //     1024 * 2,         // 栈大小（单位：words，通常 4 字节）
    //     NULL,             // 传递给任务的参数
    //     1,                // 任务优先级
    //     NULL              // 返回创建的任务句柄（可为 NULL）
    // );

    uint16_t keypad_status = 0;

    // 程序启动时间
    int16_t program_startup_time = esp_timer_get_time();

    int64_t uptime = 0;
    // 保存蓝牙启动状态
    bool ble_startup_status = false;

    int16_t pair_key_ustime = 0;
    int8_t equipmentWorkingMode = 0;
    static bool last_pair_key_pressed = false;
    // 定时检查 GPIO 状态并发送消息
    while (true)
    {
        if (!uptime)
        {
            uptime = esp_timer_get_time(); // 当前运行时间（微秒）
        }

        // 一次获取15个 按键
        keypad_scan_task((void *)&keypad_status);

        // 如果有按键按下 复位空闲
        if (keypad_status)
        {
            uptime = 0;
        }

        printf("->>keypad_status%u\n", keypad_status);

        // 蓝牙连接启动 复位空闲
        if (ble_is_connected() && ble_is_ready_for_read_write())
        {
            ws2812_color_mode(led_color, 0); // 恢复颜色

            uptime = 0;
            general_working_mode(&keypad_status);
            if (!ble_startup_status)
            {
                ble_startup_status = true;
            }
        }
        else if (ble_startup_status)
        {
            ble_startup_status = false;
        }

        // ble 控制ws2812S闪烁方式
        // ble_control_ws2812();

        // 获取模式
        equipmentWorkingMode = get_mode_from_adc();

        bool current_pair_key_pressed = is_pair_key_pressed(); // 低电平 按下

        if (last_pair_key_pressed)
        {
            if (!current_pair_key_pressed) // 只有之前松开到 现在按下才记录时间
            {
                ESP_LOGI(TAG, "检测到配对按下，开始计时");
                pair_key_ustime = esp_timer_get_time();
            }
            uptime = 0;
        }

        if (!current_pair_key_pressed && !last_pair_key_pressed) // 一直按下
        {
            if (pair_key_ustime && (esp_timer_get_time() - pair_key_ustime >= (5 * 1000000)))
            {
                ESP_LOGI(TAG, "配对键长按5秒，断开蓝牙连接");
                ble_client_disconnect();

                if (KEY_IS_PRESSED_BOOL(keypad_status, R1_KEY))
                {
                    ESP_LOGI(TAG, "清空保存的MAC");
                    clear_ble_info_mac();
                }

                pair_key_ustime = 0; // 重置

                while (!is_pair_key_pressed()) //不要一直按下
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
            }
            uptime = 0;
        }
        
        if(current_pair_key_pressed &&last_pair_key_pressed )
        {
            if (pair_key_ustime) // 没有一直按下 == 时间不满足
            {
    
                ESP_LOGI(TAG, "配对键松开，重新扫描蓝牙设备");
                start_scan_again();
    
                pair_key_ustime = 0;
            }
        }


        last_pair_key_pressed = current_pair_key_pressed; // 更新历史状态

        if (!is_charging())
        {
            ESP_LOGI(TAG, "Fully charged");
        }

        // 状态存在切换 复位空闲
        if (ws2812_color_mode(led_color, equipmentWorkingMode))
        {
            uptime = 0;
            printf("存在更新模式 %d\n", equipmentWorkingMode);
        }

        /* 空闲检查
         * uptime 判断,在程序运行每一遍时间中，是否是空闲？
         * 且 (uptime - program_startup_time)
         * 在程序运行时间 - 程序启动时间 >= 设定 空闲时间
         * 则 进入深睡眠
         */
        if (uptime && ((esp_timer_get_time() - uptime) >= max_idle_time_us))
        {
            led_color[0].r = 0;
            led_color[0].g = 0;
            led_color[0].b = 0;
            ws2812_set_colors(led_color, 1);
            printf("运行时间超过 设置 %lld ms，准备进入深睡眠\n", max_idle_time_us);
            enter_deep_sleep();
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ble 控制ws2812S闪烁方式
void ble_control_ws2812()
{
    // 蓝牙未连接
    if (!ble_is_connected() && !ble_is_ready_for_read_write())
    {
        uint8_t a[6];
        // 蓝牙未连接 并且 没有记录mac
        if (load_ble_info_mac(a, 6) == ESP_FAIL)
        {
            // ws2812_flash_mode(led_color,WS2812_FLASH_FAST_2 );
            led_color[0].r = 64;
            led_color[0].g = 0;
            led_color[0].b = 128;
            ws2812_set_colors(led_color, 1);
            return;
        }

        led_color[0].r = 128;
        led_color[0].g = 64;
        led_color[0].b = 0;
        ws2812_set_colors(led_color, 1);
        // ws2812_flash_mode(led_color, WS2812_FLASH_FAST_2);
    }

    // else if(ble_is_ready_for_read_write()) //是准备完全
    // {
    //     ws2812_flash_mode(led_color, );
    // }
    // else //是连接就绪（没有准备发送&&接收）
    // {
    //     ws2812_flash_mode(led_color,WS2812_NO_FLASH);
    // }

    return;
}