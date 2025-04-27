#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_rom_sys.h"
#include "driver/rmt.h"

#include "gpio_driver.h"

// 行和列 GPIO
#define ROWS 3
#define COLS 5

// 充满检查
#define STDBY_PIN GPIO_NUM_18
// 配对按键
#define PAIR_KEY_PIN GPIO_NUM_9

// ADC通道选择（根据你的硬件选择）
#define ADC_WIDTH ADC_WIDTH_BIT_12 // ADC分辨率设置为12位
#define ADC_SAMPLES 64             // 每次采样的样本数
#define ADC_CHANNEL ADC1_CHANNEL_0 // 选择 ADC1 的通道0，

const gpio_num_t row_pins[ROWS] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3};
const gpio_num_t col_pins[COLS] = {GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8};

// 矩阵映射（3行 × 5列）
KeyID key_map[ROWS][COLS] = {
    // 第1行：GIO1 接行，依次为列 GIO4 ~ GIO8
    {L2_KEY, L1_KEY, R1_KEY, R2_KEY, SELECT_KEY},

    // 第2行：GIO2 接行，依次为列 GIO4 ~ GIO8
    {UP_KEY, LEFT_KEY, A_KEY, B_KEY, HOME_KEY},

    // 第3行：GIO3 接行，依次为列 GIO4 ~ GIO8
    {RIGHT_KEY, DOWN_KEY, Y_KEY, X_KEY, START_KEY}};

// 通用初始化 GPIO -内部函数
static void init_gpio(uint32_t gpio_pin)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << gpio_pin);   // 选择指定 GPIO 引脚
    io_conf.mode = GPIO_MODE_INPUT;              // 设置为输入模式
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;    // 不使能上拉电阻
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // 使能下拉电阻
    gpio_config(&io_conf);
}
// 初始化ADC
static void adc_init(void)
{
    // 配置ADC宽度
    adc1_config_width(ADC_WIDTH);

    // 配置ADC通道
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11); // 选择11dB衰减
                                                             // ESP_LOGI(TAG, "ADC 初始化完成，分辨率：%d 位，通道：%d", ADC_WIDTH, ADC_CHANNEL);
}

// 初始化 PIN
void gpio_init(void)
{
    adc_init();

    /* 充满检查 */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << STDBY_PIN);   // 选择指定 GPIO 引脚
    io_conf.mode = GPIO_MODE_INPUT;               // 设置为输入模式
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // 使能上拉电阻
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 不使能下拉电阻
    gpio_config(&io_conf);

    // 设置行输出
    for (int i = 0; i < ROWS; i++)
    {
        gpio_reset_pin(row_pins[i]);
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(row_pins[i], 1); // 默认拉高
    }

    // 设置列为输入，开启上拉
    for (int j = 0; j < COLS; j++)
    {
        gpio_reset_pin(col_pins[j]);
        gpio_set_direction(col_pins[j], GPIO_MODE_INPUT);
        gpio_set_pull_mode(col_pins[j], GPIO_PULLUP_ONLY);
    }
}

// 扫描
KeyID keypad_scan(void)
{
    for (int i = 0; i < ROWS; i++)
    {
        // 逐行变化 需要扫描的行拉低，其他行拉高
        for (int r = 0; r < ROWS; r++)
        {
            gpio_set_level(row_pins[r], r == i ? 0 : 1);
        }
        esp_rom_delay_us(5); // 延迟稳定信号

        // 读取列 -是否为低
        for (int j = 0; j < COLS; j++)
        {
            if (gpio_get_level(col_pins[j]) == 0)
            {
                // 为低则按键按下-返回映射
                return key_map[i][j];
            }
        }
    }

    return KEY_NONE;
}

// 保存按键数值的变量
uint16_t typeKeypad_data;


/*
 * 扫描按键并放在结构体中间
 * 推荐放在FreeRTOS任务中扫描
 * 参数 uint16_t* 用于保存按键结果
 */
void keypad_scan_task(void *freertos_p)
{
    // 如果传入的参数非 NULL，保存扫描结果
    if (freertos_p != NULL)
    {
        //复位
        *(uint16_t *)freertos_p = 0;

        // 扫描15个按键的矩阵
        for (int i = 0; i < ROWS; i++)
        {
            // 逐行变化，设置扫描的行拉低，其他行拉高
            for (int r = 0; r < ROWS; r++)
            {
                gpio_set_level(row_pins[r], r == i ? 0 : 1);
            }
            esp_rom_delay_us(5); // 延迟稳定信号

            // 读取列 - 是否为低
            for (int j = 0; j < COLS; j++)
            {
                if (gpio_get_level(col_pins[j]) == 0)
                {
                    // 为低则按键按下-标记按键状态 置位
                    *(uint16_t *)freertos_p |= (1 << key_map[i][j]); // 置位

                    //printf("->>>>freertos_p%u\n",*(uint16_t *)freertos_p);
                }
            }
        }
    }
}

/*充电状态
 * 无参数，返回bool false 充满, true 未充满
 */
bool is_charging(void)
{
    return (gpio_get_level(STDBY_PIN));
}

/*获取是否按下Pair 按键
 * 无参数，返回bool false 按下, true 松开
 */
bool is_pair_key_pressed(void)
{
    return (gpio_get_level(PAIR_KEY_PIN));
}

// 检查滑动开关状态并返回当前模式
int get_mode_from_adc(void)
{
    // 获取 ADC 通道的原始值
    int adc_value = adc1_get_raw(ADC_CHANNEL);

    // 判断 ADC 值来确定当前模式
    if (adc_value == 0)
    {
        return 1; // 模式 1
    }
    else if (adc_value > 2300 && adc_value < 2380) // 误差范围
    {
        return 3; // 模式 3
    }
    else if (adc_value >= 3300)
    {
        return 2; // 模式 2
    }

    return -1; // 防止意外返回，表示无效模式
}
