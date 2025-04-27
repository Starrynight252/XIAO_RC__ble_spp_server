#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdbool.h>

// 定义键值
typedef enum
{
    KEY_NONE = 0,

    /* Gio 1 行 */
    L2_KEY,  //Ltrigger扳机
    L1_KEY, //Lshoulder肩键
    R1_KEY, //Rshoulder肩键
    R2_KEY,  //Rtrigger扳机
    SELECT_KEY,
    
    /* Gio 2 行*/
    UP_KEY,
    LEFT_KEY,
    A_KEY,
    B_KEY,
    HOME_KEY,

    /* Gio 3 行*/
    RIGHT_KEY,
    DOWN_KEY,
    Y_KEY,
    X_KEY,
    START_KEY //15

} KeyID;


//按键状态检查 范围KeyID
#define KEY_IS_PRESSED(keypad_status, bit)   ((keypad_status) & (1 << (bit)))
#define KEY_IS_PRESSED_BOOL(keypad_status, bit)   (bool)((keypad_status) & (1 << (bit)))

//初始化PIN
void gpio_init(void);

//扫描
KeyID keypad_scan(void);

/*
* 扫描按键并放在结构体中间
* 推荐放在freeFtos 任务中扫描
* 参数 uint16_t 保存按键结果
*/
void keypad_scan_task(void *freertos_p);

/*充电状态
* 无参数，返回bool true 充满, false 未充满
*/
bool is_charging(void);

/*获取是否按下配对按键
* 无参数，返回bool false 按下, true 松开
*/
bool is_pair_key_pressed(void);


// 检查滑动开关状态并返回当前模式
int get_mode_from_adc(void);



#endif