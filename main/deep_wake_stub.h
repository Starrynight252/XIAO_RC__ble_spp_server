#ifndef DEEP_WAKE_STUB_H
#define DEEP_WAKE_STUB_H


// 配置深度睡眠的唤醒方式（使用 GPIO 唤醒）
void configure_deep_sleep_wakeup(void);
// 进入深度睡眠模式
void enter_deep_sleep(void);

#endif // DEEP_WAKE_STUB_H