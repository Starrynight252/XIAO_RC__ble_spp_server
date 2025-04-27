#ifndef WORKING_MODE_H
#define WORKING_MODE_H

#include "spp_client_demo.h"
#include "gpio_driver.h"

/*通用工作模式
* 参数： 发送的按键数据，必须为uint16_t
*/
void general_working_mode(uint16_t *keypad_status);
#endif // WORKING_MODE_H