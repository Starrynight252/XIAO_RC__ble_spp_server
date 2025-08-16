#ifndef BLE_SPP_SERVER_DEMO_H
#define BLE_SPP_SERVER_DEMO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_err.h"

// 宏定义
#define DEVICE_NAME "BLE_SIMPLE_SERVER" // BLE 设备名称


/* 函数声明 */
// 初始化 BLE 服务器
esp_err_t ble_server_init(void);

// 获取 BLE 是否已连接
bool ble_is_connected(void);
// 获取 BLE 的 MAC 地址
const uint8_t *ble_get_mac_address(void);
// 设置本地 BLE MTU
esp_err_t ble_set_local_mtu(uint16_t mtu);


/* 发送字符串数据通过 BLE
* 返回值表示是否发送成功（仅在已连接状态下尝试发送）
*/
bool ble_send_string_data(const char *message);
/* 发送 int 数组数据（通过 BLE 通知）
* 返回值表示是否发送成功（仅在已连接状态下尝试发送）
*/
bool ble_send_int_array(int *array, size_t size);


#endif // BLE_SPP_SERVER_DEMO_H