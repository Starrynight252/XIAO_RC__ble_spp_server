#ifndef SPP_CLIENT_DEMO_H
#define SPP_CLIENT_DEMO_H

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
// 初始化 BLE
esp_err_t ble_client_init(void);

//检查蓝牙是否准备好进行读写操作
bool ble_is_ready_for_read_write();
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

//设置扫描时间
void set_ble_scan_duration(uint16_t seconds);
//再一次扫描
void start_scan_again();

// 停止扫描 并且 主动断开连接
void ble_client_disconnect(void);


#endif // SPP_CLIENT_DEMO_H