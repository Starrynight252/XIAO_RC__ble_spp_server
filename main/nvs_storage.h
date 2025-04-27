#ifndef NVS_STORTAGE_H
#define NVS_STORTAGE_H

#include "esp_err.h"


// 初始化 NVS
esp_err_t init_nvs();


// 保存 BLE MAC 地址
esp_err_t save_ble_info_mac(uint8_t *mac);
// 读取 BLE MAC 地址
esp_err_t load_ble_info_mac(uint8_t *mac,size_t name_size);
//清空mac  填入 0xff
esp_err_t clear_ble_info_mac();


// 保存 BLE 名称和 MAC 地址
esp_err_t save_ble_info_mac_num(uint8_t *mac, const char *name);
// 读取 BLE 名称和 MAC 地址
esp_err_t load_ble_info_mac_num(uint8_t *mac, char *name, size_t name_size);
#endif