#include "nvs_storage.h"
#include "nvs_flash.h"
#include "nvs.h"

// 宏定义
#define NVS_STORAGE_NAMESPACE "storage" // NVS 存储空间的命名空间
#define NVS_BLE_MAC_KEY "ble_mac"       // NVS 存储的 BLE MAC 地址的键
#define NVS_BLE_NAME_KEY "ble_name"     // NVS 存储的 BLE 名称的键

static bool is_cleared = false; // 静态变量记录是否已经调用过清空操作

// 初始化 NVS
esp_err_t init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase()); // 如果存储区没有空余页或版本不一致则擦除
        err = nvs_flash_init();             // 重新初始化
    }
    return err;
}

// 保存 BLE MAC 地址
esp_err_t save_ble_info_mac(uint8_t *mac)
{

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs); // 打开存储空间
    if (err != ESP_OK)
        return err;

    // 保存 MAC 地址
    err = nvs_set_blob(nvs, NVS_BLE_MAC_KEY, mac, 6);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }

    nvs_commit(nvs); // 提交数据
    nvs_close(nvs);
    is_cleared = false; // 清空
    return ESP_OK;
}

// 读取 BLE MAC 地址
esp_err_t load_ble_info_mac(uint8_t *mac, size_t name_size)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READONLY, &nvs); // 打开存储空间
    if (err != ESP_OK)
        return err;

    // 读取 MAC 地址
    size_t len = 6;
    err = nvs_get_blob(nvs, NVS_BLE_MAC_KEY, mac, &len);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }
    // 用于判断是否是清空的数据

    int e = 0;
    for (int i = 0; i < name_size; i++)
    {
        if (mac[i] == 0xff)
            e++;
    }

    if (e >= name_size)
    {
        return ESP_FAIL;
    }

    nvs_close(nvs);
    return ESP_OK;
}

// 清空mac  填入 0xff
esp_err_t clear_ble_info_mac()
{
    //   if (!is_cleared)

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs); // 打开存储空间
    if (err != ESP_OK)
        return err;

    // 用 0xff 填充清空 MAC 地址区域
    uint8_t mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    err = nvs_set_blob(nvs, NVS_BLE_MAC_KEY, mac, 6); // 清空数据并用 0xff 填充
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }

    nvs_commit(nvs); // 提交更改
    nvs_close(nvs);

    is_cleared = true;

    return ESP_OK;
}

// 保存 BLE 名称和 MAC 地址
esp_err_t save_ble_info_mac_num(uint8_t *mac, const char *name)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs); // 打开存储空间
    if (err != ESP_OK)
        return err;

    // 保存 MAC 地址
    err = nvs_set_blob(nvs, NVS_BLE_MAC_KEY, mac, 6);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }

    // 保存蓝牙名称
    err = nvs_set_str(nvs, NVS_BLE_NAME_KEY, name);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }

    nvs_commit(nvs); // 提交数据
    nvs_close(nvs);
    is_cleared = false; // 清空
    return ESP_OK;
}

// 读取 BLE 名称和 MAC 地址
esp_err_t load_ble_info_mac_num(uint8_t *mac, char *name, size_t name_size)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READONLY, &nvs); // 打开存储空间
    if (err != ESP_OK)
        return err;

    // 读取 MAC 地址
    size_t len = 6;
    err = nvs_get_blob(nvs, NVS_BLE_MAC_KEY, mac, &len);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }

    // 读取蓝牙名称
    size_t name_len = name_size;
    err = nvs_get_str(nvs, NVS_BLE_NAME_KEY, name, &name_len);
    if (err != ESP_OK)
    {
        nvs_close(nvs);
        return err;
    }

    nvs_close(nvs);
    return ESP_OK;
}
