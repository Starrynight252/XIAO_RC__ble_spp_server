/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * 本文件是 BLE SPP 客户端示例。
 * 它实现了 BLE 客户端的基本功能，包括扫描设备、连接服务、读取和写入特征值等。
 *
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/uart.h"

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "spp_client_demo.h"
#include "nvs_storage.h"

// 日志标签
#define GATTC_TAG "GATTC_SPP_DEMO"
// 配置的 GATT 客户端配置
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0
#define BT_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#define ESP_GATT_SPP_SERVICE_UUID 0xABF0 // 自定义 SPP 服务 UUID
#define INVALID_HANDLE 0

enum
{
    SPP_IDX_SVC, // 服务索引

    SPP_IDX_SPP_DATA_RECV_VAL, // 数据接收特征值索引

    SPP_IDX_SPP_DATA_NTY_VAL, // 数据通知特征值索引
    SPP_IDX_SPP_DATA_NTF_CFG, // 数据通知配置描述符索引

    SPP_IDX_SPP_COMMAND_VAL, // 命令特征值索引

    SPP_IDX_SPP_STATUS_VAL, // 状态特征值索引
    SPP_IDX_SPP_STATUS_CFG, // 状态配置描述符索引

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_VAL, // 心跳特征值索引
    SPP_IDX_SPP_HEARTBEAT_CFG, // 心跳配置描述符索引
#endif

    SPP_IDX_NB, // 数据库中元素的总数
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
// GATT 客户端事件处理函数
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

// GAP 事件处理函数
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// 服务 UUID: 0000fff0-0000-1000-8000-00805f9b34fb
static uint8_t service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f,
    0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00,
    0xf0, 0xff, 0x00, 0x00};

// 特征 UUID: 0000fff1-0000-1000-8000-00805f9b34fb
static uint8_t service_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f,
    0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00,
    0xf1, 0xff, 0x00, 0x00};

// GATT 客户端配置结构体
struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;       // GATT 客户端回调函数
    uint16_t gattc_if;             // GATT 接口
    uint16_t app_id;               // 应用 ID
    uint16_t conn_id;              // 连接 ID
    uint16_t service_start_handle; // 服务起始句柄
    uint16_t service_end_handle;   // 服务结束句柄
    uint16_t char_handle;          // 特征值句柄
    esp_bd_addr_t remote_bda;      // 远程设备地址
};

// GATT 客户端配置
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE, // 初始化为未定义接口
    },
};

// BLE 扫描参数配置
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,               // 主动扫描
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,           // 使用公共地址
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL, // 允许所有设备
    .scan_interval = 0x50,                           // 扫描间隔
    .scan_window = 0x30,                             // 扫描窗口
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE     // 禁用重复扫描
};

// 定义设备名称
// static const char device_name[] = "ESP_SPP_SERVER";

// 全局变量
static bool is_connect = false;              // 是否已连接
static bool is_ready_for_read_write = false; // 是否准备好进行读写操作 (服务and特征完成)
static uint16_t spp_conn_id = 0;             // SPP 连接 ID
static uint16_t spp_mtu_size = 23;           // 默认 MTU 大小
static uint16_t cmd = 0;                     // 命令
static uint16_t spp_srv_start_handle = 0;    // 服务起始句柄
static uint16_t spp_srv_end_handle = 0;      // 服务结束句柄
static uint16_t spp_gattc_if = 0xff;         // GATT 客户端接口

static int notify_value_offset = 0;        // 通知值偏移
static int notify_value_count = 0;         // 通知值计数
static uint16_t count = SPP_IDX_NB;        // 数据库元素数量
static esp_gattc_db_elem_t *db = NULL;     // GATT 数据库
static esp_ble_gap_cb_param_t scan_rst;    // 扫描结果
static QueueHandle_t cmd_reg_queue = NULL; // 命令注册队列
QueueHandle_t spp_uart_queue = NULL;       // UART 队列
static uint16_t char_handle = 0;           // 定义全局变量，用于存储特征值句柄
static uint16_t scan_all_the_time = 10;    // 设置扫描时间 单位秒

static bool found_device = false; // 全局变量，初始化为 false
#ifdef SUPPORT_HEARTBEAT
static uint8_t heartbeat_s[9] = {'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'}; // 心跳数据
static QueueHandle_t cmd_heartbeat_queue = NULL;                               // 心跳命令队列
#endif

// SPP 服务 UUID
static esp_bt_uuid_t spp_service_uuid = {
    .len = ESP_UUID_LEN_128, // UUID 长度
    //.uuid.uuid128 = service_uuid,
};

// 通知事件处理函数
static void notify_event_handler(esp_ble_gattc_cb_param_t *p_data)
{
    uint8_t handle = 0;

    if (p_data->notify.is_notify == true)
    {
        ESP_LOGI(GATTC_TAG, "+NOTIFY:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "+INDICATE:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
    }
    handle = p_data->notify.handle;

    // 检查数据库是否为空
    if (db == NULL)
    {
        ESP_LOGE(GATTC_TAG, " %s db is NULL\n", __func__);
        return;
    }

    // 处理通知数据
    if (handle == db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle)
    {
        // 处理通知值
        uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
    }
    else
    {
        esp_log_buffer_char(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
}

// 释放 GATT 客户端服务数据库
static void free_gattc_srv_db(void)
{
    ESP_LOGI("BLE_RESET", "重置 BLE 客户端状态");

    // 复位全局连接状态
    is_connect = false;
    is_ready_for_read_write = false;
    spp_conn_id = 0;
    spp_mtu_size = 23;
    cmd = 0;
    spp_srv_start_handle = 0;
    spp_srv_end_handle = 0;
    spp_gattc_if = 0xff;

    // 释放通知数据
    notify_value_offset = 0;
    notify_value_count = 0;

    // 复位特征值句柄
    char_handle = 0;

    // 清空扫描结果结构体
    // memset(&scan_rst, 0, sizeof(scan_rst));

    // // 清空 gl_profile_tab
    // for (int i = 0; i < PROFILE_NUM; i++)
    // {
    //     gl_profile_tab[i].gattc_if = ESP_GATT_IF_NONE;
    //     gl_profile_tab[i].conn_id = 0;
    //     gl_profile_tab[i].service_start_handle = 0;
    //     gl_profile_tab[i].service_end_handle = 0;
    //     gl_profile_tab[i].char_handle = 0;
    //     memset(gl_profile_tab[i].remote_bda, 0, sizeof(esp_bd_addr_t));
    // }

    // 释放 GATT 服务数据库
    if (db != NULL)
    {
        free(db);
        db = NULL;
    }

    // 清除找到设备标志
    found_device = false;

    // 清除队列指针（不删除队列，只置空，如果要删除请调用 vQueueDelete）
    // cmd_reg_queue = NULL;
    // spp_uart_queue = NULL;
}

// GAP 事件处理函数
// 该函数处理 BLE GAP（通用访问配置文件）相关的事件，例如扫描参数设置、扫描启动/停止、扫描结果等。
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL; // 用于存储广播包中的设备名称
    uint8_t adv_name_len = 0; // 广播包中设备名称的长度
    esp_err_t err;            // 用于存储错误代码

    // 根据 GAP 事件类型进行处理
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        // 扫描参数设置完成事件
        if ((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            // 如果扫描参数设置失败，打印错误日志
            ESP_LOGE(GATTC_TAG, "扫描参数设置失败: %s", esp_err_to_name(err));
            break;
        }

        ESP_LOGI(GATTC_TAG, "启动 BLE 扫描");
        // 开始扫描
        esp_ble_gap_start_scanning(scan_all_the_time);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // 扫描启动完成事件，指示扫描启动成功或失败
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            // 如果扫描启动失败，打印错误日志
            ESP_LOGE(GATTC_TAG, "扫描启动失败: %s", esp_err_to_name(err));
            break;
        }
        // 扫描启动成功
        ESP_LOGI(GATTC_TAG, "扫描启动成功");
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        // 扫描停止完成事件，指示扫描停止成功或失败
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            // 如果扫描停止失败，打印错误日志
            ESP_LOGE(GATTC_TAG, "扫描停止失败: %s", esp_err_to_name(err));
            break;
        }
        // 扫描停止成功
        ESP_LOGI(GATTC_TAG, "扫描停止成功");
        // 如果当前未连接设备，则尝试连接到目标设备
        if (is_connect == false)
        {
            ESP_LOGI(GATTC_TAG, "尝试连接到远程设备");
            esp_err_t erra = esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if,
                                                scan_rst.scan_rst.bda,
                                                BLE_ADDR_TYPE_PUBLIC,
                                                // scan_rst.scan_rst.ble_addr_type,
                                                true);

            ESP_LOGI(GATTC_TAG, "连接地址类型: %d", scan_rst.scan_rst.ble_addr_type);

            if (erra != ESP_OK)
            {
                ESP_LOGE(GATTC_TAG, "连接失败: %s", esp_err_to_name(erra));
            }
            else
            {
                ESP_LOGI(GATTC_TAG, "正在连接设备...");
            }
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
        {
            static uint8_t last_bda[ESP_BD_ADDR_LEN] = {0}; // 保存上一次扫描到的设备地址
            static bool has_last_bda = false;               // 是否已有保存

            if (found_device)
            {
                break;
            }

            // 如果已经扫描过同一个设备，就跳过
            if (has_last_bda && memcmp(last_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN) == 0)
            {
                break;
            }
            // 保存当前设备地址，避免下次重复处理
            memcpy(last_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
            has_last_bda = true;

            // 解析设备名字
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                                &adv_name_len);

            if (adv_name == NULL || adv_name_len == 0)
            {
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_SHORT,
                                                    &adv_name_len);
            }

            // 解析 128位 UUID
            uint8_t uuid_len = 0;
            uint8_t *adv_service_uuid = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                                 ESP_BLE_AD_TYPE_128SRV_CMPL,
                                                                 &uuid_len);

            // 如果 UUID 长度为0，不处理，直接跳过
            if (uuid_len == 0 || adv_service_uuid == NULL)
            {
                break;
            }

            // 打印设备信息
            ESP_LOGI(GATTC_TAG, "Adv report: rssi=%d, addr_type=%d, bda=%02x:%02x:%02x:%02x:%02x:%02x",
                     scan_result->scan_rst.rssi,
                     scan_result->scan_rst.ble_addr_type,
                     scan_result->scan_rst.bda[0], scan_result->scan_rst.bda[1], scan_result->scan_rst.bda[2],
                     scan_result->scan_rst.bda[3], scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5]);

            ESP_LOGI(GATTC_TAG, "UUID 长度: %d", uuid_len);
            esp_log_buffer_hex(GATTC_TAG, adv_service_uuid, uuid_len);

            // 检查服务 UUID 是否匹配
            if (uuid_len == 16 && memcmp(adv_service_uuid, service_uuid, 16) == 0)
            {
                uint8_t ble_mac[ESP_BD_ADDR_LEN];
                esp_err_t mac_err = load_ble_info_mac(ble_mac, ESP_BD_ADDR_LEN);

                if (mac_err == ESP_OK)
                {
                    ESP_LOGI(GATTC_TAG, "NVS 写入的 MAC: " BT_BD_ADDR_STR, BT_BD_ADDR_HEX(ble_mac));

                    // MAC 不匹配，继续扫描
                    if (memcmp(ble_mac, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN) != 0)
                    {
                        ESP_LOGI(GATTC_TAG, "MAC 不匹配，继续扫描...");
                        break;
                    }
                }

                memcpy(&scan_rst, scan_result, sizeof(esp_ble_gap_cb_param_t));

                if (adv_name && adv_name_len > 0)
                {
                    ESP_LOGI(GATTC_TAG, "发现匹配设备: 名称=%.*s", adv_name_len, adv_name);
                }
                else
                {
                    ESP_LOGI(GATTC_TAG, "发现匹配设备，但没有名称");
                }

                found_device = true;
                esp_ble_gap_stop_scanning();
            }
            else
            {
                ESP_LOGI(GATTC_TAG, "UUID 不匹配或无效设备");
            }

            break;
        }

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(GATTC_TAG, "扫描完成");

            if (!found_device)
            {
                ESP_LOGI(GATTC_TAG, "扫描超时");
            }

            break;

        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        // 广播停止完成事件
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            // 如果广播停止失败，打印错误日志
            ESP_LOGE(GATTC_TAG, "广播停止失败: %s", esp_err_to_name(err));
        }
        else
        {
            // 广播停止成功
            ESP_LOGI(GATTC_TAG, "广播停止成功");
        }
        break;

    default:
        // 处理未定义的事件
        ESP_LOGW(GATTC_TAG, "未处理的 GAP 事件: %d", event);
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* 如果事件是注册事件，则为每个 profile 存储 gattc_if */
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "注册应用失败, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    /* 如果 gattc_if 等于 profile A，则调用 profile A 的回调处理函数 */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE 表示未指定特定的 gatt_if，需要调用每个 profile 的回调函数 */
                gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}
/**
 * @brief GATT 客户端事件处理函数
 * @param event GATT 客户端事件类型
 * @param gattc_if GATT 客户端接口
 * @param param GATT 客户端事件参数
 */
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    // 将事件参数转换为更易用的结构体指针
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    // 根据事件类型处理不同的逻辑
    switch (event)
    {
    case ESP_GATTC_REG_EVT: // 注册事件
        ESP_LOGI(GATTC_TAG, "注册事件，设置扫描参数");
        // 设置 BLE 扫描参数
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;

    case ESP_GATTC_CONNECT_EVT: // 连接事件
        ESP_LOGI(GATTC_TAG, "连接事件: conn_id=%d, gatt_if=%d", p_data->connect.conn_id, gattc_if);
        ESP_LOGI(GATTC_TAG, "远程设备地址:");
        // 打印远程设备的 MAC 地址
        esp_log_buffer_hex(GATTC_TAG, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

        // 保存 GATT 客户端接口和连接 ID
        spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;

        // 保存远程设备地址
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        // 开始搜索目标服务
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;

    case ESP_GATTC_DISCONNECT_EVT: // 断开连接事件
        is_connect = false;        // 置位 连接状态
        is_ready_for_read_write = false;
        ESP_LOGI(GATTC_TAG, "断开连接");
        vTaskDelay(pdMS_TO_TICKS(200));
        // 释放 GATT 客户端服务数据库
        free_gattc_srv_db();

        // 重新开始扫描设备
        esp_ble_gap_start_scanning(scan_all_the_time);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(GATTC_TAG, "目标服务是 %d 长度UUID: ", p_data->search_res.srvc_id.uuid.len);

        // 如果服务的 UUID 长度是 16 位（即 2 字节）
        if (p_data->search_res.srvc_id.uuid.len == 2)
        {
            ESP_LOGI(GATTC_TAG, "目标服务是 16 位 UUID: 0x%04x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            // 比较16位 UUID
            if (p_data->search_res.srvc_id.uuid.uuid.uuid16 == 0xfff0)
            {
                ESP_LOGI(GATTC_TAG, "找到目标服务 16 位 UUID");
                spp_srv_start_handle = p_data->search_res.start_handle;
                spp_srv_end_handle = p_data->search_res.end_handle;
            }
        }
        // 如果服务的 UUID 长度是 128 位（即 16 字节）
        else if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128)
        {
            ESP_LOGI(GATTC_TAG, "目标服务是 128 位 UUID: ");
            for (int i = 0; i < ESP_UUID_LEN_128; i++)
            {
                ESP_LOGI(GATTC_TAG, "%02x", p_data->search_res.srvc_id.uuid.uuid.uuid128[i]);
            }
            // 比较128位 UUID
            if (memcmp(p_data->search_res.srvc_id.uuid.uuid.uuid128, service_uuid, ESP_UUID_LEN_128) == 0)
            {
                ESP_LOGI(GATTC_TAG, "找到目标服务 128 位 UUID");
                spp_srv_start_handle = p_data->search_res.start_handle;
                spp_srv_end_handle = p_data->search_res.end_handle;
            }
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "UUID 长度异常: %d", p_data->search_res.srvc_id.uuid.len);
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
    {
        ESP_LOGI(GATTC_TAG, "服务搜索完成: conn_id=%x, status=%d", spp_conn_id, p_data->search_cmpl.status);

        if (p_data->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "服务搜索失败");
            break;
        }
        // 打印服务范围和特征 UUID
        ESP_LOGI(GATTC_TAG, "服务范围: start_handle=%d, end_handle=%d", spp_srv_start_handle, spp_srv_end_handle);
        ESP_LOGI(GATTC_TAG, "特征 UUID: ");
        for (int i = 0; i < ESP_UUID_LEN_128; i++)
        {
            ESP_LOGI(GATTC_TAG, "%02x ", service_char_uuid[i]);
        }

        // 准备需要查找的特征 UUID
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_128,
        };
        memcpy(char_uuid.uuid.uuid128, service_char_uuid, ESP_UUID_LEN_128);

        uint16_t count = 0;
        esp_gattc_char_elem_t *char_elem_result = NULL;

        // 第一步: 先获取特征数量
        esp_err_t err = esp_ble_gattc_get_attr_count(
            gattc_if,
            spp_conn_id,
            ESP_GATT_DB_CHARACTERISTIC,
            spp_srv_start_handle,
            spp_srv_end_handle,
            INVALID_HANDLE, // 这里使用 INVALID_HANDLE，如果没有定义，可以自己加个宏：#define INVALID_HANDLE    0
            &count);

        if (err != ESP_OK || count == 0)
        {
            ESP_LOGE(GATTC_TAG, "获取特征数量失败: %s, count=%d", esp_err_to_name(err), count);
            break;
        }

        // 第二步: 分配内存存放特征元素
        char_elem_result = malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (!char_elem_result)
        {
            ESP_LOGE(GATTC_TAG, "分配内存失败");
            break;
        }

        // 第三步: 通过 UUID 获取特征列表
        err = esp_ble_gattc_get_char_by_uuid(
            gattc_if,
            spp_conn_id,
            spp_srv_start_handle,
            spp_srv_end_handle,
            char_uuid,
            char_elem_result,
            &count);

        if (err == ESP_OK && count > 0)
        {
            char_handle = char_elem_result[0].char_handle;
            ESP_LOGI(GATTC_TAG, "成功找到特征，handle = 0x%04x", char_handle);
            is_ready_for_read_write = true;

            uint8_t ble_mac[ESP_BD_ADDR_LEN];
            load_ble_info_mac(ble_mac, ESP_BD_ADDR_LEN);

            // 读取vfs保存的mac  进行对比，不一样或者没有 在写入 减小磨损
            if (memcmp(ble_mac, gl_profile_tab[PROFILE_APP_ID].remote_bda, ESP_BD_ADDR_LEN) != 0)
            {
                // 完成后在保存MAC
                err = save_ble_info_mac(gl_profile_tab[PROFILE_APP_ID].remote_bda);
                ESP_LOGI(GATTC_TAG, "mac-写入->nvs 是否成功%s", esp_err_to_name(err));
            }

            // ESP_LOGI(GATTC_TAG, "NVS 写入的 MAC: " BT_BD_ADDR_STR, BT_BD_ADDR_HEX(ble_mac));
        }
        else
        {
            is_ready_for_read_write = false;
            ESP_LOGE(GATTC_TAG, "通过 UUID 查找特征失败: %s", esp_err_to_name(err));
        }

        // 完成服务搜索后，获取服务和特征句柄
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;

        free(char_elem_result);
        break;
    }

    case ESP_GATTC_READ_CHAR_EVT: // 读取特征值事件
        ESP_LOGI(GATTC_TAG, "读取特征值事件: char_handle=%d, value_len=%d",
                 p_data->read.handle, p_data->read.value_len);
        esp_log_buffer_hex(GATTC_TAG, p_data->read.value, p_data->read.value_len);
        break;

    case ESP_GATTC_NOTIFY_EVT: // 通知事件
        ESP_LOGI(GATTC_TAG, "通知事件");
        // 调用通知事件处理函数
        notify_event_handler(p_data);
        break;

    case ESP_GATTC_WRITE_CHAR_EVT: // 写特征值完成事件
        ESP_LOGI(GATTC_TAG, "写特征值事件: status=%d, handle=%d", p_data->write.status, p_data->write.handle);
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "写特征值失败, 错误状态=%d", p_data->write.status);
        }
        break;

    case ESP_GATTC_CFG_MTU_EVT: // MTU 配置完成事件
        if (p_data->cfg_mtu.status != ESP_OK)
        {
            ESP_LOGE(GATTC_TAG, "MTU 配置失败，状态: %d", p_data->cfg_mtu.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "MTU 配置成功: %d", p_data->cfg_mtu.mtu);
        // 保存 MTU 大小
        spp_mtu_size = p_data->cfg_mtu.mtu;
        break;

    default: // 未处理的事件
        ESP_LOGW(GATTC_TAG, "未处理的事件: %d", event);
        break;
    }
}

void spp_client_reg_task(void *arg)
{
    uint16_t cmd_id;
    for (;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS); // 延迟 100 毫秒
        if (xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY))
        { // 从命令队列中接收命令 ID
            if (db != NULL)
            { // 检查数据库是否为空
                if (cmd_id == SPP_IDX_SPP_DATA_NTY_VAL)
                { // 如果命令是数据通知值
                    ESP_LOGI(GATTC_TAG, "Index = %d,UUID = 0x%04x, handle = %d \n",
                             cmd_id, (db + SPP_IDX_SPP_DATA_NTY_VAL)->uuid.uuid.uuid16,
                             (db + SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                    // 注册通知
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda,
                                                      (db + SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                }
                else if (cmd_id == SPP_IDX_SPP_STATUS_VAL)
                { // 如果命令是状态值
                    ESP_LOGI(GATTC_TAG, "Index = %d,UUID = 0x%04x, handle = %d \n",
                             cmd_id, (db + SPP_IDX_SPP_STATUS_VAL)->uuid.uuid.uuid16,
                             (db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                    // 注册通知
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda,
                                                      (db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
#ifdef SUPPORT_HEARTBEAT
                else if (cmd_id == SPP_IDX_SPP_HEARTBEAT_VAL)
                { // 如果命令是心跳值
                    ESP_LOGI(GATTC_TAG, "Index = %d,UUID = 0x%04x, handle = %d \n",
                             cmd_id, (db + SPP_IDX_SPP_HEARTBEAT_VAL)->uuid.uuid.uuid16,
                             (db + SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle);
                    // 注册通知
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda,
                                                      (db + SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle);
                }
#endif
            }
        }
    }
}

#ifdef SUPPORT_HEARTBEAT
void spp_heart_beat_task(void *arg)
{
    uint16_t cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 延迟 50 毫秒
        if (xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY))
        { // 从心跳队列中接收命令 ID
            while (1)
            {
                if ((is_connect == true) && (db != NULL) &&
                    ((db + SPP_IDX_SPP_HEARTBEAT_VAL)->properties &
                     (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE)))
                {
                    // 写入心跳特征值
                    esp_ble_gattc_write_char(spp_gattc_if, spp_conn_id,
                                             (db + SPP_IDX_SPP_HEARTBEAT_VAL)->attribute_handle,
                                             sizeof(heartbeat_s), (uint8_t *)heartbeat_s,
                                             ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
                    vTaskDelay(5000 / portTICK_PERIOD_MS); // 延迟 5 秒
                }
                else
                {
                    ESP_LOGI(GATTC_TAG, "断开连接\n");
                    break;
                }
            }
        }
    }
}
#endif

void ble_client_appRegister(void)
{
    esp_err_t status;
    char err_msg[20];

    ESP_LOGI(GATTC_TAG, "注册回调函数");

    // 注册 GAP 回调函数
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG, "GAP 注册失败: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    // 注册 GATTC 回调函数
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG, "GATTC 注册失败: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    // 注册应用程序
    esp_ble_gattc_app_register(PROFILE_APP_ID);

    // 设置本地 MTU
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(200);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "设置本地 MTU 失败: %s", esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
    }

    // 创建命令注册队列
    cmd_reg_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);

#ifdef SUPPORT_HEARTBEAT
    // 创建心跳队列
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_heart_beat_task, "spp_heart_beat_task", 2048, NULL, 10, NULL);
#endif
}

esp_err_t ble_client_init(void)
{
    // 设置uuid
    memcpy(spp_service_uuid.uuid.uuid128, service_uuid, sizeof(service_uuid));

    esp_err_t ret;

    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化蓝牙控制器配置
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg); // 初始化蓝牙控制器
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s 初始化控制器失败: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE); // 启用 BLE 模式
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s 启用控制器失败: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(GATTC_TAG, "%s 初始化蓝牙\n", __func__);
    ret = esp_bluedroid_init(); // 初始化 Bluedroid
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s 初始化蓝牙失败: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_enable(); // 启用 Bluedroid
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s 启用蓝牙失败: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    // 注册 BLE 客户端应用程序
    ble_client_appRegister();

    return ret;
}

bool ble_is_connected()
{
    return is_connect;
}

// 检查蓝牙是否准备好进行读写操作
bool ble_is_ready_for_read_write()
{
    return is_ready_for_read_write;
}

// 获取MAC
const uint8_t *ble_get_mac_address()
{
    return esp_bt_dev_get_address();
}

// 设置mtu
esp_err_t ble_set_local_mtu(uint16_t mtu)
{
    return esp_ble_gatt_set_local_mtu(mtu);
}

/* 发送 int 数组数据（通过 BLE 通知）
 * 返回值表示是否发送成功（仅在已连接状态下尝试发送）
 */
bool ble_send_int_array(int *array, size_t size)
{
    // 检查连接状态和数组合法性
    if ((!is_connect || !is_ready_for_read_write) || array == NULL || size == 0)
    {
        ESP_LOGW(GATTC_TAG, "无法发送数据：未准备就绪或数组为空");
        return false;
    }

    // 尝试发送通知
    esp_err_t err = esp_ble_gattc_write_char(
        spp_gattc_if,            // GATT 客户端接口
        spp_conn_id,             // 连接 ID
        char_handle,             // 特征值句柄
        size * sizeof(int),      // 数据长度
        (uint8_t *)array,        // 数据内容
        ESP_GATT_WRITE_TYPE_RSP, // 写入类型（响应）
        ESP_GATT_AUTH_REQ_NONE   // 无认证要求
    );

    // 检查发送结果
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG, "BLE 消息发送失败，错误码: 0x%x", err);
        return false;
    }

    ESP_LOGI(GATTC_TAG, "BLE 发送 int 数组数据，大小: %d", size);
    return true;
}

/* 发送字符串数据通过 BLE
 * 返回值表示是否发送成功（仅在已连接状态下尝试发送）
 */
bool ble_send_string_data(const char *message)
{
    // 检查连接状态和指针合法性
    if ((!is_connect || !is_ready_for_read_write) || message == NULL)
    {
        ESP_LOGW(GATTC_TAG, "无法发送数据：未准备就绪或消息为空");
        return false;
    }

    // 尝试发送通知
    esp_err_t err = esp_ble_gattc_write_char(
        spp_gattc_if,            // GATT 客户端接口
        spp_conn_id,             // 连接 ID
        char_handle,             // 特征值句柄
        strlen(message),         // 消息长度
        (uint8_t *)message,      // 消息数据
        ESP_GATT_WRITE_TYPE_RSP, // 写入类型（响应）
        ESP_GATT_AUTH_REQ_NONE   // 无认证要求
    );

    // 检查发送结果
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTC_TAG, "BLE 消息发送失败，错误码: 0x%x", err);
        return false;
    }

    ESP_LOGI(GATTC_TAG, "BLE 发送字符串数据: %s", message);
    return true;
}

// 设置扫描时间
void set_ble_scan_duration(uint16_t seconds)
{
    scan_all_the_time = seconds;
}

// 再一次扫描
void start_scan_again()
{
    if (!is_connect && !is_ready_for_read_write && !found_device)
    {
        esp_ble_gap_start_scanning(scan_all_the_time);
    }
}

// 停止扫描 并且 主动断开连接
void ble_client_disconnect(void)
{
    //断开连接
    esp_err_t err = esp_ble_gattc_close(spp_gattc_if, spp_conn_id);

    if (err != ESP_OK)
    {
        ESP_LOGE("BLE_DISCONNECT", "主动断开连接失败: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI("BLE_DISCONNECT", "已主动断开连接");
    }

    esp_ble_gap_stop_scanning();
}