#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"
#include "esp_bt_defs.h"

#include "ble_spp_server_demo.h"

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint16_t service_handle = 0; // 服务句柄，在服务创建成功后获得
static uint16_t char_handle = 0;    // 特征值句柄，在添加特征成功后获得
static esp_gatt_if_t gatts_if = 0;  // 当前 GATT 接口 ID
static uint8_t adv_config_done = 0;
static uint16_t descr_handle = 0;   // 保存 CCCD 描述符的句柄



static const char *TAG = "BLE_SERVER"; // 日志 TAG
static bool is_connected = false;      // 链接状态

// #define GATTS_SERVICE_UUID 0x00FF       // 自定义服务 UUID
// #define GATTS_CHAR_UUID 0xFF01 // 自定义特征 UUID
#define GATTS_NUM_HANDLE 4 // GATT 服务句柄数量
#define ESP_APP_ID 0x55    // 应用 ID

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

// 广播数据和扫描响应数据
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,       // 标志位：非扫描响应包
    .include_name = true,        // 包含设备名称
    .include_txpower = true,     // 包含 TX 功率
    .min_interval = 0x20,        // 最小广播间隔
    .max_interval = 0x40,        // 最大广播间隔
    .appearance = 0x00,          // 设备外观（可以设置为具体的外观类型）
    .manufacturer_len = 0,       // 制造商数据长度
    .p_manufacturer_data = NULL, // 制造商数据
    .service_data_len = 0,       // 服务数据长度
    .p_service_data = NULL,      // 服务数据
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,        // 标志位：扫描响应包
    .include_name = true,        // 包含设备名称
    .include_txpower = true,     // 包含 TX 功率
    .min_interval = 0x20,        // 最小广播间隔
    .max_interval = 0x40,        // 最大广播间隔
    .appearance = 0x00,          // 设备外观
    .manufacturer_len = 0,       // 制造商数据长度
    .p_manufacturer_data = NULL, // 制造商数据
    .service_data_len = 0,       // 服务数据长度
    .p_service_data = NULL,      // 服务数据
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x40,                                    // 设置最小广播间隔为 20ms（0x20 * 0.625ms
    .adv_int_max = 0x40,                                    // 设置最大广播间隔为 40ms（0x40 * 0.625ms）
    .adv_type = ADV_TYPE_IND,                               // 设置广播类型为
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,                  // 使用公共设备地址类型进行广播
    .channel_map = ADV_CHNL_ALL,                            // 使用所有 BLE 广播频道（37, 38, 39）
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // 允许任何设备扫描并允许任何设备连接
};

// 转换
static uint8_t hex_char_to_val(char c)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if ('a' <= c && c <= 'f')
        return c - 'a' + 10;
    if ('A' <= c && c <= 'F')
        return c - 'A' + 10;
    return 0xFF;
}

// 字符串转换uuid
bool parse_uuid128_string(const char *uuid_str, uint8_t uuid[16])
{
    if (!uuid_str || strlen(uuid_str) != 36)
        return false;

    // 格式检查：xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
    for (int i = 0; i < 36; ++i)
    {
        if ((i == 8 || i == 13 || i == 18 || i == 23))
        {
            if (uuid_str[i] != '-')
                return false;
        }
        else
        {
            if (!isxdigit((unsigned char)uuid_str[i]))
                return false;
        }
    }

    int hex_index = 0;
    for (int i = 0; i < 36 && hex_index < 16; i += 2)
    {
        if (uuid_str[i] == '-')
        {
            i++; // skip dash
        }

        uint8_t high = hex_char_to_val(uuid_str[i]);
        uint8_t low = hex_char_to_val(uuid_str[i + 1]);

        if (high == 0xFF || low == 0xFF)
            return false;

        uuid[15 - hex_index++] = (high << 4) | low;
    }

    return true;
}

// GATT 事件处理函数，用于响应 GATT 协议相关事件
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatt_if,
                         esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT: // 注册事件，GATT 服务启动入口
        ESP_LOGI(TAG, "注册完成，开始创建服务");
        esp_ble_gap_set_device_name(DEVICE_NAME); // 设置广播设备名称

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data); // 配置广告数据

        if (ret)
        {
            ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;

        esp_ble_gap_config_adv_data(&scan_rsp_data); // 配置扫描响应数据
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;

        // 配置服务 ID（UUID + 实例号）
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,              // 表示该服务是主服务（primary service），设为 false 则表示为辅助服务（secondary service）
            .id.inst_id = 0x00,              // 服务实例 ID，多个相同 UUID 的服务时可用于区分（通常设为 0）
            .id.uuid.len = ESP_UUID_LEN_128, // 设置 UUID 长度为 16 位，如果使用 128 位 UUID，则改为 ESP_UUID_LEN_128
            //.id.uuid.uuid.uuid128 = service_uuid, // 填写服务的 UUID，这里是自定义的 16 位 UUID 宏定义
        };

        // 拷贝
        memcpy(service_id.id.uuid.uuid.uuid128, service_uuid, 16);

        /*向GATT 协议栈请求创建一个服务：
        gatt_if：GATT 接口编号，由注册回调时返回。

        &service_id：刚刚定义的服务 ID 指针。

        GATTS_NUM_HANDLE：该服务需要分配的总句柄数量（比如服务本身、特征值、描述符等的总和）。

        创建完成后会触发 ESP_GATTS_CREATE_EVT 事件，在事件处理函数中可以获得 service_handle 并继续添加特征值。
        */
        esp_ble_gatts_create_service(gatt_if, &service_id, GATTS_NUM_HANDLE);

        gatts_if = gatt_if;

        break;

    case ESP_GATTS_CREATE_EVT: // 服务创建成功事件
        ESP_LOGI(TAG, "服务创建成功");
        service_handle = param->create.service_handle; // 保存服务句柄

        // 添加特征值（可写 + Notify 属性）
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_128,
            // .uuid.uuid128 = service_char_uuid,
        };

        // 拷贝
        memcpy(char_uuid.uuid.uuid128, service_char_uuid, 16);

        esp_ble_gatts_add_char(service_handle, &char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,                    // 可读可写权限
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, // 属性
                               NULL, NULL);

        esp_ble_gatts_start_service(service_handle); // 启动服务
        break;

    case ESP_GATTS_ADD_CHAR_EVT: // 特征添加成功事件
        ESP_LOGI(TAG, "添加特征成功");
        char_handle = param->add_char.attr_handle; // 保存特征值句柄

        esp_bt_uuid_t descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, // CCCD 的标准 UUID
        };

        esp_ble_gatts_add_char_descr(service_handle, &descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(TAG, "添加描述符成功");
        descr_handle = param->add_char_descr.attr_handle;
        break;
    case ESP_GATTS_WRITE_EVT: // 收到写入事件（手机写入数据）
        if (!param->write.is_prep && param->write.len > 0)
        {
            ESP_LOGI(TAG, "收到写入，长度 %d", param->write.len);
            char recv_buf[128] = {0};
            memcpy(recv_buf, param->write.value, param->write.len);
            ESP_LOGI(TAG, "接收到字符串: %s", recv_buf);

            // 发送 Notify 给客户端（如手机 App）
            // esp_ble_gatts_send_indicate(gatt_if, param->write.conn_id,
            //                             char_handle, strlen(recv_buf), (uint8_t *)recv_buf, false);

            // 如果是写入 CCCD，我们必须响应
            if (param->write.handle == descr_handle)
            {
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->write.handle;
                rsp.attr_value.len = param->write.len;
                memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, &rsp);

                // 解析值（小端），0x0001 是 Notify，0x0002 是 Indicate
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(TAG, "notify enable");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                        notify_data[i] = i % 0xff;

                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, char_handle,
                                                sizeof(notify_data), notify_data, false);
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(TAG, "indicate enable");
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); ++i)
                        indicate_data[i] = i % 0xff;

                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                }
            }
        }
        break;

    // case ESP_GATTS_EXEC_WRITE_EVT:
    //     // the length of gattc prepare write data must be less than GATTS_EXAMPLE_CHAR_VAL_LEN_MAX.
    //     ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT, Length=%d", prepare_write_env.prepare_len);
    //     example_exec_write_event_env(&prepare_write_env, param);
    //     break;
    case ESP_GATTS_MTU_EVT:
        // 收到对方设置的最大传输单元（MTU）大小
        ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT，MTU = %d", param->mtu.mtu);
        break;

    case ESP_GATTS_CONF_EVT:
        // 通知或指示确认事件（对端设备确认已收到）
        ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT，写确认状态 = %d", param->conf.status);
        break;

    case ESP_GATTS_START_EVT:
        // 服务启动事件（服务创建后正式启动）
        ESP_LOGI(TAG, "SERVICE_START_EVT，状态 = %d，服务句柄 = %d",
                 param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT:
        // 连接事件（被主设备连接）
        ESP_LOGI(TAG, "设备已连接，conn_ID = %d", param->connect.conn_id);
        is_connected = true;
        // 建议开启加密连接，提高安全性（可选）
        // esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        // 断开连接事件
        ESP_LOGI(TAG, "设备已断开，reason = %d", param->disconnect.reason);
        is_connected = false;
        // 重新开始广播，使设备可以再次被发现并连接
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

// GAP（广播相关）事件处理
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    // // 判断事件类型，如果是广播数据设置完成事件（ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT）
    // if (event == ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT)
    // {
    //     // 广播数据设置完成，开始启动广播
    //     esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
    //         .adv_int_min = 0x20,                                    // 设置最小广播间隔为 20ms（0x20 * 0.625ms）
    //         .adv_int_max = 0x40,                                    // 设置最大广播间隔为 40ms（0x40 * 0.625ms）
    //         .adv_type = ADV_TYPE_IND,                               // 设置广播类型为指示广播（ADV_TYPE_IND）
    //         .own_addr_type = BLE_ADDR_TYPE_PUBLIC,                  // 使用公共设备地址类型进行广播
    //         .channel_map = ADV_CHNL_ALL,                            // 使用所有 BLE 广播频道（37, 38, 39）
    //         .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // 允许任何设备扫描并允许任何设备连接
    //     });
    // }

    switch (event)
    {
    // 广播数据设置完成事件
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // 清除广播数据配置完成标志位
        adv_config_done &= (~ADV_CONFIG_FLAG);
        // 如果广播数据和扫描响应数据都配置完成，则开始广播
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    // 扫描响应数据设置完成事件
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        // 清除扫描响应数据配置完成标志位
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        // 如果广播数据和扫描响应数据都配置完成，则开始广播
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    // 默认处理（不处理其他 GAP 事件）
    default:
        break;
    }
}

// 默认初始化 BLE 服务器
esp_err_t ble_server_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "初始化 BLE 控制器...");

    // 释放经典蓝牙内存（仅使用 BLE）
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "释放经典蓝牙内存失败: 0x%x", ret);
        return ret;
    }

    // 初始化 BLE 控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BLE 控制器初始化失败: 0x%x", ret);
        return ret;
    }

    // 启用 BLE 控制器
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "启用 BLE 控制器失败: 0x%x", ret);
        return ret;
    }

    // 初始化并启用 BlueDroid 协议栈
    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid 初始化失败: 0x%x", ret);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid 启用失败: 0x%x", ret);
        return ret;
    }

    // 注册 GATT 和 GAP 回调函数
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "注册 GATT 回调失败: 0x%x", ret);
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "注册 GAP 回调失败: 0x%x", ret);
        return ret;
    }

    // 注册应用
    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GATTS 应用注册失败: 0x%x", ret);
        return ret;
    }

    // 设置本地最大 MTU
    ret = esp_ble_gatt_set_local_mtu(33);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置本地 MTU 失败: 0x%x", ret);
        return ret;
    }
    // // 注册一个 GATT 应用，传入的 app_id 参数为 0（可以忽略，ID 可用于区分不同的应用）
    // ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    ESP_LOGI(TAG, "BLE 服务器初始化完成！");
    return ESP_OK;
}

bool ble_is_connected()
{
    return is_connected;
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
    if (!is_connected || array == NULL || size == 0)
    {
        ESP_LOGW(TAG, "无法发送数据：未连接或数组为空");
        return false;
    }

    // 尝试发送通知
    esp_err_t err = esp_ble_gatts_send_indicate(
        gatts_if,           // GATT 服务接口
        0,                  // 连接 ID
        char_handle,        // 特征值句柄
        size * sizeof(int), // 数据长度
        (uint8_t *)array,   // 数据内容
        false               // 不要求确认
    );

    // 检查发送结果
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BLE 消息发送失败，错误码: 0x%x", err);
        return false;
    }

    ESP_LOGI(TAG, "BLE 发送 int 数组数据，大小: %d", size);
    return true;
}

/* 发送字符串数据通过 BLE
 * 返回值表示是否发送成功（仅在已连接状态下尝试发送）
 */
bool ble_send_string_data(const char *message)
{
    // 检查连接状态和指针合法性
    if (!is_connected || message == NULL)
    {
        ESP_LOGW(TAG, "无法发送数据：未连接或消息为空");
        return false;
    }

    // 尝试发送通知
    esp_err_t err = esp_ble_gatts_send_indicate(
        gatts_if,           // GATT 服务接口
        0,                  // 连接 ID，这里为 0，需确保连接时记录正确
        char_handle,        // 特征值句柄
        strlen(message),    // 消息长度
        (uint8_t *)message, // 消息数据
        false               // 不要求确认（notify 而非 indicate）
    );

    // 检查发送结果
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BLE 消息发送失败，错误码: 0x%x", err);
        return false;
    }

    ESP_LOGI(TAG, "BLE 发送字符串数据: %s", message);
    return true;
}
