/* ble_conductor.cpp — Spike 0：生产 motoguide1.43 BLE 外设移植
 * 源：components/ble_peripheral_bsp/ble_peripheral_bsp.c（App 已验证可连的线上方案）
 * 裁剪/新增清单见 ble_conductor.h 头注释。
 * ★ 广播名 "Moto Guide" 一字不动（App 扫描按名称过滤）。
 * ★ UUID 三件套与生产逐字节一致：svc 4fafc201-… / 写 beb5483e-… / 通知 e1f2a3b4-… */

#include "ble_conductor.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"
#include "esp_heap_caps.h"

#define TAG "BLECOND"
#define DEVICE_NAME "Moto Guide"
#define GATTS_NUM_HANDLE 7

/* ── UUID（照抄生产，BLE 空口字节序 = 分组小端）──
 * ★ 2026-10-28 定案：这就是生产正规形态，不许"修正"！
 *   App 侧 NordicBleManager.huaweiUuidMatches() 专门匹配此形态
 *   （14:43~15:37 五次校验全过：4b9131c3-…/a8261b36-…/6f5e4d3c-… 全命中）。
 *   若改成"标准全反"，华为机上标准 getService 和兼容函数都匹配不上，生产配对直接坏。 */
static uint8_t svc_uuid[ESP_UUID_LEN_128] = {
    0x01, 0xc2, 0xaf, 0x4f,  0xb5, 0x1f,  0x9e, 0x45,
    0x8f, 0xcc,  0xc5, 0xc9, 0xc3, 0x31, 0x91, 0x4b
};
static uint8_t char_uuid[ESP_UUID_LEN_128] = {       // beb5483e-…（写）
    0x3e, 0x48, 0xb5, 0xbe,  0xe1, 0x36,  0x88, 0x46,
    0xb7, 0xf5,  0xea, 0x07,  0x36, 0x1b, 0x26, 0xa8
};
static uint8_t dev_status_char_uuid[ESP_UUID_LEN_128] = {  // e1f2a3b4-…（通知）
    0xb4, 0xa3, 0xf2, 0xe1,  0xd6, 0xc5,  0x89, 0x47,
    0x8e, 0x0f,  0x1a, 0x2b,  0x3c, 0x4d, 0x5e, 0x6f
};

enum { IDX_SVC, IDX_CHAR_DECL, IDX_CHAR_VAL, IDX_CHAR_CFG,
       IDX_CHAR2_DECL, IDX_CHAR2_VAL, IDX_CHAR2_CFG };

static char     g_device_name[32] = DEVICE_NAME;
static bool     g_connected = false;
static bool     g_advertising = false;
static bool     g_e2_subscribed = false;   // CCCD 订阅状态（每连接复位）
static uint16_t g_conn_id = 0;
static esp_gatt_if_t g_gatts_if = ESP_GATT_IF_NONE;
static ble_conductor_write_cb_t g_write_cb = NULL;

/* 写日志大包限速（照抄生产：BTC 任务里刷串口会拖垮 BLE 主机） */
#define WR_LOG_BULK_LEN_BYTES 200
#define WR_LOG_BULK_MIN_MS    1000
static int        s_wr_log_len  = -1;
static TickType_t s_wr_log_tick = 0;

/* 0xD0 内置解析暂存（BTC 任务写 / 播放任务读，跨核 —— 自旋锁保护） */
static ble_conductor_target_t s_target;
static portMUX_TYPE s_target_mux = portMUX_INITIALIZER_UNLOCKED;

void ble_conductor_target_snapshot(ble_conductor_target_t *out)
{
    portENTER_CRITICAL(&s_target_mux);
    *out = s_target;
    portEXIT_CRITICAL(&s_target_mux);
}

/* ── 内存账（Spike 0 核心产出）── */
static void heap_report(const char* when)
{
    uint32_t intern      = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t intern_max  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint32_t dma_max     = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "[heap] %s: 内部 %uKB(最大 %uKB) DMA最大 %uKB",
             when, (unsigned)(intern / 1024), (unsigned)(intern_max / 1024),
             (unsigned)(dma_max / 1024));
}

/* ── GATTS 属性表（照抄生产）── */
static const uint8_t char_prop = ESP_GATT_CHAR_PROP_BIT_WRITE
                               | ESP_GATT_CHAR_PROP_BIT_WRITE_NR
                               | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t dev_status_char_prop = ESP_GATT_CHAR_PROP_BIT_NOTIFY
                                          | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint16_t prim_svc_uuid  = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_ccc_uuid  = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static uint8_t char_value[517] = {0};        // 生产 09-28 教训：必须 ≥ MTU-3
static uint8_t ccc_value[2]        = {0};
static uint8_t dev_status_value[32] = {0};
static uint8_t dev_status_ccc[2]   = {0};

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)
static uint8_t adv_config_done = 0;
static uint16_t gatt_handle_table[GATTS_NUM_HANDLE];

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x160,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = false,
    .min_interval = 0x0018,   // 30ms（生产降频省电值，照抄）
    .max_interval = 0x0030,   // 60ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = ESP_UUID_LEN_128,
    .p_service_uuid = svc_uuid,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

// IDF 5.5 的 esp_ble_gap_config_adv_data() 参数是非 const 指针（esp_gap_ble_api.h:2861），
// 不能加 const —— 编译报错 [-fpermissive]（2026-10-28 首次编译实测）。
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,     // ★ 名字在 scan response —— App 按名过滤，不可动
    .include_txpower = true,
    .min_interval = 0x0018,
    .max_interval = 0x0030,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static const esp_gatts_attr_db_t gatts_db[GATTS_NUM_HANDLE] = {
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&prim_svc_uuid, ESP_GATT_PERM_READ,
         ESP_UUID_LEN_128, ESP_UUID_LEN_128, svc_uuid}
    },
    [IDX_CHAR_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
         sizeof(char_prop), sizeof(char_prop), (uint8_t *)&char_prop}
    },
    [IDX_CHAR_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, char_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(char_value), sizeof(char_value), char_value}
    },
    [IDX_CHAR_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_ccc_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(ccc_value), sizeof(ccc_value), ccc_value}
    },
    [IDX_CHAR2_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
         sizeof(dev_status_char_prop), sizeof(dev_status_char_prop),
         (uint8_t *)&dev_status_char_prop}
    },
    [IDX_CHAR2_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, dev_status_char_uuid,
         ESP_GATT_PERM_READ,
         sizeof(dev_status_value), sizeof(dev_status_value), dev_status_value}
    },
    [IDX_CHAR2_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_ccc_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(dev_status_ccc), sizeof(dev_status_ccc), dev_status_ccc}
    },
};

static void start_advertising(void)
{
    if (!g_advertising) esp_ble_gap_start_advertising(&adv_params);
}

static void handle_gatts_event(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                               esp_ble_gatts_cb_param_t *param);
static void handle_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void handle_gatts_event(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                               esp_ble_gatts_cb_param_t *param)
{
    /* 本工程无 OTA，单 GATTS app，无需生产版的转发/过滤 */
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTS register failed, status=%d", param->reg.status);
            return;
        }
        g_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(g_device_name);   // "Moto Guide"
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        esp_ble_gatts_create_attr_tab(gatts_db, gatts_if, GATTS_NUM_HANDLE, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Create attr table failed, status=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != GATTS_NUM_HANDLE) {
            ESP_LOGE(TAG, "Attr table num_handle mismatch: %d", param->add_attr_tab.num_handle);
        } else {
            memcpy(gatt_handle_table, param->add_attr_tab.handles, sizeof(gatt_handle_table));
            esp_ble_gatts_start_service(gatt_handle_table[IDX_SVC]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        g_conn_id = param->connect.conn_id;
        g_connected = true;
        g_advertising = false;
        g_e2_subscribed = false;
        ESP_LOGI(TAG, "Client connected, conn_id=%d", param->connect.conn_id);
        heap_report("connected");
        {   // 连接参数照抄生产：30-50ms / latency 0 / 8s 监督超时
            esp_ble_conn_update_params_t conn_params = {
                .bda = {0},  // 下一行 memcpy 填入对端地址
                .min_int = 0x18, .max_int = 0x28,
                .latency = 0,    .timeout = 800,
            };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gap_update_conn_params(&conn_params);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected, reason=0x%x", param->disconnect.reason);
        g_connected = false;
        g_conn_id = 0;
        g_e2_subscribed = false;
        heap_report("disconnected");
        start_advertising();
        break;

    case ESP_GATTS_READ_EVT:
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_WRITE_EVT: {
        const int        wr_len     = (int)param->write.len;
        const bool       wr_is_bulk = (wr_len > WR_LOG_BULK_LEN_BYTES);
        const TickType_t wr_now     = xTaskGetTickCount();
        if (!wr_is_bulk || wr_len != s_wr_log_len ||
            (wr_now - s_wr_log_tick) >= pdMS_TO_TICKS(WR_LOG_BULK_MIN_MS)) {
            ESP_LOGI(TAG, "Write: handle=%d, len=%d", param->write.handle, param->write.len);
            s_wr_log_len  = wr_len;
            s_wr_log_tick = wr_now;
        }
        /* E2 通知 CCCD 订阅跟踪（照抄生产：CCCD 是每连接状态） */
        if (param->write.handle == gatt_handle_table[IDX_CHAR2_CFG] && param->write.len == 2) {
            uint16_t cccd = (uint16_t)param->write.value[0]
                          | ((uint16_t)param->write.value[1] << 8);
            g_e2_subscribed = (cccd == 0x0001);
            ESP_LOGI(TAG, "E2 notify %s (CCCD=0x%04X)",
                     g_e2_subscribed ? "subscribed" : "unsubscribed", cccd);
        }
        /* 0xD0 对拍指令解析（10B：[0xD0][segId u32 LE][frameIdx u32 LE][flags u8]）
         * ★ 2026-10-28 二轮意见①：meter→frameIdx（字节布局不变，语义替换，帧号 1:1 对拍） */
        if (param->write.len == 10 && param->write.value[0] == 0xD0) {
            const uint8_t *v = param->write.value;
            portENTER_CRITICAL(&s_target_mux);
            s_target.valid   = true;
            s_target.seg_id  = (uint32_t)v[1] | ((uint32_t)v[2] << 8)
                             | ((uint32_t)v[3] << 16) | ((uint32_t)v[4] << 24);
            s_target.frame_idx = (uint32_t)v[5] | ((uint32_t)v[6] << 8)
                               | ((uint32_t)v[7] << 16) | ((uint32_t)v[8] << 24);
            s_target.flags   = v[9];
            s_target.last_tick = wr_now;
            portEXIT_CRITICAL(&s_target_mux);
            ESP_LOGI(TAG, "0xD0 seg=%u frame=%u flags=0x%02X",
                     (unsigned)s_target.seg_id, (unsigned)s_target.frame_idx, s_target.flags);
        }
        if (g_write_cb && param->write.len > 0) {
            g_write_cb(param->write.value, param->write.len);
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
    }

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU changed: %d", param->mtu.mtu);
        break;

    default:
        break;
    }
}

static void handle_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) start_advertising();
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) start_advertising();
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed: 0x%x", param->adv_start_cmpl.status);
        } else {
            g_advertising = true;
            ESP_LOGI(TAG, "Advertising started as \"%s\"", g_device_name);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        g_advertising = false;
        break;

    default:
        break;
    }
}

int ble_conductor_start(void)
{
    memset(&s_target, 0, sizeof(s_target));
    adv_config_done = 0;
    g_gatts_if = ESP_GATT_IF_NONE;

    heap_report("ble start 前");

    /* NVS：SMP bond 存储依赖；已被其他组件初始化过则直接过 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* 生产 main.c 同款链路：释放经典 BT 内存 → controller → Bluedroid */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    heap_report("controller init 后");
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    heap_report("controller enable 后");
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    heap_report("bluedroid enable 后");

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(handle_gatts_event));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(handle_gap_event));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));

    /* 广播全功率（生产有低功率开机阶段，本工程无此需求） */
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
    ESP_LOGI(TAG, "BLE conductor init complete (name=\"%s\")", g_device_name);
    return 0;
}

void ble_conductor_set_write_callback(ble_conductor_write_cb_t cb) { g_write_cb = cb; }

bool ble_conductor_is_connected(void)      { return g_connected; }
bool ble_conductor_is_e2_subscribed(void)  { return g_e2_subscribed; }

void ble_conductor_send_notify(const uint8_t *data, size_t len)
{
    if (!g_connected || g_gatts_if == ESP_GATT_IF_NONE || !data || len == 0) return;
    esp_err_t err = esp_ble_gatts_send_indicate(g_gatts_if, g_conn_id,
                                                gatt_handle_table[IDX_CHAR2_VAL],
                                                len, (uint8_t *)data, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "notify FAILED: %s (e2_subscribed=%d)",
                 esp_err_to_name(err), (int)g_e2_subscribed);
    }
}
