#ifndef BLE_CONDUCTOR_H
#define BLE_CONDUCTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Spike 0：从生产工程 motoguide1.43 (components/ble_peripheral_bsp) 移植的 BLE 外设。
 * 服务/特征 UUID、广播内容、MTU、连接参数全部照抄 —— App 已验证能连生产板，
 * 这里保持线上格式逐字节一致，App 零改动即可连接。
 *
 * 移植裁剪（相对生产版）：
 *   - 去掉 OTA GATTS 回调转发（本工程无 OTA 组件，单 app_id=0）
 *   - 去掉 boot_sequencer 启动时序依赖（低功率广播阶段，直接全功率）
 *   - 去掉 Screen3 设备名 NVS 编辑（固定 "Moto Guide"，与生产广播名一致）
 * 新增：
 *   - 内存账：init 各阶段打 内部RAM/最大连续块/DMA最大块（Spike 0 的核心产出）
 *   - 0xD0 对拍指令最小解析（segId/frameIdx/flags 暂存，供伺服联调用）
 *
 * ★ 初始化时机（生产 main.c:1164 的血泪注释）：esp_bt_controller_init() 会短暂
 *   禁用 PSRAM 缓存，若此刻其他任务正在写 PSRAM 帧缓冲，SPI DMA 会失败。
 *   → 必须在 app_main 最开头调用，早于一切 PSRAM 大流量任务。 */

/* 全链初始化：NVS → BT controller → Bluedroid → GATTS 注册 → 广播。
 * 返回 0 = 已走完同步部分（广播异步起，看串口 "Advertising started"）。 */
int ble_conductor_start(void);

bool ble_conductor_is_connected(void);
bool ble_conductor_is_e2_subscribed(void);   // 对端已写 CCCD 开通知

/* 收到对拍器指令（0xD0，10B）时的回调；NULL = 只走内置解析+日志 */
typedef void (*ble_conductor_write_cb_t)(const uint8_t *data, size_t len);
void ble_conductor_set_write_callback(ble_conductor_write_cb_t cb);

/* E2 通知（devStatus 特征，即对拍器 0xE3 回显的载体） */
void ble_conductor_send_notify(const uint8_t *data, size_t len);

/* 0xD0 对拍指令解析结果（内部自旋锁保护，跨核读安全）
 * ★ 2026-10-28 二轮意见①：对拍从「里程」改「段内帧号」（纯整数 1:1，无除法舍入）。
 *   线协议字节布局不变（10B），原 meter_m 字段语义替换为 frame_idx。 */
typedef struct {
    bool     valid;      // 收到过至少一条 0xD0
    uint32_t seg_id;     // LE u32
    uint32_t frame_idx;  // LE u32 目标段内帧号（收流顺序计数，与 gtwc::pop 的 localIdx 同源）
    uint8_t  flags;      // bit0=段重置 bit1=暂停 bit2=恢复
    uint32_t last_tick;  // 最近一次收到的 xTaskGetTickCount()（算静默时长用）
} ble_conductor_target_t;

/* 原子快照（播放任务调用；BTC 任务在写回调里更新，跨核必须走锁不能裸读） */
void ble_conductor_target_snapshot(ble_conductor_target_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONDUCTOR_H */
