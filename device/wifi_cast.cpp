// wifi_cast.cpp — WiFi 投屏收流端（STA/SoftAP + TCP 服务 + 16MB flash 桶式帧池）
//
// 【线协议】（手机 App 侧按此实现）：
//   v2（2026-10-28，docs/cast_pool_servo_plan.md）：TCP 连上后 App 先发会话头——
//     [magic u16 LE=0x5743('C','W')][ver u8=2][len u8=23]
//     [u16 w][u16 h][u8 mode][u16 flags]
//     [u32 segId][u32 segM0 米][u16 segStep 厘米/帧][u32 frameCount][u16 kfInterval]
//   v1（旧 App，保留兼容）：16B 'G''W''C''1' + w + h + fps + 6B rsv(0)。
//   设备先读 4B：magic=0x5743 → 按 len 读 v2；否则把 4B 当 v1 前缀（'G''W' 的
//   LE u16 = 0x5747，与 0x5743 无碰撞）补读 12B 走实时模式。
//   v2 flags bit0 = 帧压缩（2026-10-28 二轮意见③）：会话级开关，开启后每帧
//   [u32 len][u32 clen][u32 rawLen][zlib deflate(SCF1)]，len = clen+8。设备
//   inflate 进桶——桶里存的永远是解压后的 SCF1，解码/伺服路径零改动。无损。
//   之后每帧：u32 LE size + size 字节 SCF1 帧（27B 帧头 + run 流）。
//
// 【桶式帧池 → flash 后端】（2026-10-28 二轮意见②：PSRAM 4MB 太小，搬进 castpool
//   16MB 分区）64 桶 × 256KB，桶内连续多帧：[u32 bucketLen][帧0: u32 len+body][帧1]...
//   帧不跨桶；下一帧装不下 → 关桶发布、开新桶（keyframe ≤252KB 必然装得进空桶）。
//   ★ 存储层压缩（原方案的可选决策点，落地时发现是物理必选）：flash 擦除是硬墙——
//   4KB sector 擦 ~35ms，即使用 64KB 块擦（esp_partition_erase_range 对齐段自动
//   走块擦，吞吐 ~220KB/s），明文灌入也贴线；且桶重用（环回）每桶必擦 ~1s。
//   故桶里存【线上 body 原样】（压缩会话 = deflate 信封不 inflate 直接入桶），
//   pop() 读出后再 inflate 回明文给解码器——解码/伺服路径零改动，无损红线不破。
//   容量 16MB ≈ 320 帧（压缩后均值 ~50KB）≈ 21s @15fps，拐弯段痛点 ×3。
//   写路径：开桶先整桶擦（~1s，锁外做，期间 TCP 背压吸收），再逐帧 esp_partition_write。
//
// 【流控铁律】池满/擦除慢时不丢帧：收流任务停在等空桶/擦除上 → 本端不 recv → TCP 接收
//   窗口收窄 → App 侧 send 自然阻塞。
//
// 【并发模型】（评审复审①）桶内容读写两阶段永不同时：生产者独占「填桶阶段」
//   （整桶发布 = 推进 q_tail 一次），消费者独占「消费阶段」（桶内游标 c_off 逐帧
//   读，桶空推 q_head 一次）。共享状态只有 q_head/q_tail（+gen/clearReq）。
//   ★ tcp 任务 core0 / 播放任务 core1 —— 跨核是并行执行，FreeRTOS 切换屏障管不到
//   对方核的重排，所以共享游标一律走 portENTER_CRITICAL(&q_mux)（自旋锁入口自带
//   全内存屏障，等价 release/acquire；v1 同款约定已跨核跑通）。c_off 只归消费者。
//
// 【段重置】poolClearRequest() 置 s_clearReq；生产者在下一帧边界（q_reserve 里）
//   丢弃半满桶、q_head=q_tail=0、generation++。消费侧 poolGeneration() 看到 gen
//   变化才重置桶内游标——在此之前 pop 到的都是旧段帧（伺服在 RESET 态自行丢弃）。
//
// 【内存账】桶池 16MB（flash castpool 分区，不占 RAM）+ 收流/弹出暂存 3×257KB
//   （PSRAM）+ 解码滑窗环 657KB（nav_playback 分配）≈ 1.7MB，8MB Octal PSRAM 从
//   原桶池 4MB 里释放出 3.7MB。内部 RAM：WiFi/LWIP 压缩配置见 sdkconfig。
//
// 【线程】tcp 任务（core0 prio5）= 唯一生产者；播放任务（core1）= 唯一消费者。
#include "wifi_cast.h"

#include <cstring>
#include <cstdio>
#include <cerrno>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"     // castpool 分区桶池后端（erase/write/read）
#include "nvs_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <netinet/tcp.h>       // TCP_KEEPIDLE/KEEPINTVL/KEEPCNT（假死检测）
#include <zlib.h>              // 传输层帧解压（espressif__zlib 组件，PRIV_REQUIRES 已链）

namespace gtwc {

static const char* TAG = "gtamwc";

static constexpr const char* AP_SSID = "GTAM-CAST";
static constexpr const char* AP_PASS = "12345678";   // WPA2 要 ≥8 位
static constexpr uint16_t    TCP_PORT = 7123;
// ★ 10-28 实验：SoftAP 收方向实测只有 2~3Mbps（channel 6/11 无差），换 STA 连
//   手机热点——设备变 STA 后「手机→设备」方向变成 AP→STA 下行，且手机是网络
//   拥有者，不再有无网 churn。true=连手机热点；false=设备自开 SoftAP（旧路径保留）。
static constexpr bool        kUseStaHotspot = true;
static constexpr const char* STA_SSID = "your-hotspot-ssid";    // 手机热点（务必锁 2.4G）
static constexpr const char* STA_PASS = "your-hotspot-password";
static constexpr size_t      Q_SLOT   = 256u * 1024;              // 桶：4B 桶头 + 多帧
static constexpr uint32_t    Q_SLOTS  = 64;                       // 64 × 256KB = 16MB castpool 分区
static constexpr uint32_t    FRAME_MAX = Q_SLOT - 4 - 4;          // 单帧上限（帧长 + 帧内 4B 长度头都要装进空桶）
// 传输层压缩（2026-10-28 二轮意见③）：v2 会话头 flags bit0 = 帧走 deflate 信封
// [u32 clen][u32 rawLen][zlib(SCF1)]。★ 存储层压缩（二轮意见②落地时定案）：信封
// 不 inflate 直接入桶（flash 擦除吞吐硬墙，见文件头），pop() 读出后再解压——
// 无损 ⇒ 渲染零损失红线不破。信封线上长度上限 = rawLen 上限 + 信封头 +
// stored-block 最坏展开余量（不可压数据 deflate 存储块 ~+5B/64KB）；裸帧仍 ≤ FRAME_MAX。
static constexpr uint32_t    ENV_HEAD = 8;                        // 信封头：clen u32 + rawLen u32
static constexpr uint32_t    ENV_WIRE_MAX = FRAME_MAX + ENV_HEAD + 4096;
static uint8_t* s_rxBuf = nullptr;                                // 收流暂存 [len u32][body]（PSRAM）
static uint8_t* s_popRaw = nullptr;                               // pop 读桶原文暂存 [len u32][body]（PSRAM）
static uint8_t* s_popPlain = nullptr;                             // pop inflate 出缓冲（明文 SCF1，PSRAM）
static const esp_partition_t* s_part = nullptr;                   // castpool 分区句柄
static volatile bool s_zStore = false;                            // 本会话桶内存的是压缩信封（会话级）
static TaskHandle_t          s_playTask = nullptr;                // 消费者，整桶发布时 notify

// ---------------- 状态与会话（先声明——pop 里要用 s_bad） ----------------

static volatile State    s_state = State::Wait;
static Session           s_sess;     // 会话头（q_mux 保护；volatile 会废掉拷贝赋值）
static volatile uint32_t s_frames = 0;
static volatile uint32_t s_bad    = 0;
static volatile bool     s_firstFrameDone = false;   // 首帧前 RCVTIMEO 用宽限档

// ---------------- 桶式帧池（SPSC；桶内 [u32 bucketLen][u32 len][body]...；flash 后端） ----------------

static volatile uint32_t q_head = 0;            // 消费者桶序号
static volatile uint32_t q_tail = 0;            // 生产者桶序号
static portMUX_TYPE     q_mux   = portMUX_INITIALIZER_UNLOCKED;
// 清池代号（generation）：段重置时生产者 ++，消费者对比感知
static volatile uint32_t s_gen      = 0;
static volatile bool     s_clearReq = false;
// 生产者私有（只有 tcp 任务碰；clear 在写路径里自处理）
static int      p_curBkt = -1;                  // 正在填的桶（-1=无）
static uint32_t p_curOff = 0;                   // 桶内写偏移（4 起步，0..3 是 bucketLen）
// 消费者私有（只有播放任务碰）
static uint32_t c_gen    = 0;
static uint32_t c_off    = 4;                   // 桶内读偏移
static uint32_t c_idx    = 0;                   // 段内帧号计数（清池归零）
static uint32_t c_bktLen = 0xFFFFFFFF;          // 当前桶已发布长度（进入新桶时从 flash 读一次）

static inline uint32_t bucketOff(int b) { return (uint32_t)b * (uint32_t)Q_SLOT; }
static inline bool q_fullLocked()  { return ((q_tail + 1) % Q_SLOTS) == q_head; }
static inline bool q_emptyLocked() { return q_tail == q_head; }
static inline uint32_t rd32b(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
// SCF1 帧头 flags 偏移（= core/cast.h kCastOffFlags；这里不 include core 头，避免耦合）
static constexpr int SCF1_OFF_FLAGS = 2;

void poolClearRequest() {
    portENTER_CRITICAL(&q_mux);
    s_clearReq = true;
    portEXIT_CRITICAL(&q_mux);
}

uint32_t poolGeneration() {
    portENTER_CRITICAL(&q_mux);
    const uint32_t g = s_gen;
    portEXIT_CRITICAL(&q_mux);
    return g;
}

// 生产者帧边界：处理清池请求（丢弃半满桶 + 清队列 + gen++）。
// 必须持 q_mux 调用。★ 放在 q_reserve 的重试循环里 = 每帧边界 + 池满等待期间都检查。
static void producer_handle_clear_locked() {
    if (s_clearReq) {
        s_clearReq = false;
        q_head = 0; q_tail = 0;
        s_gen++;
        p_curBkt = -1; p_curOff = 0;
        ESP_LOGW(TAG, "池已清（段重置 gen=%u）", (unsigned)s_gen);
    }
}

// 生产者：为下一帧决策落点。返回值：0=写当前桶 / 1=关桶发布后开新桶 / -1=池满等待。
// 必须持 q_mux 调用。★ 放在写循环里 = 每帧边界 + 池满等待期间都检查清池请求。
static int producer_decide_locked(uint32_t len, int* newBkt) {
    producer_handle_clear_locked();
    *newBkt = -1;
    if (p_curBkt < 0) {
        // 开新桶（空桶保证装得下任何合法帧：len+4 ≤ FRAME_MAX+4 < Q_SLOT）
        if (!q_fullLocked()) { *newBkt = (int)q_tail; return 1; }
        return -1;
    }
    if (p_curOff + 4 + len > Q_SLOT) {
        // 关桶发布：bucketLen = 已写帧字节总数（不含 4B 桶头）
        if (!q_fullLocked()) { *newBkt = (int)((q_tail + 1) % Q_SLOTS); return 1; }
        return -1;   // 满则本轮只等（关桶推迟到有空桶时）
    }
    return 0;        // 当前桶还装得下
}

// 生产者：关桶发布。bucketLen 写 flash（桶未发布，生产者独占，无需锁）→ 锁内推 q_tail。
static void producer_publish_bucket() {
    const uint32_t bl = p_curOff - 4;
    const uint8_t b4[4] = { (uint8_t)bl, (uint8_t)(bl >> 8), (uint8_t)(bl >> 16), (uint8_t)(bl >> 24) };
    esp_partition_write(s_part, bucketOff(p_curBkt), b4, 4);
    portENTER_CRITICAL(&q_mux);
    q_tail = (q_tail + 1) % Q_SLOTS;
    if (s_playTask) xTaskNotifyGive(s_playTask);
    p_curBkt = -1;
    portEXIT_CRITICAL(&q_mux);
}

// 生产者：把一帧写入 flash 桶。s_rxBuf 布局 = [u32 len LE][body len 字节]（调用方布置）。
// 满则阻塞等待（背压）；开新桶先整桶擦（锁外 ~1s，擦除期间 TCP 背压吸收）。
static void q_write_frame(uint32_t len, uint32_t* waitMs) {
    const int64_t t0 = esp_timer_get_time();
    for (;;) {
        int newBkt = -1;
        portENTER_CRITICAL(&q_mux);
        const int decision = producer_decide_locked(len, &newBkt);
        portEXIT_CRITICAL(&q_mux);

        if (decision < 0) {          // 池满：等消费者（背压）
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (p_curBkt >= 0 && newBkt < 0) {
            // 决策=写当前桶（decision==0）——直接写 [len][body]（已在 s_rxBuf 连续）
            esp_partition_write(s_part, bucketOff(p_curBkt) + p_curOff, s_rxBuf, 4 + len);
            p_curOff += 4 + len;
            if (waitMs) *waitMs = (uint32_t)((esp_timer_get_time() - t0) / 1000);
            return;
        }
        // decision==1：关旧桶（若有）→ 擦新桶（锁外）→ 锁内确认归属 → 开桶
        if (p_curBkt >= 0) producer_publish_bucket();
        const int64_t tE0 = esp_timer_get_time();
        esp_err_t er = esp_partition_erase_range(s_part, bucketOff(newBkt), Q_SLOT);
        if (er != ESP_OK) {
            ESP_LOGE(TAG, "桶 %d 擦除失败 %s", newBkt, esp_err_to_name(er));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;                // 下轮重决策重擦
        }
        const uint32_t eraseMs = (uint32_t)((esp_timer_get_time() - tE0) / 1000);
        bool opened = false;
        portENTER_CRITICAL(&q_mux);
        producer_handle_clear_locked();
        if (p_curBkt < 0 && (int)q_tail == newBkt && !q_fullLocked()) {
            p_curBkt = newBkt;
            p_curOff = 4;
            opened = true;
        }
        portEXIT_CRITICAL(&q_mux);
        if (opened) {
            ESP_LOGI(TAG, "开桶 %d（擦 %ums）", newBkt, (unsigned)eraseMs);
            continue;                // 下轮走「写当前桶」路径
        }
        // clear 竞争/又满了 → 重走决策（擦除白做一次，无害）
    }
}

bool pop(uint32_t timeoutMs, const uint8_t** data, uint32_t* size, uint32_t* seq,
         bool* isKey, uint32_t* localIdx) {
    if (!s_part) return false;
    if (s_playTask == nullptr) s_playTask = xTaskGetCurrentTaskHandle();
    const TickType_t t0 = xTaskGetTickCount();
    for (;;) {
        bool has = false;
        const uint8_t* fp = nullptr;
        uint32_t flen = 0;
        portENTER_CRITICAL(&q_mux);
        // gen 对齐：清池生效（生产者已 gen++ 且清了队列）→ 重置消费游标
        if (s_gen != c_gen) { c_gen = s_gen; c_off = 4; c_idx = 0; c_bktLen = 0xFFFFFFFF; }
        const bool empty = q_emptyLocked();
        const uint32_t head = q_head;
        portEXIT_CRITICAL(&q_mux);
        if (!empty) {
            // 桶内容已发布（q_tail 越过），生产者不可能回写——锁外慢速 IO 安全。
            // 桶长懒读：进入新桶时从 flash 读一次 bucketLen
            if (c_bktLen == 0xFFFFFFFF) {
                uint8_t b4[4];
                if (esp_partition_read(s_part, bucketOff((int)head), b4, 4) != ESP_OK) {
                    ESP_LOGW(TAG, "pop: 桶头读失败，跳桶");
                    portENTER_CRITICAL(&q_mux);
                    q_head = (q_head + 1) % Q_SLOTS;
                    portEXIT_CRITICAL(&q_mux);
                    c_off = 4; c_bktLen = 0xFFFFFFFF;
                    continue;
                }
                c_bktLen = rd32b(b4);
            }
            if (c_off >= c_bktLen + 4) {
                // 桶尽 → 推进 q_head
                portENTER_CRITICAL(&q_mux);
                q_head = (q_head + 1) % Q_SLOTS;
                portEXIT_CRITICAL(&q_mux);
                c_off = 4; c_bktLen = 0xFFFFFFFF;
                continue;
            }
            // 读 [u32 len][body] 一次进 s_popRaw
            const uint32_t room = (Q_SLOT - c_off < 4 + FRAME_MAX) ? Q_SLOT - c_off : 4 + FRAME_MAX;
            if (esp_partition_read(s_part, bucketOff((int)head) + c_off, s_popRaw, room) != ESP_OK) {
                ESP_LOGW(TAG, "pop: 帧读失败，跳桶");
                portENTER_CRITICAL(&q_mux);
                q_head = (q_head + 1) % Q_SLOTS;
                portEXIT_CRITICAL(&q_mux);
                c_off = 4; c_bktLen = 0xFFFFFFFF;
                continue;
            }
            flen = rd32b(s_popRaw);
            // 协议错（发送侧限长双保险，理论上到不了）——弃整桶计数
            if (flen == 0 || flen > FRAME_MAX || c_off + 4 + flen > c_bktLen + 4) {
                s_bad++;
                portENTER_CRITICAL(&q_mux);
                q_head = (q_head + 1) % Q_SLOTS;
                portEXIT_CRITICAL(&q_mux);
                c_off = 4; c_bktLen = 0xFFFFFFFF;
                continue;
            }
            // 存储层压缩：桶内是 deflate 信封 → 解压到 s_popPlain 返回明文
            if (s_zStore) {
                if (flen < ENV_HEAD + 27) {
                    s_bad++;
                    portENTER_CRITICAL(&q_mux);
                    q_head = (q_head + 1) % Q_SLOTS;
                    portEXIT_CRITICAL(&q_mux);
                    c_off = 4; c_bktLen = 0xFFFFFFFF;
                    continue;
                }
                const uint32_t clen   = rd32b(s_popRaw + 4);
                const uint32_t rawLen = rd32b(s_popRaw + 8);
                uLongf dLen = rawLen;
                if (clen != flen - ENV_HEAD || rawLen < 27 || rawLen > FRAME_MAX ||
                        uncompress(s_popPlain, &dLen, s_popRaw + 4 + ENV_HEAD, clen) != Z_OK
                        || dLen != rawLen) {
                    ESP_LOGW(TAG, "pop: 信封解压失败 clen=%u rawLen=%u，跳帧", (unsigned)clen, (unsigned)rawLen);
                    s_bad++;
                    // 跳过本帧（推进游标），不弃桶——后续帧可能还健康
                    c_off += 4 + flen;
                    if (c_off >= c_bktLen + 4) {
                        portENTER_CRITICAL(&q_mux);
                        q_head = (q_head + 1) % Q_SLOTS;
                        portEXIT_CRITICAL(&q_mux);
                        c_off = 4; c_bktLen = 0xFFFFFFFF;
                    }
                    continue;
                }
                fp = s_popPlain;
                flen = rawLen;
            } else {
                fp = s_popRaw + 4;
            }
            has = true;
        }

        if (has) {
            *data = fp;
            *size = flen;
            // seq 在 SCF1 帧头 kCastOffSeq=17（LE u32），供断流/丢帧诊断
            if (seq) {
                *seq = (uint32_t)fp[17] | ((uint32_t)fp[18] << 8) |
                       ((uint32_t)fp[19] << 16) | ((uint32_t)fp[20] << 24);
            }
            if (isKey)    *isKey    = (fp[SCF1_OFF_FLAGS] & 1) != 0;
            if (localIdx) *localIdx = c_idx;
            // 消费推进（帧边界）：桶内游标 + 桶尽推 q_head + 段内帧号
            const uint32_t adv = 4 + rd32b(s_popRaw);   // 桶内原样步进（压缩帧按信封长度）
            portENTER_CRITICAL(&q_mux);
            c_off += adv;
            if (c_bktLen != 0xFFFFFFFF && c_off >= c_bktLen + 4) {
                q_head = (q_head + 1) % Q_SLOTS;
                c_off = 4;
                c_bktLen = 0xFFFFFFFF;
            }
            c_idx++;
            portEXIT_CRITICAL(&q_mux);
            return true;
        }
        if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(timeoutMs)) return false;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ---------------- 状态与会话（声明已在队列段之前） ----------------

State    state()            { return s_state; }
bool     session(Session* out) {
    portENTER_CRITICAL(&q_mux);   // 单写者 tcp_task / 单读者播放任务，临界区兜住撕裂
    *out = s_sess;
    portEXIT_CRITICAL(&q_mux);
    return s_sess.w != 0;
}
uint32_t framesRecv()       { return s_frames; }
uint32_t framesBad()        { return s_bad; }

static char s_ipText[16] = "0.0.0.0";   // SoftAP=起手即真值；STA=DHCP 到手后事件刷新
const char* ipText()        { return s_ipText; }
const char* netSsid()       { return kUseStaHotspot ? STA_SSID : AP_SSID; }
bool        staMode()       { return kUseStaHotspot; }

// ---------------- WiFi（SoftAP / STA 连手机热点 双模式） ----------------

static void wifi_event_handler(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_AP_STACONNECTED) {
            ESP_LOGI(TAG, "STA 接入 SoftAP");
        } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
            ESP_LOGW(TAG, "STA 离开 SoftAP");
        } else if (id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "STA 启动，连接热点...");
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            // iPhone 热点空闲会被系统收走/重置——必须自动重连才可用。
            // 2s 退避（在事件循环里 delay 只影响本实验，可接受），避免热点没开时刷屏重试。
            ESP_LOGW(TAG, "热点断开/未连上，2s 后重试");
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* ev = static_cast<ip_event_got_ip_t*>(data);
        esp_ip4addr_ntoa(&ev->ip_info.ip, s_ipText, sizeof(s_ipText));
        ESP_LOGI(TAG, "热点 DHCP 到手 IP %s（App 连这个地址的端口 %u）", s_ipText, TCP_PORT);
    }
}

static int wifi_ap_start(void) {
    // NVS（WiFi 校准数据仓库）；第一次跑可能没格式化，擦了重来
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, nullptr));

    // ★ AP 模式配置走 union 的 .ap 成员（不是 .sta——password/ssid_len 偏移不同）
    wifi_config_t ap = {};
    std::strncpy(reinterpret_cast<char*>(ap.ap.ssid), AP_SSID, sizeof(ap.ap.ssid) - 1);
    std::strncpy(reinterpret_cast<char*>(ap.ap.password), AP_PASS, sizeof(ap.ap.password) - 1);
    ap.ap.ssid_len = std::strlen(AP_SSID);
    ap.ap.channel = 11;   // 10-28：6→11（channel 6 是 2.4G 最挤默认频点；实测对比吞吐）
    ap.ap.max_connection = 2;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    // ★ 锁 HT20（2026-10-28）：channel 6 是 2.4G 最挤默认频点，40MHz 下一次重传
    //   双倍占空口，干扰环境实测吞吐反而更差。HT20 最大 PHY 72.2Mbps，仍够
    //   24Mbps 需求（瓶颈本就不在 PHY，见 debug.log 分析）。
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 把实际 IP 抠出来（SoftAP 默认 192.168.4.1，但读真值画屏，别硬编码）
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip = {};
    if (netif) {
        esp_netif_get_ip_info(netif, &ip);
        esp_ip4addr_ntoa(&ip.ip, s_ipText, sizeof(s_ipText));
    }
    ESP_LOGI(TAG, "SoftAP %s/%s  IP %s  端口 %u",
             AP_SSID, AP_PASS, s_ipText, TCP_PORT);
    return 0;
}

// STA 模式：设备连手机热点（iPhone）。TCP 仍在本机 :7123 监听，App 连设备 IP。
// IP 由热点 DHCP 异步发放（iPhone 热点网段 172.20.10.x），到手后事件里刷新 s_ipText。
static int wifi_sta_start(void) {
    // NVS（WiFi 校准数据仓库）；第一次跑可能没格式化，擦了重来
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, nullptr));

    // STA 配置走 union 的 .sta 成员
    wifi_config_t sta = {};
    std::strncpy(reinterpret_cast<char*>(sta.sta.ssid), STA_SSID, sizeof(sta.sta.ssid) - 1);
    std::strncpy(reinterpret_cast<char*>(sta.sta.password), STA_PASS, sizeof(sta.sta.password) - 1);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    // ★ 关 STA 省电（modem sleep 是持续收流的延迟/吞吐杀手）；SoftAP 模式无此问题
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "STA 连热点 %s/%s ...（IP 等 DHCP）", STA_SSID, STA_PASS);
    return 0;
}

// ---------------- TCP 收流任务 ----------------

// 精确读 n 字节（阻塞；对端断开/出错返回 false）
static bool recv_exact(int fd, void* buf, size_t n) {
    auto* p = static_cast<uint8_t*>(buf);
    size_t got = 0;
    while (got < n) {
        const ssize_t r = recv(fd, p + got, n - got, 0);
        if (r <= 0) return false;      // 0=对端关，-1=错
        got += (size_t)r;
    }
    return true;
}

static void tcp_task(void*) {
    // ★ 屏幕状态页由播放任务画（单写者纪律：所有 blit 只从播放任务发）；这里只管收帧
    for (;;) {   // accept 循环：一次服务一个手机，断开再等下一个
        const int lfd = socket(AF_INET, SOCK_STREAM, 0);
        if (lfd < 0) { ESP_LOGE(TAG, "socket 失败 errno=%d", errno); vTaskDelay(1000); continue; }
        int on = 1;
        setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(TCP_PORT);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(lfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
            listen(lfd, 1) != 0) {
            ESP_LOGE(TAG, "bind/listen 失败 errno=%d", errno);
            close(lfd);
            vTaskDelay(1000);
            continue;
        }
        ESP_LOGI(TAG, "TCP 监听 :%u，等手机...", TCP_PORT);

        for (;;) {
            sockaddr_in cli = {};
            socklen_t cl = sizeof(cli);
            const int fd = accept(lfd, reinterpret_cast<sockaddr*>(&cli), &cl);
            if (fd < 0) { vTaskDelay(500); continue; }
            ESP_LOGI(TAG, "手机已连 %s:%u", inet_ntoa(cli.sin_addr), (unsigned)ntohs(cli.sin_port));

            // ★ 诊断口径按会话归零（否则第二会话一上来就带旧累计值，log 不可读）
            s_frames = 0; s_bad = 0;
            s_firstFrameDone = false;   // 新会话重新进入首帧宽限档

            // ★ TCP 假死保护：keepalive 15s 空闲后每 2s 探一次、2 次无应答判死。
            //   IDLE 不能配 5s——App「连接→首帧」要读 7MB 地图+解析（静默 ~9s+），
            //   实测 5s 档把健康连接杀了（debug.log 两轮 8~9s 死会话）。
            //   RCVTIMEO 同理：首帧前 30s 宽限（覆盖读图+解析），收到首帧后收紧 10s。
            int ka = 1;
            setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
            int idle = 15, intvl = 2, cnt = 2;
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
            setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
            struct timeval rto = { 30, 0 };
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof(rto));

            s_state = State::Wait;     // 连上但会话头未到

            // ① 会话头：先读 4B 探版本（TCP 流无边界，v2 必须自带长度前缀——评审⑤）
            //    v2 magic = 0x5743（LE 字节 'C','W'）；v1 首两字节 'G','W' = 0x5747 无碰撞
            uint8_t h4[4];
            if (!recv_exact(fd, h4, 4)) { close(fd); continue; }
            const uint16_t magic = (uint16_t)(h4[0] | (h4[1] << 8));
            Session sess;
            if (magic == 0x5743) {
                const uint8_t ver = h4[2], hlen = h4[3];
                if (ver != 2 || hlen != 23) {
                    ESP_LOGW(TAG, "会话头 v2 版本/长度非法 ver=%u len=%u，断开", ver, hlen);
                    close(fd);
                    continue;
                }
                uint8_t body[23];
                if (!recv_exact(fd, body, sizeof(body))) { close(fd); continue; }
                sess.isV2       = true;
                sess.w          = (uint16_t)(body[0] | (body[1] << 8));
                sess.h          = (uint16_t)(body[2] | (body[3] << 8));
                sess.mode       = body[4];
                sess.flags      = (uint16_t)(body[5] | (body[6] << 8));
                sess.segId      = rd32b(body + 7);
                sess.segM0m     = rd32b(body + 11);
                sess.segStepCm  = (uint16_t)(body[15] | (body[16] << 8));
                sess.frameCount = rd32b(body + 17);
                sess.kfInterval = (uint16_t)(body[21] | (body[22] << 8));
                if (sess.segStepCm == 0) {
                    ESP_LOGW(TAG, "v2 segStep=0 非法，断开");
                    close(fd);
                    continue;
                }
                ESP_LOGI(TAG, "会话 v2 %ux%u mode=%u seg=%u m0=%u step=%ucm 帧%u kf%u",
                         sess.w, sess.h, sess.mode, (unsigned)sess.segId,
                         (unsigned)sess.segM0m, sess.segStepCm,
                         (unsigned)sess.frameCount, sess.kfInterval);
            } else {
                // v1：把 4B 当作 16B 头的前缀，补读 12B
                uint8_t hdr[16];
                std::memcpy(hdr, h4, 4);
                if (!recv_exact(fd, hdr + 4, 12) || std::memcmp(hdr, "GWC1", 4) != 0) {
                    ESP_LOGW(TAG, "会话头非法（非 v2 magic / 非 GWC1），断开");
                    close(fd);
                    continue;
                }
                sess.w   = (uint16_t)(hdr[4] | (hdr[5] << 8));
                sess.h   = (uint16_t)(hdr[6] | (hdr[7] << 8));
                sess.fps = (uint16_t)(hdr[8] | (hdr[9] << 8));
                ESP_LOGI(TAG, "会话 v1 %ux%u @%ufps", sess.w, sess.h, sess.fps);
            }
            portENTER_CRITICAL(&q_mux);
            s_sess = sess;
            s_zStore = sess.isV2 && (sess.flags & 0x01) != 0;   // 桶内存信封（pop 时解压）
            q_head = 0; q_tail = 0;     // ★ 新会话清池：残留上一轮的旧帧会串台重放
            s_gen++;                    // 消费侧同步归零（帧号计数 + 桶内游标）
            s_clearReq = false;
            portEXIT_CRITICAL(&q_mux);
            p_curBkt = -1; p_curOff = 0;   // 生产者侧丢弃半满桶（生产者私有，直接重置）
            s_state = State::Link;
            // 传输层压缩开关（会话级单点判定，无逐帧魔数歧义）
            const bool zFrames = sess.isV2 && (sess.flags & 0x01) != 0;
            if (zFrames) ESP_LOGI(TAG, "帧压缩开（信封入桶存储层压缩，pop 时解压）");

            // ② 帧流：[u32 LE size][body]——body 先收进 s_rxBuf 再原样入桶
            //    （存储层压缩：信封不 inflate 直接入桶，pop 时解压；裸会话 body=SCF1）
            //    探针：每 5s 聚合一次收流账（等空槽+擦除 / 套接字读）
            bool alive = true;
            uint32_t wFrames = 0, wBytes = 0, wQWaitSum = 0, wQWaitMax = 0, wRecvSum = 0;
            int64_t wT0 = esp_timer_get_time();
            while (alive) {
                const int64_t tRx0 = esp_timer_get_time();
                uint8_t lb[4];
                if (!recv_exact(fd, lb, 4)) { alive = false; break; }
                const uint32_t len = (uint32_t)lb[0] | ((uint32_t)lb[1] << 8) |
                                     ((uint32_t)lb[2] << 16) | ((uint32_t)lb[3] << 24);
                if (len == 0 || len > ENV_WIRE_MAX) {
                    ESP_LOGW(TAG, "帧长越界 %u，断开", (unsigned)len);
                    alive = false;
                    break;
                }
                if (zFrames && len < ENV_HEAD + 27) {   // 信封至少 8B 头 + 27B SCF1 帧头
                    ESP_LOGW(TAG, "压缩信封过短 %u，断开", (unsigned)len);
                    alive = false;
                    break;
                }
                if (!zFrames && len > FRAME_MAX) {
                    ESP_LOGW(TAG, "裸帧超上限 %u，断开", (unsigned)len);
                    alive = false;
                    break;
                }
                if (!recv_exact(fd, s_rxBuf + 4, len)) { alive = false; break; }
                // 帧长头进 s_rxBuf[0..4)（桶内 [u32 len][body] 连续，一次写 flash）
                s_rxBuf[0] = lb[0]; s_rxBuf[1] = lb[1]; s_rxBuf[2] = lb[2]; s_rxBuf[3] = lb[3];
                if (zFrames) {
                    // 入桶前校验信封（早断腐败流；不解压——存储层压缩）
                    const uint32_t clen   = rd32b(s_rxBuf + 4);
                    const uint32_t rawLen = rd32b(s_rxBuf + 8);
                    if (clen != len - ENV_HEAD || rawLen < 27 || rawLen > FRAME_MAX) {
                        ESP_LOGW(TAG, "信封校验失败 clen=%u rawLen=%u len=%u，断开",
                                 (unsigned)clen, (unsigned)rawLen, (unsigned)len);
                        alive = false;
                        break;
                    }
                }
                uint32_t qWait = 0;
                q_write_frame(len, &qWait);              // 满/擦除则阻塞=背压
                if (!s_firstFrameDone) {
                    // 首帧已到 = App 初始化结束，假死保护收紧到稳流档
                    s_firstFrameDone = true;
                    struct timeval rto10 = { 10, 0 };
                    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rto10, sizeof(rto10));
                    ESP_LOGI(TAG, "首帧到手，假死保护收紧为 10s");
                }
                s_frames++;
                s_state = State::Play;
                // 聚合：qWait=等空桶+擦除（生产停滞证据）；recvSum=套接字收数据总耗时
                wFrames++; wBytes += len + 4;
                wQWaitSum += qWait; if (qWait > wQWaitMax) wQWaitMax = qWait;
                wRecvSum += (uint32_t)((esp_timer_get_time() - tRx0) / 1000) - qWait;
                if ((esp_timer_get_time() - wT0) >= 5000000) {
                    ESP_LOGI(TAG, "recv 5s: 帧%u %uKB%s | 等桶+擦 累%ums 峰%ums | "
                             "recv内耗累%ums | 深度%d",
                             (unsigned)wFrames, (unsigned)(wBytes / 1024),
                             zFrames ? "（信封直存）" : "",
                             (unsigned)wQWaitSum, (unsigned)wQWaitMax,
                             (unsigned)wRecvSum, depth());
                    wFrames = 0; wBytes = 0; wQWaitSum = 0; wQWaitMax = 0; wRecvSum = 0;
                    wT0 = esp_timer_get_time();
                }
            }
            ESP_LOGW(TAG, "连接结束（累计收 %u 帧，坏 %u）",
                     (unsigned)s_frames, (unsigned)s_bad);
            s_state = State::Wait;     // 回等待页（播放任务感知后重画）
            close(fd);
        }
    }
}

int depth() {
    portENTER_CRITICAL(&q_mux);
    const int d = (int)((q_tail + Q_SLOTS - q_head) % Q_SLOTS);
    portEXIT_CRITICAL(&q_mux);
    return d;
}

// ---------------- iperf 精简 server（:5001，测裸管道吞吐） ----------------
// 与官方 iperf2 TCP 客户端兼容：客户端连上即灌数据，本端只吞不吐、5s 打点、
// 断开打总账（客户端自己也会报告发送速率，两端互为印证）。
// 目的：把「无线链路裸吞吐」从推流链路（渲染+编码+会话）里剥出来单独定量——
// iperf 都跑不动的管道，推流侧任何优化都是白费。
static void iperf_task(void*) {
    // ★ 吞流缓冲必须放 PSRAM：放内部 RAM（static BSS）实测吃掉 8KB 后屏幕 SPI
    //   DMA 私有 TX 缓冲分配失败（ESP_ERR_NO_MEM 刷屏、DMA 池 24→12KB）。
    //   recv 目的地无 DMA 约束（拷贝发生在 lwip，WiFi 包已在驱动侧缓冲），PSRAM 无碍。
    auto* sink = static_cast<uint8_t*>(heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!sink) { ESP_LOGE(TAG, "[iperf] sink 分配失败"); vTaskDelete(nullptr); return; }
    for (;;) {
        const int lfd = socket(AF_INET, SOCK_STREAM, 0);
        if (lfd < 0) { ESP_LOGE(TAG, "[iperf] socket 失败 errno=%d", errno); vTaskDelay(1000); continue; }
        int on = 1;
        setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(5001);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(lfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
            listen(lfd, 1) != 0) {
            ESP_LOGE(TAG, "[iperf] bind/listen 失败 errno=%d", errno);
            close(lfd);
            vTaskDelay(1000);
            continue;
        }
        ESP_LOGI(TAG, "[iperf] 监听 :5001（App「iperf测速」或 iperf2 客户端均可）");
        for (;;) {
            sockaddr_in cli = {};
            socklen_t cl = sizeof(cli);
            const int fd = accept(lfd, reinterpret_cast<sockaddr*>(&cli), &cl);
            if (fd < 0) { vTaskDelay(500); continue; }
            ESP_LOGI(TAG, "[iperf] 灌流开始 %s", inet_ntoa(cli.sin_addr));
            uint64_t total = 0, markBytes = 0;
            int64_t t0 = esp_timer_get_time(), tMark = t0;
            for (;;) {
                const ssize_t r = recv(fd, sink, sizeof(sink), 0);
                if (r <= 0) break;
                total += (uint64_t)r; markBytes += (uint64_t)r;
                if (esp_timer_get_time() - tMark >= 5000000) {
                    const double s = (esp_timer_get_time() - tMark) / 1e6;
                    ESP_LOGI(TAG, "[iperf] 5s: %uKB (%.1f Mbps)",
                             (unsigned)(markBytes / 1024), markBytes * 8.0 / s / 1e6);
                    tMark = esp_timer_get_time(); markBytes = 0;
                }
            }
            const double secs = (esp_timer_get_time() - t0) / 1e6;
            ESP_LOGW(TAG, "[iperf] 灌流结束 总 %uKB / %.1fs = %.2f Mbps",
                     (unsigned)(total / 1024), secs, secs > 0 ? total * 8.0 / secs / 1e6 : 0.0);
            close(fd);
        }
    }
}

// ---------------- 诊断任务：CPU 画像 + 内存体检（每 5s） ----------------
// 零 ACK 之谜的终极判决：停顿窗口里看 tcpip/WiFi/各自任务谁在吃核（饥饿）
// 还是全员空闲（阻塞）——两种病两种修法。
static void diag_task(void*) {
    static char buf[4096];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "[diag] 内部%uKB(最大%u) DMA%uKB(最大%u) PSRAM余%uKB 队列深%d",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                 (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024),
                 (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_DMA) / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                 depth());
#if CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        vTaskGetRunTimeStats(buf);
        // 只打前 12 行（含 IDLE0/IDLE1——各自核的空闲度），按行拆避免超长行被截
        char* line = buf;
        int n = 0;
        for (char* p = buf; *p && n < 12; ++p) {
            if (*p == '\n') { *p = 0; ESP_LOGI(TAG, "[cpu] %s", line); line = p + 1; ++n; }
        }
#else
        ESP_LOGI(TAG, "[diag] 运行时统计未启用");
#endif
    }
}

int start() {
    if (s_part) return 0;   // 幂等
    // flash 桶池后端：castpool 分区（data/spiffs 16M，partitions.csv）
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "castpool");
    if (!s_part) {
        ESP_LOGE(TAG, "castpool 分区找不到——检查 partitions.csv 是否烧进设备（分区表变更需全量重烧）");
        return -1;
    }
    if (s_part->size < Q_SLOTS * Q_SLOT) {
        ESP_LOGE(TAG, "castpool 分区 %uKB < 桶池 %uKB", (unsigned)(s_part->size / 1024),
                 (unsigned)(Q_SLOTS * Q_SLOT / 1024));
        return -1;
    }
    // 收流暂存 + pop 读原文暂存 + pop 解压出缓冲（PSRAM；替代原 4MB 桶池，净省 ~3.5MB）
    s_rxBuf    = static_cast<uint8_t*>(heap_caps_malloc(4 + FRAME_MAX + 64,
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_popRaw   = static_cast<uint8_t*>(heap_caps_malloc(4 + FRAME_MAX + 64,
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_popPlain = static_cast<uint8_t*>(heap_caps_malloc(FRAME_MAX + 64,
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!s_rxBuf || !s_popRaw || !s_popPlain) {
        ESP_LOGE(TAG, "收流/弹出暂存缓冲分配失败");
        return -1;
    }
    ESP_LOGI(TAG, "桶池 %u桶×%uKB=%uMB @flash(castpool 0x%06x) + 暂存 3×%uKB @PSRAM"
             "（分配后 SPIRAM 余 %u KB）",
             (unsigned)Q_SLOTS, (unsigned)(Q_SLOT / 1024),
             (unsigned)(Q_SLOTS * Q_SLOT / 1024 / 1024),
             (unsigned)s_part->address, (unsigned)((4 + FRAME_MAX) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    if (kUseStaHotspot ? wifi_sta_start() != 0 : wifi_ap_start() != 0) return -1;

    // 8KB 栈：收流路径只做 recv→槽位直写，无递归无大局部缓冲（2026-10-28 从 16K 省下
    // 8KB 内部 RAM——diag 实测最大连续块仅 8KB，WiFi 动态 rx pbuf 分配失败是丢包主因）
    if (xTaskCreatePinnedToCore(tcp_task, "gtam_tcp", 8192, nullptr, 5, nullptr, 0) != pdPASS) {
        ESP_LOGE(TAG, "tcp 任务创建失败");
        return -1;
    }
    // 诊断任务钉 core0 prio1（最低）：若 core0 被吃满，它的日志间隔会自己拉长——
    // 日志间隙本身就是证据
    if (xTaskCreatePinnedToCore(diag_task, "gtam_diag", 4096, nullptr, 1, nullptr, 0) != pdPASS) {
        ESP_LOGW(TAG, "diag 任务创建失败（诊断降级）");
    }
    // iperf 精简 server（:5001）：平时 idle，只在有人灌流时收包——裸管道吞吐定量
    if (xTaskCreatePinnedToCore(iperf_task, "gtam_iperf", 4096, nullptr, 3, nullptr, 0) != pdPASS) {
        ESP_LOGW(TAG, "iperf 任务创建失败（测速降级）");
    }
    return 0;
}

} // namespace gtwc
