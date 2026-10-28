// wifi_cast.h — WiFi 投屏收流端对外接口（02_lvgl_demo_v9）
//
// v2（2026-10-28，投屏方案 docs/cast_pool_servo_plan.md）：
//   - 队列「桶式」+ flash 后端（二轮意见②）：64 桶 × 256KB = 16MB @castpool 分区，
//     桶内连续多帧（帧不跨桶）；桶内存【线上 body 原样】（压缩会话=deflate 信封，
//     存储层压缩），pop() 解压回明文——解码/伺服零改动
//   - 会话头 v2：magic+ver+len 前缀，mode 分流（0=实时 1=池伺服）
//   - 段重置清池接口（generation 计数，收流/播放两侧对齐）
//   - pop() 扩展 isKey/localIdx 出参（伺服追帧用），v1 调用点零改动
// 本文件只管"收帧进池"；解码/推屏/伺服在 nav_playback.cpp（唯一 blit 写者）。
#pragma once

#include <cstdint>
#include <cstddef>

namespace gtwc {

enum class State {
    Wait,   // 等手机连
    Link,   // TCP 已连 + 会话头已收，等首帧
    Play,   // 帧流已到
};

// 会话头。v1 = 16B 'GWC1'{w,h,fps,rsv}；v2 = magic+ver+len 前缀 + 23B 字段。
// ★ v1/v2 共存：设备按 magic 分流（见 wifi_cast.cpp tcp_task），旧 App 发 v1 照常工作。
struct Session {
    // v1 字段（实时模式仍用）
    uint16_t w   = 0;
    uint16_t h   = 0;
    uint16_t fps = 0;

    // ---- v2 扩展（isV2=false 时以下全部无效）----
    bool     isV2       = false;
    uint8_t  mode       = 0;     // 0=实时推流（pop 驱动） 1=池伺服（BLE 对拍器驱动）
    uint16_t flags      = 0;     // bit0=帧压缩（deflate 信封，2026-10-28 二轮意见③）；
                                 // 其余预留
    uint32_t segId      = 0;     // 段号（偏航重算 = 新段）
    uint32_t segM0m     = 0;     // 段起始里程（米）；帧 i 里程 = segM0m + i*segStepCm/100
    uint16_t segStepCm  = 0;     // 帧间隔（厘米/帧）：15fps@60km/h ≈ 110
    uint32_t frameCount = 0;     // 段内总帧数（App 声明，目标帧号 clamp 用）
    uint16_t kfInterval = 0;     // keyframe 间隔（帧；0=未知，设备只按帧头 flags 判）

    // 段内帧号 → 里程（厘米精度，返回米×100）
    int64_t idxToMeterCm(int64_t idx) const { return (int64_t)segM0m * 100 + idx * (int64_t)segStepCm; }
    // 里程（厘米）→ 段内帧号（可负，调用方 clamp）
    int64_t meterCmToIdx(int64_t meterCm) const {
        if (segStepCm == 0) return 0;
        return (meterCm - (int64_t)segM0m * 100) / (int64_t)segStepCm;
    }
};

/// 取一帧（阻塞至多 timeoutMs）。false = 超时无帧。
/// 成功后 *data/*size 指向池内部缓冲，**只保证有效到下一次 pop**。
/// isKey（可空）：帧头 flags bit0（keyframe 自包含，可跳转入口）。
/// localIdx（可空）：段内帧号（收流顺序计数，清池归零；伺服按它与目标帧号对齐）。
bool pop(uint32_t timeoutMs, const uint8_t** data, uint32_t* size, uint32_t* seq,
         bool* isKey = nullptr, uint32_t* localIdx = nullptr);

/// 请求清池（段重置）：生产者在下一帧边界丢弃半满桶并清空队列、generation++。
/// 消费者用 generation() 感知清池生效（在此之前 pop 到的都是旧段帧，调用方自行丢弃）。
void     poolClearRequest();
uint32_t poolGeneration();   // 当前清池代号（消费侧对比用）

State      state();          // 当前状态（连接/断开由收流任务维护）
bool       session(Session* out);   // 会话头（App 连上后就有）
const char* ipText();        // 本机 IP 字符串（AP=固定值；STA=DHCP 到手后异步刷新）
const char* netSsid();       // 当前网络名（STA=热点名，AP=GTAM-CAST）
bool        staMode();       // true = STA 连手机热点模式（false = SoftAP）
int        depth();          // 池占用（桶数；0=空，消费停滞诊断用）
uint32_t   framesRecv();     // 已收帧计数（诊断）
uint32_t   framesBad();      // 协议错/尺寸超限帧数（诊断）

/// 启动：定位 castpool flash 分区（16MB 桶池）+ 分配收流/弹出暂存 + 起 WiFi/STA +
/// TCP 收流任务 + 诊断。必须在 bsp_display_start 之后、播放任务出图之前调用。
/// 返回 0 = 成功。
int start();

} // namespace gtwc
