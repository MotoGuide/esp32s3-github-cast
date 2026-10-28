// gtam_cast.h — 投屏模拟会话（GTAM P1）：PolylineEngine → NavCoreBridge → renderFrame 全链
//
// 【定位】未来"App 渲染 → SCF1 → WiFi → 设备播放"链路的**本地彩排**：
//   引擎、桥、渲染全在 C++ 会话里跑，Kotlin 只喂 1Hz 模拟 GPS + 拉 RGBA 帧。
//   参考实现 = gtamap/platforms/wasm/gtam_wasm.cpp 的 w_navcore_* 系列（逐位对齐其会话结构）。
//
// 【线程契约】PolylineEngine 假定单线程（updatePosition/tick/loadRoute 同线程，
//   见 polyline_engine.h:66）——调用方（Kotlin CastSimPanel）必须把会话全部调用
//   收在单线程执行器上串行。
//
// 【隔离】本文件属于 :gtam-render（debug-only 模块），release 构建不存在。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mapcore/gtam3.h"          // MapV3 / parse3
#include "mapcore/raster.h"         // Canvas（RGB565）
#include "mapcore/render.h"         // renderFrame
#include "api/input.h"              // NavInput / UiConfig / DeviceProfile
#include "api/frame.h"              // renderFrame / EngineInput / mppAtZoom
#include "api/zoom_smooth.h"        // TurnZoom（临近拐弯自动放大）
#include "icon/sprite.h"            // spriteFromBinary（GSPR → Sprite）
#include "icon/chebiao.h"           // 内嵌车标 GSPR（快照 core/icon 自带，embed_assets.py 产物）
#include "cast/cast.h"              // SCF1 编码（castEncode / castSwap16；设备端解码同一份）
#include "navcore/polyline_engine.h"      // 引擎真源（include 直指 :nav-core-cpp，不拷贝）
#include "bridge/nav_core_bridge.h"       // 5Hz tick → 30fps 帧桥

namespace gtamcast {

// 会话：引擎 + 桥 + 地图 + 画布。成员顺序即析构顺序（ncb 最后析构，兜住引擎回调引用）。
struct Session {
    std::vector<uint8_t> mapBytes;   // ★ 地图字节本体——MapV3.src 指向它，会话必须持有
    motonav::navcore::NullEngineCallback ncb;               // 空回调（偏航/到达事件渲染侧不用）
    std::unique_ptr<motonav::navcore::PolylineEngine> eng;  // 引擎（真源 navcore）
    gtam::bridge::NavCoreBridge nbr;                        // 引擎 5Hz → 渲染 30fps 桥
    gtam::MapV3 map;
    gtam::Canvas cv;                 // RGB565 画布（resize 后 core 自持缓冲）
    gtam::Canvas src;                // 旋转 scratch 画布（rotateOn 必需，frame.h:355）
    gtam::Sprite carSpr;             // 车标（open 时从内嵌 GSPR kChebiao 解码）
    bool hasCar = false;
    gtam::TurnZoom tz;               // 临近拐弯自动放大（engage 100m / targetZoom 17，core 默认）
    std::vector<uint8_t> rgba;       // RGBA 出帧缓冲（每帧覆写；本地预览用）
    int64_t lastTickMs = 0;          // 上次引擎 tick 墙钟（200ms 节拍泵用）
    int64_t lastFrameMs = 0;         // 上一帧墙钟（TurnZoom 平滑 dt 用）
    int64_t lastNavMs = 0;           // 上一帧墙钟（航向平滑 dt 用；与 TurnZoom 同值可合并，留独立好拆）
    bool    renderOk = false;        // 最近一次 render 是否成功（encode 的前置）

    // ---- SCF1 编码态（WiFi 推流用；本地预览不碰）----
    std::vector<uint16_t> beCur;     // 大端画布（线上口径：设备直写零换序，见 cast.h 字节序注释）
    std::vector<uint16_t> bePrev;    // 上一帧大端画布（delta 基准）
    std::vector<uint16_t> palBE;     // 大端调色板（编码反查用，与设备解码同份才咬合）
    bool    hasPrev = false;         // bePrev 有效（首帧必 keyframe）
    uint32_t seq = 0;                // 帧序号（写帧头 kCastOffSeq，设备断流诊断）
    uint32_t frameCount = 0;         // 强制 keyframe 周期计数（每 150 帧 ≈5s）
    size_t   lastKeySize = 0;        // 最近 keyframe 字节数（delta 收益判据基线）
    double  zoom = 16.5;             // 默认缩放档（Web-Mercator zoom）
    bool    started = false;         // 引擎 start() 已调（首个 GPS 到达时触发）

    // ---- 预渲染池模式开关（2026-10-28 v2 池+伺服；实时推流路径不受影响）----
    bool     tzEnabled = true;       // TurnZoom 开关。预渲染池模式要关：逐帧 mpp
                                     // 指数逼近 → 全画面重采样 → delta 全死（帧大小
                                     // 归因结论）；关掉后直行段纯竖直位移帧 3-30KB
    bool     directHeading = false;  // 直出航向：跳过平滑器，用 App 喂的航向（已量化
                                     // 5° 网格）。平滑器会把 5° 阶跃摊成多帧中间角度，
                                     // 每个中间角都是旋转帧；直出则只在跨格那 1 帧旋转
    double   directHeadingDeg = 0;   // 直出航向值（App 侧按 5° 量化后喂入）
    uint32_t keyInterval = 150;      // 强制 keyframe 周期（帧）。池+伺服模式 = 32
                                     //（kfInterval，跳转入口；150 是实时流老值）

    // ---- 传输层压缩（2026-10-28 二轮意见③；v2 会话头 flags bit0 对应设备开关）----
    // 开启后 encode() 产物从裸 SCF1 变为信封 [u32 clen][u32 rawLen][zlib deflate(SCF1)]。
    // 无损（deflate），SCF1 逐位一致 ⇒ 渲染零损失红线不破；桶里存的是解压后的 SCF1，
    // 设备解码/伺服路径零改动。实时 v1 推流不开（老路径保持裸帧）。
    bool     wireCompress = false;
    std::vector<uint8_t> zbuf;       // deflate 复用缓冲（encode 内部）
};

/// 开会话：解析地图 + 引擎 loadRoute + 桥 setRoute + 解码内嵌车标。
/// latlngPairs = [lat0,lng0, lat1,lng1, ...]（GCJ-02，与地图同系零转换）。
/// 失败返回 nullptr（地图解析失败 / 路线 < 2 点）。
Session* open(const uint8_t* mapData, size_t mapLen,
              const double* latlngPairs, int ptCount, double zoom);

/// 喂 GPS（1Hz，与真机同构）：只缓存快照不触发计算（计算在 tick）。
/// 首次调用触发引擎 start(nowMs)。nowMs = 墙钟（SystemClock.elapsedRealtime 口径，
/// 与后续 frame() 的 nowMs 同一时钟域——与 wasm 会话同规矩）。
void gps(Session* s, double lat, double lng, double speedKmh,
         double bearingDeg, int64_t nowMs);

/// 出一帧：内部先按 200ms 节拍泵引擎 tick + 桥 onTick，再桥插值 → TurnZoom 调视野
/// → renderFrame（路线→车标→HUD 三层，车头向上整幅旋转）→ RGB565 转 RGBA8888。
/// 成功返回字节数并置 *out（缓冲归会话所有，下次调用前有效）；失败/首帧前返回 0。
int frame(Session* s, int64_t nowMs, int w, int h, uint8_t** out);

/// 只渲不转（WiFi 推流用：设备要的是 RGB565→SCF1，RGBA 那道 868KB 转换纯浪费）。
/// true = 渲染成功，可以接着调 encode()。
bool render_raw(Session* s, int64_t nowMs, int w, int h);

/// SCF1 编码最近渲染的帧（须先 render_raw 成功）。策略：
///   首帧 / 每 150 帧 / delta 不划算 → keyframe（全覆盖）；
///   否则竖直 shift 搜索（±48px 采样网格——车头向上时位移恒竖直，v2 实测"水平 0 帧"）
///   → delta(shift)；分数不够（转弯旋转）→ 退 delta(0,0)；
///   编出大小超过 keyframe×3/4 → 改 keyframe（delta 收益判据）。
/// 返回编码字节数（out 带走）；失败 0。seq 会话内自增（写帧头，设备断流诊断）。
size_t encode(Session* s, std::vector<uint8_t>& out);

/// 传输层压缩开关（2026-10-28 二轮意见③）。true = encode() 产物变 deflate 信封；
/// 必须与会话头 flags bit0 同步置位（设备按会话头决定解不清帧）。池模式开。
void setCompress(Session* s, bool enabled);

/// 关会话（引擎/桥/地图随会话析构）。
void close(Session* s);

// ---- 预渲染池模式（v2 池+BLE 对拍；只影响渲染/编码策略，不动引擎状态机）----

/// TurnZoom 开关。池模式关（每帧 mpp 变化毁 delta）；实时推流保持开。
void setTurnZoom(Session* s, bool enabled);

/// 强制 keyframe 周期（帧；≥1）。池模式喂 kfInterval=32（伺服跳转入口）。
void setKeyInterval(Session* s, uint32_t frames);

/// 直出航向：跳过航向平滑器，后续渲染用 deg（App 侧已按 5° 网格量化）。
/// false = 恢复平滑器（实时路径）。off 后无需恢复喂值。
void setDirectHeading(Session* s, bool enabled, double deg);

/// 每帧更新直出航向值（仅 directHeading 开启时有意义）。
void updateDirectHeading(Session* s, double deg);

/// 路线总长（米；引擎预处理口径，haversine 真实米）。会话无效返回 0。
double routeTotalMeters(Session* s);

/// 沿路线取点：meter 米处的 [lat, lng, bearingDeg]（段内线性插值；bearing = 所在
/// 段方向角，正北 0 顺时针）。meter<0 当 0，超出总长返回 false（池播完的判据）。
bool routePosAt(Session* s, double meterM, double* lat, double* lng, double* bearingDeg);

} // namespace gtamcast
