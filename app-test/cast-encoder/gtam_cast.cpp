// gtam_cast.cpp — 投屏模拟会话实现（薄；会话结构与节拍对齐 wasm gtam_wasm.cpp 的
// w_navcore_load / w_navcore_tick / w_navcore_frame 三件套）。
//
// 与 wasm 的差异（有意为之）：
//   · GPS 不用 C++ SimNavController，而由 Kotlin SimGpsProvider（1Hz）喂 ——
//     与真机 GPS 节奏同构，顺带复用测试模式现成的模拟器
//   · tick 不单独暴露：frame() 内按 200ms 节拍泵（引擎 tick 吃墙钟，节奏自洽）
//   · 圆裁不做 alpha 掩码（wasm 的 flushRGBA/maskCircleRGBA），交给 Compose
//     CircleShape 裁 —— 显示层的事显示层办
#include "gtam_cast.h"

#include <cmath>
#include <cstring>
#include <android/log.h>   // 诊断日志（调色板命中率分析）
#include <zlib.h>          // 传输层 deflate（NDK libz，见 CMakeLists target_link_libraries z）

using namespace gtam;

namespace gtamcast {

/// SCF1 调色板 565 值 → 渲染器 Color（8bit 反推保证截断还原：r8=r5<<3|r5>>2 ⇒ r8>>3==r5）
static Color colFrom565(uint16_t v) {
    const int r5 = (v >> 11) & 0x1F, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
    return Color((r5 << 3) | (r5 >> 2), (g6 << 2) | (g6 >> 4), (b5 << 3) | (b5 >> 2));
}

Session* open(const uint8_t* mapData, size_t mapLen,
              const double* latlngPairs, int ptCount, double zoom) {
    if (!mapData || mapLen < 32 || !latlngPairs || ptCount < 2) return nullptr;

    auto* s = new Session();
    s->zoom = zoom;
    // 地图字节必须由会话持有（MapV3.src 指向调用方缓冲，见 gtam3.h:35）
    s->mapBytes.assign(mapData, mapData + mapLen);
    if (!parse3(s->mapBytes.data(), s->mapBytes.size(), s->map)) {
        delete s;
        return nullptr;
    }

    // 大端调色板（编码反查用；与设备解码端 s_pal 同源同份，palette run 颜色才不翻）
    s->palBE.resize(gtam::kCastPaletteN);
    for (int k = 0; k < gtam::kCastPaletteN; ++k)
        s->palBE[k] = gtam::castSwap16(gtam::kCastPalette[k]);

    // 车标：内嵌 GSPR（kChebiao，159×128 重采样版）→ Sprite。解码失败只丢车标不失败
    std::string sprErr;
    s->hasCar = spriteFromBinary(gtam::embedded::kChebiao,
                                 sizeof(gtam::embedded::kChebiao), s->carSpr, &sprErr);

    std::vector<motonav::navcore::LatLng> route;
    route.reserve((size_t)ptCount);
    for (int i = 0; i < ptCount; ++i)
        route.emplace_back(latlngPairs[2 * i], latlngPairs[2 * i + 1]);   // LatLng(lat, lng)

    s->eng = std::make_unique<motonav::navcore::PolylineEngine>(s->ncb);
    s->eng->loadRoute(route);
    // 桥直接吃引擎的 PreprocessedRoute：polyline / cumulativeDistM / totalDistanceM
    // 与引擎距离口径逐位同源（wasm 同款做法）
    if (const motonav::navcore::PreprocessedRoute* r = s->eng->getPreprocessed())
        s->nbr.setRoute(*r);
    s->nbr.reset();
    s->tz.reset();     // 换路线：拐弯放大从头起算（wasm 同款）
    return s;
}

void gps(Session* s, double lat, double lng, double speedKmh,
         double bearingDeg, int64_t nowMs) {
    if (!s || !s->eng) return;
    if (!s->started) {
        s->eng->start(nowMs);
        s->started = true;
    }
    // updatePosition(lat, lng, speedMs, bearingDeg, gpsTimeMs, accuracyM, nowMs)
    // —— 只缓存快照，tick 时才算（polyline_engine.h:171）。模拟精度固定 3m（真机良好档）
    s->eng->updatePosition(lat, lng, (float)(speedKmh / 3.6), (float)bearingDeg,
                           nowMs, 3.0, nowMs);
}

// 渲染核心（frame / render_raw 共用）：tick 泵 → 引擎直出 NavInput → TurnZoom → renderFrame
static bool render_core(Session* s, int64_t nowMs, int w, int h) {
    if (!s || !s->eng || !s->started || w <= 0 || h <= 0) return false;
    if (s->cv.w != w || s->cv.h != h || !s->cv.data) s->cv.resize(w, h);

    // ---- 引擎 tick 每帧一跳（~30fps）：模拟 GPS 也是 30fps（SimGpsProvider 33ms 档），
    //      每个 tick 都吃最新定位 ⇒ 引擎 renderLat/Lng 就是唯一位置源（单源实验，
    //      用户 2026-10-27 拍板：桥的里程插值从本会话路径删除；推屏/真机路径不受影响）----
    s->eng->tick(nowMs);

    // ---- 引擎直出 NavInput（不走 bridge.sample/fill —— 插值就在那里面）----
    // 桥在本会话只剩两个职责：① 持有路线几何（route()，供画路线）② 航向平滑器
    const motonav::navcore::EngineOutput o = s->eng->getLatestOutput();
    if (!o.hasRenderPoint) return false;      // 首帧定位前不出图（别把地图甩到 (0,0)）

    // ---- TurnZoom 视野（提前算：像素吸附需要 mpp）----
    //     池模式关（tzEnabled=false）：直出默认视野，消除逐帧 mpp 动画毁 delta
    const double defMpp = mppAtZoom(o.renderLat, s->zoom);
    const double tgtMpp = mppAtZoom(o.renderLat, s->tz.config().targetZoom);
    double dtFrame = (s->lastFrameMs > 0 && nowMs > s->lastFrameMs)
                   ? (double)(nowMs - s->lastFrameMs) / 1000.0 : 0.0;
    if (dtFrame > 0.5) dtFrame = 0.0;         // 切后台 / 长卡顿：不补一大步（wasm 同款）
    s->lastFrameMs = nowMs;
    const double mpp = s->tzEnabled
        ? s->tz.update((double)o.nextDistM, defMpp, tgtMpp, dtFrame)
        : defMpp;

    // ---- ★ 航向：平滑 + 量化到整度 ----
    // 真实航向每帧微变（平滑器连续输出）→ 车头向上整幅旋转角逐帧不同 → 两帧间
    // 不存在纯竖直位移 → shift-delta 必败 → delta 退化为 keyframe 尺寸（实测
    // 105KB）。量化 1°：相邻帧旋转角恒定，纯平移恢复，delta 回到 10-30KB 量级；
    // 1° 在 466px 上边缘差 ~4px，肉眼无感。
    // ★ 直出模式（预渲染池）：跳过平滑器直接用 App 喂的航向（App 已按 5° 网格
    //   量化）——平滑器会把 5° 阶跃摊成多帧中间角，每个中间角都是旋转 keyframe；
    //   直出则旋转只发生在跨格那一帧。
    double heading = s->directHeading
        ? std::round(s->directHeadingDeg)
        : std::round(s->nbr.heading.update(o.heading, dtFrame));

    // ---- ★ 像素吸附：车点按视野米/像素取整到整地图像素 ----
    // 亚像素位移会让道路边缘 AA 逐帧微变，逐位比较全 miss。吸附后相邻帧地图
    // 内容要么逐位相同、要么整像素位移（shift 可编码）；车标锚点固定在屏幕
    // 中心，地图在脚下整像素步进，1px ≈ 2.4m 视觉无感。
    double lat = o.renderLat, lng = o.renderLng;
    {
        const double dLatPx = mpp / 111320.0;
        const double dLngPx = mpp / (111320.0 * std::cos(lat * M_PI / 180.0));
        lat = std::round(lat / dLatPx) * dLatPx;
        lng = std::round(lng / dLngPx) * dLngPx;
    }

    NavInput nav;
    const std::vector<gtam::GeoPoint>& routePts = s->nbr.route();
    nav.setRoute(routePts.empty() ? nullptr : routePts.data(), (int)routePts.size(), -1);
    nav.routeClosed = false;
    nav.fixValid    = true;
    nav.lon         = lng;
    nav.lat         = lat;
    nav.speedMps    = (double)o.speedKmhFloat / 3.6;
    nav.turnKind    = o.turnDirection;
    nav.turnDistM   = (double)o.nextDistM;
    nav.remainDistM = (double)o.remainingDistM;
    nav.progressPct = o.progressPercent;
    nav.headingDeg = heading;

    UiConfig ui;
    // ★ 调色板钉死成 SCF1 的 8 色（kCastPalette）——设备解码 palette run 才能命中，
    //   帧尺寸才回得到 ~34KB 量级。走 mapStyle 内置风格（20 色）会让 castPalIndex
    //   全 miss → 全变 type2 像素序列（首推流实测 150~240KB/帧的原因）。
    //   Color 由 565 反推 8bit（r8=r5<<3|r5>>2），经渲染器截断后逐位还原回 565 表值。
    ui.useCustomPalette = true;
    ui.bg        = colFrom565(gtam::kCastPalette[0]);
    ui.road1     = colFrom565(gtam::kCastPalette[1]);
    ui.road2     = colFrom565(gtam::kCastPalette[1]);
    ui.road3     = colFrom565(gtam::kCastPalette[1]);
    ui.building  = colFrom565(gtam::kCastPalette[2]);
    ui.water     = colFrom565(gtam::kCastPalette[3]);
    ui.river     = colFrom565(gtam::kCastPalette[3]);
    ui.green     = colFrom565(gtam::kCastPalette[4]);
    ui.casing    = colFrom565(gtam::kCastPalette[5]);
    ui.gap       = colFrom565(gtam::kCastPalette[6]);
    // 路线色默认已是琥珀 #F2AC16 = kCastPalette[7]，不用动
    // ★ 路线描边/已走段钉进 8 色表：默认 #0A0A0A 与 #已走段色不在 kCastPalette
    //   里，路线横贯全屏，miss 一次就是几千 type2 像素。
    ui.routeCasing    = colFrom565(gtam::kCastPalette[5]);   // 描边 #0E0E0E（表内 casing）
    ui.routeDoneColor = colFrom565(gtam::kCastPalette[1]);   // 已走段 = 道路灰（表内 road）
    // ★ vignette 必须关：暗角把边缘像素混成成千上万种渐变色，8 色调色板全 miss，
    //   SCF1 帧从 ~100KB 爆到 239KB（实测）。投屏帧内容口径 = demo 验证过的
    //   「地图+路线+车标」；圆屏边缘淡出将来由设备侧自绘，不进帧。
    ui.vignetteVisible = false;
    ui.viewMpp = mpp;                         // TurnZoom 算好的视野（引擎策略，页面不自算）
    ui.rotateWithHeading = true;              // ★ 车头向上（旋转中心=车标锚点，引擎内定）
    if (s->hasCar) ui.iconSprite = &s->carSpr;   // ★ 车标图形（iconVisible/iconAutoSize 默认开）
    // ★ 矩形档案（不要 round466）：round 会把圆外四角填黑，#000000 不在 8 色表里
    //   → 4.7 万像素 ≈ 90KB type2（实测帧 176KB 的主犯）。满幅矩形帧 + 圆屏物理
    //   裁边 = demo 验证过的正确口径（demo 帧就是满幅 466×466 地图）。
    DeviceProfile dev = DeviceProfile::rect(w, h);

    EngineInput in;
    in.nav = nav; in.ui = ui; in.dev = dev;

    // ★ scratch 画布必须给：rotateOn 无 scratch 会退化为北向直出（frame.h:355）
    renderFrame(s->map, in, s->cv, nullptr, nullptr, &s->src);
    s->renderOk = true;
    return true;
}

int frame(Session* s, int64_t nowMs, int w, int h, uint8_t** out) {
    if (!render_core(s, nowMs, w, h)) return 0;

    // ---- RGB565 → RGBA8888（位展开式与 gtam_core.cpp 完全一致）----
    s->rgba.resize((size_t)w * h * 4);
    const int stride = s->cv.pitch / 2;                       // pitch 字节 → uint16 步长
    const uint16_t* src = reinterpret_cast<const uint16_t*>(s->cv.data);
    uint8_t* dst = s->rgba.data();
    for (int y = 0; y < h; ++y) {
        const uint16_t* row = src + (size_t)y * stride;
        for (int x = 0; x < w; ++x) {
            const uint16_t v = row[x];
            const uint8_t r5 = (v >> 11) & 0x1F, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
            dst[0] = (uint8_t)((r5 << 3) | (r5 >> 2));
            dst[1] = (uint8_t)((g6 << 2) | (g6 >> 4));
            dst[2] = (uint8_t)((b5 << 3) | (b5 >> 2));
            dst[3] = 0xFF;
            dst += 4;
        }
    }
    *out = s->rgba.data();
    return (int)s->rgba.size();
}

bool render_raw(Session* s, int64_t nowMs, int w, int h) {
    return render_core(s, nowMs, w, h);   // 推流路径：canvas 里的 RGB565 就是编码原料
}

// 竖直 shift 评分：采样网格上 cur[y] 与 prev[y-sy] 的命中率（0~1）
static double shiftScore(const std::vector<uint16_t>& cur, const std::vector<uint16_t>& prev,
                         int W, int H, int sy) {
    int total = 0, match = 0;
    for (int y = 0; y < H; y += 16) {
        const int py = y - sy;
        if (py < 0 || py >= H) continue;
        for (int x = 0; x < W; x += 8) {
            ++total;
            if (cur[(size_t)y * W + x] == prev[(size_t)py * W + x]) ++match;
        }
    }
    return total ? (double)match / (double)total : 0.0;
}

size_t encode(Session* s, std::vector<uint8_t>& out) {
    out.clear();
    if (!s || !s->renderOk) return 0;
    const int W = s->cv.w, H = s->cv.h;
    const size_t n = (size_t)W * H;

    // ① 大端化（线上口径：设备直写通道零换序，见 cast.h 字节序注释；stride 兼容非连续行）
    s->beCur.resize(n);
    const uint16_t* src = reinterpret_cast<const uint16_t*>(s->cv.data);
    const int stride = s->cv.pitch / 2;
    for (int y = 0; y < H; ++y) {
        const uint16_t* row = src + (size_t)y * stride;
        for (int x = 0; x < W; ++x)
            s->beCur[(size_t)y * W + x] = castSwap16(row[x]);
    }

    // ② keyframe 时机：首帧 / 每 keyInterval 帧（实时流 150≈5s 兜底锚点；
    //    池+伺服模式 = kfInterval 32，伺服跳转入口）
    ++s->frameCount;
    const uint32_t kfEvery = s->keyInterval ? s->keyInterval : 150;
    const bool forceKey = !s->hasPrev || (s->frameCount % kfEvery == 0);

    // ---- 诊断（仅首帧跑一次）：8 色命中率 + 最大的 8 个表外颜色 ----
    // 回答"帧为什么大"：命中率低 = 有成片非调色板内容（直方图给出具体色值）。
    static uint32_t* diagCnt = nullptr;   // 65536 色 × 4B = 256KB，会话级一次（原生堆）
    if (s->frameCount == 1) {
        if (!diagCnt) diagCnt = (uint32_t*)std::calloc(65536, 4);
        if (diagCnt) {
            std::memset(diagCnt, 0, 65536 * 4);
            for (size_t i = 0; i < n; ++i) diagCnt[s->beCur[i]]++;
            size_t hit = 0;
            for (int k = 0; k < gtam::kCastPaletteN; ++k) hit += diagCnt[s->palBE[k]];
            struct Top { uint16_t c; uint32_t n; };
            Top top[8] = {};
            for (uint32_t c = 0; c < 65536; ++c) {
                bool isPal = false;
                for (int k = 0; k < gtam::kCastPaletteN; ++k)
                    if ((uint16_t)c == s->palBE[k]) { isPal = true; break; }
                if (isPal || diagCnt[c] == 0) continue;
                for (auto& t : top)
                    if (diagCnt[c] > t.n) {
                        // 插入排序挤入前 8
                        Top cur{ (uint16_t)c, diagCnt[c] };
                        for (auto& t2 : top)
                            if (cur.n > t2.n) { Top tmp = t2; t2 = cur; cur = tmp; }
                        break;
                    }
            }
            char tops[160] = "";
            int off = 0;
            for (auto& t : top)
                if (t.n) {
                    const uint16_t le = gtam::castSwap16(t.c);   // 还原主机序好读
                    off += std::snprintf(tops + off, sizeof(tops) - off,
                                         " %04X:%uK", (unsigned)le, (unsigned)(t.n / 1000));
                }
            __android_log_print(ANDROID_LOG_INFO, "gtamcast",
                                "首帧调色板命中 %u/%u = %.1f%% | 表外Top8(565,像素K):%s",
                                (unsigned)hit, (unsigned)n,
                                100.0 * (double)hit / (double)n, tops);
        }
    }

    // ③ 竖直 shift 搜索（车头向上位移恒竖直；97 候选 × 采样网格 ≈ 17 万比较，亚毫秒）
    int bestSy = 0;
    double bestScore = 0.0;
    if (s->hasPrev && !forceKey) {
        for (int sy = -48; sy <= 48; ++sy) {
            const double sc = shiftScore(s->beCur, s->bePrev, W, H, sy);
            if (sc > bestScore) { bestScore = sc; bestSy = sy; }
        }
    }

    // ④ 编码策略：delta(bestSy) → delta(0,0) → keyframe，按"大小 ≤ key×3/4"逐级退
    CastEncodeOpt opt;
    opt.seq = s->seq;
    std::vector<uint8_t> tmp;
    auto tryEnc = [&](bool delta, int sy) -> size_t {
        opt.delta = delta;
        opt.prev  = delta ? s->bePrev.data() : nullptr;
        opt.prevW = delta ? W : 0;
        opt.prevH = delta ? H : 0;
        opt.shiftX = 0;
        opt.shiftY = delta ? sy : 0;
        return castEncode(s->beCur.data(), W, H, opt, tmp, s->palBE.data());
    };

    bool ok = false;
    if (!forceKey && bestScore >= 0.95) {
        const size_t n1 = tryEnc(true, bestSy);
        if (n1 > 0 && (s->lastKeySize == 0 || n1 <= s->lastKeySize * 3 / 4)) {
            out.swap(tmp);
            ok = true;
        }
    }
    if (!ok && !forceKey && s->hasPrev) {
        // 转弯/旋转段：无单一平移，退行级 delta（0,0）；不划算再上 keyframe
        const size_t n2 = tryEnc(true, 0);
        if (n2 > 0 && (s->lastKeySize == 0 || n2 <= s->lastKeySize * 3 / 4)) {
            out.swap(tmp);
            ok = true;
        }
    }
    if (!ok) {
        const size_t nk = tryEnc(false, 0);   // keyframe：全覆盖
        if (nk == 0) return 0;
        out.swap(tmp);
        s->lastKeySize = nk;
    }

    // ⑤ 状态推进：bePrev ← 本帧（下一帧 delta 基准；swap 复用旧 prev 的内存）
    s->hasPrev = true;
    s->bePrev.swap(s->beCur);
    ++s->seq;

    // ⑥ 传输层压缩（2026-10-28 二轮意见③）：SCF1 → 信封
    //    [u32 clen][u32 rawLen][zlib deflate(SCF1)]，外层 [u32 len] 由 Kotlin 照旧写
    //    enc.size（= clen+8）。设备按 v2 会话头 flags bit0 决定解不清帧，无逐帧歧义。
    //    ★ 无损：uncompress(compress2(x)) == x（zlib 保证）⇒ SCF1 逐位一致，渲染
    //    零损失红线不破。level：地图大色块 + varint 冗余高。
    //    2026-10-28 晚实测（level 6）：压缩率 37-44% 达标，但 deflate 成为手机编码
    //    瓶颈的一部分（推流 2fps→1.5fps）。level 6→1：速度约 3-5×，预期压缩率
    //    降到 ~45-55%（仍比不压省一半），配合面板阶段计时回填定档。
    if (s->wireCompress) {
        const size_t rawLen = out.size();
        s->zbuf.resize(::compressBound((uLong)rawLen));
        uLongf clen = s->zbuf.size();
        if (::compress2(s->zbuf.data(), &clen, out.data(), (uLong)rawLen, 1) != Z_OK
                || clen == 0 || clen > 0xFFFFFF) {          // clen 留 24bit 余量防溢出
            return 0;                                       // 压缩失败 = 本帧放弃（保守）
        }
        std::vector<uint8_t> env;
        env.resize(8 + (size_t)clen);
        env[0] = (uint8_t)(clen & 0xFF);
        env[1] = (uint8_t)((clen >> 8) & 0xFF);
        env[2] = (uint8_t)((clen >> 16) & 0xFF);
        env[3] = (uint8_t)((clen >> 24) & 0xFF);
        env[4] = (uint8_t)(rawLen & 0xFF);
        env[5] = (uint8_t)((rawLen >> 8) & 0xFF);
        env[6] = (uint8_t)((rawLen >> 16) & 0xFF);
        env[7] = (uint8_t)((rawLen >> 24) & 0xFF);
        std::memcpy(env.data() + 8, s->zbuf.data(), (size_t)clen);
        out.swap(env);
    }
    return out.size();
}

void close(Session* s) {
    delete s;   // eng/桥/地图随会话析构（ncb 声明在 eng 之前，析构顺序安全）
}

// ════════════ 预渲染池模式（v2 池+BLE 对拍） ════════════

void setTurnZoom(Session* s, bool enabled) {
    if (s) s->tzEnabled = enabled;
}

void setKeyInterval(Session* s, uint32_t frames) {
    if (s && frames >= 1) s->keyInterval = frames;
}

void setDirectHeading(Session* s, bool enabled, double deg) {
    if (!s) return;
    s->directHeading = enabled;
    s->directHeadingDeg = deg;
}

void updateDirectHeading(Session* s, double deg) {
    if (s) s->directHeadingDeg = deg;
}

void setCompress(Session* s, bool enabled) {
    if (s) s->wireCompress = enabled;
}

double routeTotalMeters(Session* s) {
    if (!s || !s->eng) return 0.0;
    const auto* r = s->eng->getPreprocessed();
    return r ? r->totalDistanceM : 0.0;
}

bool routePosAt(Session* s, double meterM, double* lat, double* lng, double* bearingDeg) {
    if (!s || !s->eng || !lat || !lng || !bearingDeg) return false;
    const auto* r = s->eng->getPreprocessed();
    if (!r || r->cumulativeDistM.size() < 2 || r->polyline.size() < 2) return false;

    const size_t n = r->cumulativeDistM.size();
    if (meterM < 0) meterM = 0;
    if (meterM > r->totalDistanceM) return false;      // 超出 = 池播完判据

    // 找 meter 所在段（线性扫；路线几百点，逐帧一次亚毫秒）
    size_t i = 1;
    while (i < n - 1 && r->cumulativeDistM[i] < meterM) ++i;
    const double d0 = r->cumulativeDistM[i - 1];
    const double d1 = r->cumulativeDistM[i];
    const double t = (d1 > d0) ? (meterM - d0) / (d1 - d0) : 0.0;
    const auto& p0 = r->polyline[i - 1];
    const auto& p1 = r->polyline[i];
    *lat = p0.lat + (p1.lat - p0.lat) * t;
    *lng = p0.lng + (p1.lng - p0.lng) * t;

    // 段方向角（平面近似：短段内误差可忽略；正北 0 顺时针）
    const double dLatM = (p1.lat - p0.lat) * 111320.0;
    const double dLngM = (p1.lng - p0.lng) * 111320.0
                       * std::cos(p0.lat * M_PI / 180.0);
    double brg = std::atan2(dLngM, dLatM) * 180.0 / M_PI;   // atan2(东, 北)
    if (brg < 0) brg += 360.0;
    *bearingDeg = brg;
    return true;
}

} // namespace gtamcast
