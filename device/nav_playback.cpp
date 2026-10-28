// nav_playback.cpp — 投屏演示播放端 v2（验证「ESP32 = 纯播放器」这条链路）
//
// 上游：tools/_tmp/gen_nav_frames.cpp 用 core 渲染出 100 帧导航画面（含移动、
//       旋转、黄色路线、车头箭头、转向灯），按 format_cast_v1.md（SCF1 v2）
//       编码成 build/nav_frames.bin，内嵌进 app 镜像 .rodata（flash 原地读，
//       不占 PSRAM）。帧容器 NVF1：
//         [0..4)   magic "NVF1"
//         [4..24)  nFrames / w / h / fps / rsv（u32 LE）
//         [24..)   帧 offset 表：u32 offset + u32 size，offset 相对数据区
//         数据区   每帧一段独立的 SCF1 帧（各自带 27B 帧头）
//
// ★ v2（2026-10-22）：直写 + 滑窗。旧版 62ms/帧的账（串口实测）：
//     等锁 38.5 + decode 24.0 ‖ LVGL refr 41.3 = render 41.1 + flush 2.1
//   两笔纯冗余搬运是全部瓶颈：
//     ① render 41ms = LVGL 把 434KB canvas 逐像素混合进 draw buffer（868KB 流量）
//     ② decode 24ms = shift 帧内部整屏 memmove（868KB 流量）
//   v2 两条都砍掉：
//     ① 直写：开 dummy draw（esp_lv_adapter_set_dummy_draw），LVGL 渲染整个旁路，
//        解码结果直接由 esp_lv_adapter_dummy_draw_blit → esp_lcd_panel_draw_bitmap
//        推屏。没有 canvas、没有 draw buffer、没有等锁。
//     ② 滑窗：缓冲做成环（ring 高 = h + RING_MARGIN），竖直位移不搬一个像素，
//        只挪窗口基址 oy（内容下移 sy → 基址上移 oy -= sy），新露出的边行由
//        run 流覆盖（编码器「越界即不同」规则保证全覆盖，见 test_slide.cpp）。
//        → 解码器必须跳过那次 memmove：cast.h 的 skipShift 参数。
//   实测位移构成：100 帧里 68 帧是纯竖直位移（可滑窗）——车头向上导航时，
//   无论朝哪开，位移在屏幕上都是「前方来的内容从上往下走」，水平位移 0 帧。
//   （31 帧旋转段没有单一平移，只能老实解码；1 帧 keyframe 全写。）
//
// ★ 时序纪律（撕裂防线）：每帧严格「先写完、再推屏」。解码的写入全部落在
//   blit 入队之前，而上一帧的 SPI DMA 早已在 22ms 内跑完（帧周期 33ms），
//   所以写入与 DMA 读区永不重叠 → 环缓冲 + 异步 DMA 不需要双缓冲/翻页。
//
// HUD：dummy draw 下 LVGL 不渲染，屏上信息自绘——左下角小区域，
//   独立 blit 叠加。★ 不能画进 ring：ring 会滑动，画进去的像素会跟着窗口
//   漂成幽灵。
//
// ★ GTAM_WIFI_CAST=1（2026-10-27）：WiFi 投屏收流模式。帧源从「flash 内嵌
//   NVF1 容器」换成「wifi_cast.cpp 的 TCP 收流队列」（手机 App 推帧），播放
//   任务改为按帧到达节奏出图（App 30fps 推，本端跟着播；队列满 → TCP 背压
//   传导回 App）。开机流程：SoftAP 等连（屏绘 SSID/密码/IP）→ TCP 连上
//   （LINK 页）→ 帧流到 → 全屏逐帧播放（HUD 秒级 fps）。旧 NVF1 演示路径
//   原样保留（GTAM_WIFI_CAST=0 回退），两条路径共用滑窗/解码/推屏的时序纪律。
//
// watchdog：任务 prio 1 钉 core1。解码无双精度，节奏由帧到达/节拍保证。

#ifndef GTAM_NAV_PLAYBACK
#define GTAM_NAV_PLAYBACK 0
#endif
// WiFi 投屏收流模式开关（CMake 传入；=0 时本文件即旧的内嵌帧播放演示）
#ifndef GTAM_WIFI_CAST
#define GTAM_WIFI_CAST 0
#endif

#if !GTAM_NAV_PLAYBACK
// 浏览模式（嵌 tiles_pack.gts）：本文件不参与链接，只留空壳
extern "C" int nav_playback_start(void) { return -1; }
#else

#include <cstring>
#include <cstdint>
#include <cstdio>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "esp_lv_adapter.h"   // 直写通道：dummy draw（头自带 extern "C"）

#include "cast/cast.h"   // gtam::castDecode（core 同一份，PC/ESP32 共用）
#if GTAM_WIFI_CAST
#include "wifi_cast.h"   // TCP 收流队列（SoftAP/协议/背压都在那边）
#include "ble_conductor.h"   // BLE 对拍器：0xD0 目标里程快照 + 0xE3 播放头回显
#endif

static const char* TAG = "navplay";

// 环上窗口可滑动的余量（行）。必须 ≥ 单轮内竖直位移的累积绝对值；
// 撞边才 memmove 归位一次（罕见）。672KB PSRAM 换 68% 帧免搬运。
static constexpr int RING_MARGIN = 256;
// HUD 区域（左下角）。★ 坐标必须偶数：CO5300 推屏按 2 像素对齐（BSP rounder
// 同规格），奇数边长会让最后一列/行推不上去。
static constexpr int HUD_W = 24, HUD_H = 12;
static constexpr int HUD_X = 0, HUD_Y = 452;   // 452 + 12 = 464 < 466

#if !GTAM_WIFI_CAST
// 内嵌帧数据符号（main/CMakeLists.txt target_add_binary_data nav_frames.bin）
// ★ 仅内嵌模式链接（投屏模式不嵌 bin，符号不存在，引用必须一起关掉）
extern const uint8_t _binary_nav_frames_bin_start[];
extern const uint8_t _binary_nav_frames_bin_end[];
#endif

static uint32_t     s_nFrames = 0, s_w = 0, s_h = 0, s_fps = 0;
static const uint8_t* s_tab  = nullptr;   // 帧 offset 表（仅内嵌模式）
static const uint8_t* s_data = nullptr;   // 帧数据区（仅内嵌模式）
static size_t       s_dataLen = 0;
static uint16_t*    s_ring = nullptr;     // 环缓冲 = 解码目标 = 推屏源（PSRAM）
static int          s_ringH = 0;          // = s_h + RING_MARGIN
static int          s_oy = 0;             // 当前窗口在环内的行偏移
static uint16_t*    s_hudBuf = nullptr;   // HUD 小画布（独立 blit，不进 ring）
// ★ 调色板（大端版）。帧包里的像素一律大端存（见 core/cast.h 顶部 castSwap16 注释），
//   因为本工程的直写通道绕过了 LVGL flush 里那次 lv_draw_sw_rgb565_swap
//   （lvgl_bridge_v9.c:2294，条件 interface==PANEL_IF_OTHER && cf==RGB565 都成立）。
//   解码端照抄字节 → 推屏即正确，热路径零开销；只在这里换序 8 个数。
static uint16_t     s_pal[gtam::kCastPaletteN];
static lv_display_t* s_disp = nullptr;
static TaskHandle_t s_task = nullptr;

static inline uint32_t rd32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// 全屏 blit 分带推送。★ 为什么不一把梭：434KB 整幅传输需要同量级 DMA 私有内部
// 缓冲（esp_lcd spi panel_io 的 setup_dma_priv_buffer），WiFi 栈常驻后内部 RAM
// 挤不出来 → ESP_ERR_NO_MEM（实测 22:58 固件开机第一笔画等待页就炸）。32 行/条
// = 30KB/条，priv 需求缩 14 倍；带高 32 保持 2 像素对齐（CO5300 rounder 纪律）。
static constexpr int BLIT_BAND_H = 32;
static esp_err_t blit_banded(const uint16_t* win) {
    esp_err_t last = ESP_OK;
    for (int y = 0; y < (int)s_h; y += BLIT_BAND_H) {
        const int h = (y + BLIT_BAND_H <= (int)s_h) ? BLIT_BAND_H : (int)s_h - y;
        const esp_err_t e = esp_lv_adapter_dummy_draw_blit(s_disp, 0, y, (int)s_w, y + h,
                                       win + (size_t)y * s_w, false);
        if (e != ESP_OK) last = e;   // 记最后一条失败（带间互不阻断，尽量多推）
    }
    return last;
}

// ---- 3×5 点阵字模（bit2=左 bit1=中 bit0=右；够画 HUD 两位数 + 单位）----
// 0123456789 FPS —— 只留用得上的 13 个，其余按空格处理
static const uint8_t kFont3x5[13][5] = {
    { 0x7, 0x5, 0x5, 0x5, 0x7 },  // 0
    { 0x2, 0x6, 0x2, 0x2, 0x7 },  // 1
    { 0x7, 0x1, 0x7, 0x4, 0x7 },  // 2
    { 0x7, 0x1, 0x7, 0x1, 0x7 },  // 3
    { 0x5, 0x5, 0x7, 0x1, 0x1 },  // 4
    { 0x7, 0x4, 0x7, 0x1, 0x7 },  // 5
    { 0x7, 0x4, 0x7, 0x5, 0x7 },  // 6
    { 0x7, 0x1, 0x1, 0x1, 0x1 },  // 7
    { 0x7, 0x5, 0x7, 0x5, 0x7 },  // 8
    { 0x7, 0x5, 0x7, 0x1, 0x7 },  // 9
    { 0x7, 0x4, 0x7, 0x4, 0x4 },  // F
    { 0x7, 0x5, 0x7, 0x4, 0x4 },  // P
    { 0x7, 0x4, 0x7, 0x1, 0x7 },  // S
};
static int fontIdx(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == 'F') return 10;
    if (c == 'P') return 11;
    if (c == 'S') return 12;
    return -1;
}

// HUD 只画 3 个字符："xxF"（实测 fps 取整 + F）。左上角 (2,3) 起笔，
// 每字符 4px 步进（3px 字 + 1px 间距）→ 占 12px，24 宽的画布放得下。
static void hud_render(int fps) {
    if (!s_hudBuf) return;
    for (int i = 0; i < HUD_W * HUD_H; i++) s_hudBuf[i] = 0x0000;   // 纯黑底
    char s[4];
    std::snprintf(s, sizeof(s), "%2dF", fps > 99 ? 99 : fps);
    const uint16_t col = s_pal[7];                    // 橙 #F2AC16（大端，与路线同色）
    int x = 2;
    for (int k = 0; k < 3; k++) {
        const int fi = fontIdx(s[k]);
        if (fi >= 0) {
            for (int r = 0; r < 5; r++)
                for (int b = 0; b < 3; b++)
                    if (kFont3x5[fi][r] & (1u << (2 - b)))
                        s_hudBuf[(3 + r) * HUD_W + x + b] = col;
        }
        x += 4;
    }
}

#if GTAM_WIFI_CAST
// ════════════════ WiFi 投屏模式（帧源 = TCP 收流队列） ════════════════

// 完整 3×5 字模（状态页用；播放 HUD 用上面的 kFont3x5，互不干扰）
struct FG { char c; uint8_t g[5]; };
static const FG kGlyphs[] = {
    {'0',{7,5,5,5,7}},{'1',{2,6,2,2,7}},{'2',{7,1,7,4,7}},{'3',{7,1,7,1,7}},
    {'4',{5,5,7,1,1}},{'5',{7,4,7,1,7}},{'6',{7,4,7,5,7}},{'7',{7,1,1,1,1}},
    {'8',{7,5,7,5,7}},{'9',{7,5,7,1,7}},
    {'A',{2,5,7,5,5}},{'B',{6,5,6,5,6}},{'C',{3,4,4,4,3}},{'D',{6,5,5,5,6}},
    {'E',{7,4,6,4,7}},{'F',{7,4,6,4,4}},{'G',{3,4,5,5,3}},{'H',{5,5,7,5,5}},
    {'I',{7,2,2,2,7}},{'J',{1,1,1,5,2}},{'K',{5,5,6,5,5}},{'L',{4,4,4,4,7}},
    {'M',{5,7,5,5,5}},{'N',{6,5,5,5,5}},{'O',{7,5,5,5,7}},{'P',{7,5,7,4,4}},
    {'Q',{7,5,5,7,1}},{'R',{6,5,6,5,5}},{'S',{7,4,7,1,7}},{'T',{7,2,2,2,2}},
    {'U',{5,5,5,5,7}},{'V',{5,5,5,5,2}},{'W',{5,5,5,7,5}},{'X',{5,5,2,5,5}},
    {'Y',{5,5,2,2,2}},{'Z',{7,1,2,4,7}},
    {'-',{0,0,0,0,7}},{'.',{0,0,0,0,2}},{':',{0,2,0,2,0}},{'+',{0,2,7,2,0}},
};
static const FG* glyph(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');   // 热点名等用户串按大写渲染
    for (const FG& f : kGlyphs) if (f.c == c) return &f;
    return nullptr;    // 未收录字符 = 空一格
}

// 缩放点阵字绘制（scale=1 即 3×5；未收录字符按 3×scale 空白步进）
static void draw_text5(uint16_t* buf, int W, int H, int x, int y,
                       const char* s, int scale, uint16_t col) {
    for (const char* q = s; *q; ++q) {
        const FG* g = glyph(*q);
        if (g) {
            for (int r = 0; r < 5; r++)
                for (int b = 0; b < 3; b++)
                    if (g->g[r] & (1u << (2 - b)))
                        for (int sy = 0; sy < scale; sy++)
                            for (int sx = 0; sx < scale; sx++) {
                                const int px = x + b * scale + sx;
                                const int py = y + r * scale + sy;
                                if (px >= 0 && px < W && py >= 0 && py < H)
                                    buf[(size_t)py * W + px] = col;
                            }
        }
        x += 4 * scale;    // 3px 字 + 1px 间距（×scale）
    }
}

// 状态页（画进滑窗环再整屏 blit——此时不在播帧，环窗口空闲可当画布）
// 颜色全用大端口径：白 0xFFFF / 黑 0x0000 对称不用换，橙用 s_pal[7]
static void draw_status_screen(gtwc::State st) {
    uint16_t* win = s_ring + (size_t)(RING_MARGIN / 2) * s_w;
    const int W = (int)s_w, H = (int)s_h;
    for (int i = 0; i < W * H; i++) win[i] = 0x0000;   // 黑底
    const uint16_t amber = s_pal[7];
    const uint16_t white = 0xFFFF;

    if (st == gtwc::State::Wait) {
        // 大标题居中：9 字符 ×4px×6 = 216px → x=(466-216)/2≈125
        draw_text5(win, W, H, (W - 9 * 4 * 6) / 2, 100, "GTAM-CAST", 6, amber);
        char line[48];
        std::snprintf(line, sizeof(line), "NET %s", gtwc::netSsid());
        draw_text5(win, W, H, 100, 230, line, 3, white);
        if (gtwc::staMode())
            std::snprintf(line, sizeof(line), "MODE STA :7123");   // 连热点模式：密码是用户自己的，不外显
        else
            std::snprintf(line, sizeof(line), "PASS %s", "87654321");
        draw_text5(win, W, H, 100, 270, line, 3, white);
        std::snprintf(line, sizeof(line), "IP %s", gtwc::ipText());   // STA 下 DHCP 到手后自动重画
        draw_text5(win, W, H, 100, 310, line, 3, white);
        draw_text5(win, W, H, (W - 10 * 4 * 3) / 2, 390, "WAIT PHONE", 3, amber);
    } else if (st == gtwc::State::Link) {
        draw_text5(win, W, H, (W - 7 * 4 * 6) / 2, 150, "LINK OK", 6, amber);
        draw_text5(win, W, H, (W - 10 * 4 * 3) / 2, 260, "WAIT FRAME", 3, white);
    }
    blit_banded(win);
}

// 尺寸错页（会话 w/h ≠ 屏幕：直写不能成立，不出图、背压传导回 App）
static void draw_size_err(const gtwc::Session& sess) {
    uint16_t* win = s_ring + (size_t)(RING_MARGIN / 2) * s_w;
    const int W = (int)s_w, H = (int)s_h;
    for (int i = 0; i < W * H; i++) win[i] = 0x0000;
    char line[48];
    std::snprintf(line, sizeof(line), "SIZE ERR %ux%u", sess.w, sess.h);
    // 颜色就地取（amber/white 是 draw_status_screen 的局部量，这里直接用同源值）
    draw_text5(win, W, H, (W - 15 * 4 * 3) / 2, 200, line, 3, s_pal[7]);
    draw_text5(win, W, H, (W - 16 * 4 * 3) / 2, 260, "CHECK APP DEVICE", 3, 0xFFFF);
    blit_banded(win);
}

// 投屏播放任务：等帧 → 滑窗/解码/推屏（时序纪律与内嵌路径同一套，勿改）
// 解码一帧进滑窗环（实时/伺服共用；isKey/slide 判定都在内）。
// 返回 decode 耗时 ms；解码失败返回 -1（环内容可能已脏，调用方自行处置）。
static int32_t decode_into_ring(const uint8_t* p, uint32_t sz) {
    const bool isKey = (p[gtam::kCastOffFlags] & 1) != 0;
    const int16_t shX = (int16_t)(p[gtam::kCastOffShiftX] |
                                  (p[gtam::kCastOffShiftX + 1] << 8));
    const int16_t shY = (int16_t)(p[gtam::kCastOffShiftY] |
                                  (p[gtam::kCastOffShiftY + 1] << 8));
    // 只有「纯竖直位移」能滑窗（与内嵌路径同规则）
    const bool canSlide = !isKey && shX == 0 && shY != 0;

    if (isKey) {
        s_oy = RING_MARGIN / 2;      // 免费归位：反正整帧全写，不用搬
    } else if (canSlide) {
        int oy2 = s_oy - shY;
        if (oy2 < 0 || oy2 + (int)s_h > s_ringH) {
            std::memmove(s_ring + (size_t)(RING_MARGIN / 2) * s_w,
                         s_ring + (size_t)s_oy * s_w,
                         (size_t)s_h * s_w * 2);
            s_oy = RING_MARGIN / 2;
            oy2  = s_oy - shY;
        }
        s_oy = oy2;
    }

    uint16_t* win = s_ring + (size_t)s_oy * s_w;
    const int64_t t0 = esp_timer_get_time();
    const bool ok = gtam::castDecode(p, sz, win, (int)s_w, (int)s_h,
                                     nullptr, nullptr, nullptr, nullptr,
                                     canSlide, s_pal);
    return ok ? (int32_t)((esp_timer_get_time() - t0) / 1000) : -1;
}

// blit 当前滑窗 + 补推 HUD（实时/伺服共用）
static void present_frame() {
    blit_banded(s_ring + (size_t)s_oy * s_w);
    if (s_hudBuf)
        esp_lv_adapter_dummy_draw_blit(s_disp, HUD_X, HUD_Y,
                                       HUD_X + HUD_W, HUD_Y + HUD_H,
                                       s_hudBuf, false);
}

#if GTAM_WIFI_CAST
// ══════════════ 池伺服（方案 §5/§5.1；会话 v2 mode=1） ══════════════
//
// 状态机唯一事实源 = docs/cast_pool_servo_plan.md §5.1 权威状态迁移表。
// ★ 改这里必须同步改表，改表必须同步改这里（先补表再补码）。
// 裁决规则落地：① SILENT 优先 ② 不倒放（回退一律保持+告警）③ paused 是 HOLD
// 的子状态、段重置将其清除。

enum { SV_PLAY = 0, SV_HOLD = 1, SV_STALL = 2, SV_SILENT = 3, SV_RESET = 4 };
static const char* svName(int s) {
    switch (s) {
        case SV_PLAY:   return "PLAY";
        case SV_HOLD:   return "HOLD";
        case SV_STALL:  return "STALL";
        case SV_SILENT: return "SILENT";
        case SV_RESET:  return "RESET";
    }
    return "?";
}

static constexpr int64_t KF_JUMP      = 4;      // 缺口 >4 帧走 keyframe 跳（三审⑤）
static constexpr int64_t BT_BACK      = 8;      // 目标回退 >8 帧 = 告警（App 应走段重置）
static constexpr int64_t TICK_US      = 66000;  // 15fps
static constexpr int64_t SILENT_US    = 6000000;   // BLE 静默 >6s 冻结（2026-10-28 实测
                                                   // 空口瞬时有 2-5s 停顿，2s 档误冻结）
static constexpr int64_t RESET_TIMEOUT_US = 5000000;   // RESET 5s 无 keyframe → STALL
static constexpr int64_t STALL_WARN_US    = 30000000;  // STALL 持续 >30s 周期性上报
static constexpr int32_t DELTA_EST_MS = 24;     // delta 解码实测均值（预算守卫）
static constexpr int32_t BLIT_RES_MS  = 8;      // blit 预留
static constexpr int32_t MARGIN_MS    = 3;      // tick 裕度

static void servo_loop(const gtwc::Session* sess0) {
    int      state    = SV_RESET;    // 起始 = 等首帧 keyframe（App 段首必发 keyframe）
    bool     paused   = false;       // HOLD 的子状态（裁决 3）
    bool     havePlay = false;       // 已有解码帧（画面非空）
    bool     skipping = false;       // KF 跳中：丢弃直到 keyframe（delta 链已断不能解）
    bool     clearPending = false;   // 已发清池请求、等生产者 gen++ 生效
    uint32_t playIdx  = 0;           // 最后解码帧的段内帧号
    uint32_t myGen    = gtwc::poolGeneration();
    uint32_t curSegId = sess0->segId;
    int64_t  resetT0 = esp_timer_get_time();
    int64_t  stallT0 = 0;
    int64_t  lastReport   = 0;
    int64_t  lastBackWarn = 0;
    int32_t  kfDecMs = 30;           // keyframe 解码实测（滚动更新，预算守卫用）
    uint32_t fpsCnt = 0;
    int64_t  fpsT0 = esp_timer_get_time();
    // 5s 诊断聚合
    uint32_t dFrames = 0, dDecMax = 0, dBlitMax = 0;
    int      dState0 = state;
    int64_t  dT0 = esp_timer_get_time();

    ESP_LOGI(TAG, "伺服启动：seg=%u 帧%u kf%u（帧号对拍）",
             (unsigned)curSegId, (unsigned)sess0->frameCount, sess0->kfInterval);

    TickType_t lastTick = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&lastTick, pdMS_TO_TICKS(66));

        // 会话存活检查（TCP 断开 / App 降级实时 → 退出伺服，调用方重画状态页）
        gtwc::Session s;
        if (!gtwc::session(&s) || s.w == 0 || !s.isV2 || s.mode != 1) return;
        const int64_t now = esp_timer_get_time();

        // ── 0xD0 快照（BTC 任务跨核写，走锁）──
        ble_conductor_target_t tgt;
        ble_conductor_target_snapshot(&tgt);
        const bool haveTgt = tgt.valid;
        const int64_t lastUs = haveTgt
            ? (int64_t)tgt.last_tick * (int64_t)portTICK_PERIOD_MS * 1000 : 0;
        const bool silent = !haveTgt || (now - lastUs) > SILENT_US;

        // ── flags 处理（暂停/恢复/段重置；§5.1 对应行）──
        if (haveTgt) {
            if (tgt.flags & 0x01) {   // 段重置（bit0）：清池重来；paused 清除（裁决 3）
                gtwc::poolClearRequest();
                curSegId = tgt.seg_id;
                state = SV_RESET; havePlay = false; skipping = false;
                paused = false; playIdx = 0;
                clearPending = true;
                resetT0 = now;
                ESP_LOGW(TAG, "段重置 → RESET seg=%u", (unsigned)curSegId);
            }
            if (tgt.flags & 0x02) paused = true;    // bit1 暂停
            if (tgt.flags & 0x04) paused = false;   // bit2 恢复
        }

        // ── 0xE3 播放头回显（10Hz；帧号对拍：直接回段内帧号，App 端零换算）──
        if (now - lastReport >= 100000) {
            lastReport = now;
            if (ble_conductor_is_connected() && ble_conductor_is_e2_subscribed()) {
                uint8_t rep[10];
                rep[0] = 0xE3;
                rep[1] = (uint8_t)(curSegId);
                rep[2] = (uint8_t)(curSegId >> 8);
                rep[3] = (uint8_t)(curSegId >> 16);
                rep[4] = (uint8_t)(curSegId >> 24);
                rep[5] = (uint8_t)(playIdx);
                rep[6] = (uint8_t)(playIdx >> 8);
                rep[7] = (uint8_t)(playIdx >> 16);
                rep[8] = (uint8_t)(playIdx >> 24);
                rep[9] = (uint8_t)state;
                ble_conductor_send_notify(rep, sizeof(rep));
            }
        }

        // ── SILENT 优先（裁决 1）：没有目标里程无从谈池 ──
        if (silent) {
            if (state != SV_SILENT) {
                ESP_LOGW(TAG, "BLE 静默 >6s → 冻结（保持最后帧）");
                state = SV_SILENT;
            }
            continue;
        }
        if (state == SV_SILENT) {   // 静默恢复：先收目标、再查池（裁决 1 衔接行）
            state = havePlay ? SV_HOLD : SV_RESET;
            ESP_LOGI(TAG, "静默恢复 → %s", svName(state));
        }

        // ── RESET：等清池生效 + keyframe 起播 ──
        if (state == SV_RESET) {
            const uint32_t g = gtwc::poolGeneration();
            if (g != myGen) { myGen = g; clearPending = false; resetT0 = now; }
            const uint8_t* p = nullptr; uint32_t sz = 0, seq = 0, li = 0; bool isKey = false;
            if (gtwc::pop(0, &p, &sz, &seq, &isKey, &li)) {
                if (clearPending || !isKey) {
                    // 旧段帧 / 非 keyframe：丢弃不解（delta 链不可信）
                    continue;
                }
                const int32_t dms = decode_into_ring(p, sz);
                if (dms >= 0) {
                    if (dms > 0) kfDecMs = dms;
                    playIdx = li; havePlay = true; state = SV_PLAY;
                    present_frame();
                    ESP_LOGI(TAG, "段起播 keyframe idx=%u（decode %ums）", (unsigned)li, dms);
                }
            } else if (now - resetT0 > RESET_TIMEOUT_US) {
                ESP_LOGE(TAG, "RESET 5s 无 keyframe → STALL（0xE3 上报，等 App 重发段头）");
                state = SV_STALL;
                stallT0 = now;
            }
            continue;
        }

        if (!havePlay) { state = SV_RESET; continue; }   // 表外兜底：无画面回 RESET 等段

        // ── 运行态（PLAY/HOLD/STALL 统一路径，state 记最近一次落点）──
        // ★ 帧号对拍（2026-10-28 二轮意见①）：目标=0xD0 载荷直读段内帧号，
        //   与 pop() 的 localIdx（收流顺序计数）同源同刻度，无里程换算。
        int64_t targetIdx = (int64_t)tgt.frame_idx;
        if (targetIdx < 0) targetIdx = 0;
        int64_t gap = targetIdx - (int64_t)playIdx;

        if (paused) {   // 暂停：保持当前帧，池照常回填（§5.1）
            state = SV_HOLD;
            continue;
        }
        if (gap <= 0) { // 到站保持 / 回退 1~8 帧保持（不倒放，裁决 2）
            if (gap < -BT_BACK && now - lastBackWarn > 3000000) {
                lastBackWarn = now;
                ESP_LOGW(TAG, "目标回退 %lld 帧（>8，不倒放；App 应走段重置）",
                         (long long)(-gap));
            }
            state = SV_HOLD;
            continue;
        }
        if (gap > KF_JUMP) skipping = true;   // 大缺口：跳 keyframe，不解中间 delta

        // 追帧：tick 预算守卫（三审④）——剩余 ≥ 预估解码 + blit + 裕度才解下一帧；
        // 超预算缺口顺延下 tick，永不超时掉拍。只 blit 最新一帧（N 次 decode 后窗口即最新）。
        const int64_t tickT0 = now;
        uint32_t chased = 0;
        bool     stalled = false;
        while (gap > 0 || skipping) {
            const int64_t elapsed = esp_timer_get_time() - tickT0;
            const int32_t est = skipping ? kfDecMs : DELTA_EST_MS;
            if (elapsed + est + BLIT_RES_MS * 1000 + MARGIN_MS * 1000 > TICK_US) break;
            const uint8_t* p = nullptr; uint32_t sz = 0, seq = 0, li = 0; bool isKey = false;
            if (!gtwc::pop(0, &p, &sz, &seq, &isKey, &li)) { stalled = true; break; }  // 池内无后续帧
            if (skipping) {
                if (!isKey) continue;    // 丢弃 delta（链已断）
                skipping = false;        // keyframe 到：恢复正常解码
            }
            const int32_t dms = decode_into_ring(p, sz);
            if (dms < 0) {
                ESP_LOGE(TAG, "帧 idx=%u 解码失败（%uB）", (unsigned)li, (unsigned)sz);
                playIdx = li;
                gap = targetIdx - (int64_t)playIdx;
                continue;
            }
            if (isKey && dms > 0) kfDecMs = dms;
            playIdx = li;
            gap = targetIdx - (int64_t)playIdx;
            chased++;
        }
        if (chased > 0) {
            present_frame();
            fpsCnt++;
            if (now - fpsT0 >= 1000000) {
                hud_render((int)(fpsCnt * 1000000 / (now - fpsT0)));
                fpsCnt = 0; fpsT0 = now;
            }
        }
        state = stalled ? SV_STALL : (gap <= 0 ? SV_HOLD : SV_PLAY);
        if (stalled) {
            if (stallT0 == 0) stallT0 = now;
            else if (now - stallT0 > STALL_WARN_US) {
                ESP_LOGW(TAG, "STALL 持续 >30s（0xE3 持续上报 state=2，等 App 处置）");
                stallT0 = now;
            }
        } else {
            stallT0 = 0;
        }

        // 5s 诊断
        dFrames += chased;
        if ((now - dT0) >= 5000000) {
            ESP_LOGI(TAG, "servo 5s: 态%s→%s 帧%u play%lld target%lld 池%d kfDec%ums",
                     svName(dState0), svName(state), (unsigned)dFrames,
                     (long long)playIdx, (long long)targetIdx,
                     gtwc::depth(), kfDecMs);
            dFrames = 0; dDecMax = 0; dBlitMax = 0; dState0 = state; dT0 = now;
        }
    }
}
#endif  // GTAM_WIFI_CAST
static void playback_cast_task(void*) {
    s_disp = lv_display_get_default();
    if (!s_disp) { ESP_LOGE(TAG, "没有 LVGL display"); vTaskDelete(nullptr); return; }
    const int scrW = (int)lv_display_get_horizontal_resolution(s_disp);
    const int scrH = (int)lv_display_get_vertical_resolution(s_disp);
    s_w = (uint32_t)scrW;
    s_h = (uint32_t)scrH;

    // 环缓冲：h + RING_MARGIN 行（与内嵌路径同一笔账）
    s_ringH = (int)s_h + RING_MARGIN;
    s_ring = (uint16_t*)heap_caps_malloc((size_t)s_ringH * s_w * 2,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        ESP_LOGE(TAG, "环缓冲分配失败（%u KB PSRAM）",
                 (unsigned)(s_ringH * s_w * 2 / 1024));
        vTaskDelete(nullptr);
        return;
    }
    std::memset(s_ring, 0, (size_t)s_ringH * s_w * 2);
    s_oy = RING_MARGIN / 2;

    s_hudBuf = (uint16_t*)heap_caps_malloc(HUD_W * HUD_H * 2, MALLOC_CAP_8BIT);
    if (!s_hudBuf) ESP_LOGW(TAG, "HUD 缓冲分配失败，屏上无 fps（串口照常）");

    // ★ 直写开关：LVGL 渲染整个旁路，屏幕全归本任务（状态页也是直写 blit）
    const esp_err_t e = esp_lv_adapter_set_dummy_draw(s_disp, true);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "dummy draw 开启失败：%s", esp_err_to_name(e));
        vTaskDelete(nullptr);
        return;
    }

    // 收流端（SoftAP + TCP + 3MB 队列）。失败不 abort——屏上 WAIT 页照画，串口看错
    if (gtwc::start() != 0) ESP_LOGE(TAG, "wifi_cast 启动失败（无 SoftAP，屏停 WAIT）");

    ESP_LOGI(TAG, "WiFi 投屏模式：屏 %ux%u，环 %u 行 | SPIRAM 余 %u KB",
             (unsigned)s_w, (unsigned)s_h, (unsigned)s_ringH,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    int drawn = -1;                  // 已画到屏上的状态（-1 = 未画，强制首绘）
    char drawnIp[16] = "";           // WAIT 页已画的 IP（STA 模式 DHCP 异步到手要重画）
    uint32_t fpsCnt = 0;             // 秒级 fps 统计
    int64_t  fpsT0 = esp_timer_get_time();
    uint32_t lastSeq = 0;
    bool     seqInit = false;
    bool     sizeErrDrawn = false;

    // 播放侧探针：每 5s 聚合（pop等待=生产慢于消费的证据；解码/blit=自身开销）
    uint32_t pFrames = 0, pPopMax = 0, pDecMax = 0, pBlitMax = 0;
    uint64_t pPopSum = 0, pDecSum = 0, pBlitSum = 0;
    int64_t pT0 = esp_timer_get_time();
    while (1) {
        // ★ 会话 v2 mode=1 → 池伺服（方案 §5）；其余（v1 / v2 mode=0）走实时路径回退
        {
            gtwc::Session sm;
            if (gtwc::session(&sm) && sm.w != 0 && sm.isV2 && sm.mode == 1) {
                servo_loop(&sm);
                drawn = -1;      // 退出伺服（TCP 断/模式切）强制重画状态页
                sizeErrDrawn = false;
                continue;
            }
        }
        const int64_t tPop0 = esp_timer_get_time();
        const uint8_t* p = nullptr;
        uint32_t sz = 0, seq = 0;
        const bool got = gtwc::pop(200, &p, &sz, &seq);
        const uint32_t popWait = (uint32_t)((esp_timer_get_time() - tPop0) / 1000);

        if (!got) {
            // 无帧：状态页（状态变了重画；WAIT 页 IP 变了也重画——STA 模式 DHCP
            //   异步到手，首绘时还是 0.0.0.0，必须跟着 s_ipText 刷新）
            const gtwc::State st = gtwc::state();
            const bool ipChanged = (st == gtwc::State::Wait) &&
                                   std::strncmp(gtwc::ipText(), drawnIp, sizeof(drawnIp)) != 0;
            if ((int)st != drawn || ipChanged) {
                draw_status_screen(st);
                drawn = (int)st;
                if (st == gtwc::State::Wait)
                    std::strncpy(drawnIp, gtwc::ipText(), sizeof(drawnIp) - 1);
                sizeErrDrawn = false;
            }
            continue;
        }

        // 会话尺寸校验（直写要求帧=屏；不符则不出图，让背压停住 App）
        gtwc::Session sess;
        if (gtwc::session(&sess) && sess.w != 0 &&
            ((int)sess.w != (int)s_w || (int)sess.h != (int)s_h)) {
            if (!sizeErrDrawn) {
                ESP_LOGE(TAG, "会话 %ux%u 与屏幕 %ux%u 不符，拒播",
                         sess.w, sess.h, (unsigned)s_w, (unsigned)s_h);
                draw_size_err(sess);
                sizeErrDrawn = true;
            }
            vTaskDelay(pdMS_TO_TICKS(100));   // 慢慢耗（不 pop，让 TCP 窗口堵住 App）
            continue;
        }

        drawn = (int)gtwc::State::Play;
        sizeErrDrawn = false;

        // seq 断流诊断（帧头 kCastOffSeq=17，LE u32；App 侧逐帧递增）
        if (seqInit && seq != lastSeq + 1) {
            if (seq <= lastSeq)
                ESP_LOGW(TAG, "seq 归零 %u→%u（新会话/队列混串告警）", (unsigned)lastSeq, (unsigned)seq);
            else
                ESP_LOGW(TAG, "seq 跳变 %u→%u（丢 %u 帧）",
                         (unsigned)lastSeq, (unsigned)seq,
                         (unsigned)(seq - lastSeq - 1));
        }
        lastSeq = seq;
        seqInit = true;

        // 解码（isKey/滑窗判定在助手内）+ 推屏（时序纪律不变：解码写完才推）
        const int32_t decMs = decode_into_ring(p, sz);
        const bool ok = decMs >= 0;

        // ② 推屏（分带异步入队，DMA 后台推完）
        esp_err_t er = ESP_FAIL;
        uint32_t blitMs = 0;
        if (ok) {
            const int64_t tB0 = esp_timer_get_time();
            er = blit_banded(s_ring + (size_t)s_oy * s_w);
            blitMs = (uint32_t)((esp_timer_get_time() - tB0) / 1000);
        }
        // HUD 常驻：地图每帧全屏推，必须每帧补推
        if (s_hudBuf)
            esp_lv_adapter_dummy_draw_blit(s_disp, HUD_X, HUD_Y,
                                           HUD_X + HUD_W, HUD_Y + HUD_H,
                                           s_hudBuf, false);
        if (!ok || er != ESP_OK)
            ESP_LOGE(TAG, "帧失败 decode=%d blit=%s sz=%u",
                     (int)ok, esp_err_to_name(er), (unsigned)sz);

        // 秒级 fps → HUD
        fpsCnt++;
        const int64_t now = esp_timer_get_time();
        if (now - fpsT0 >= 1000000) {
            hud_render((int)(fpsCnt * 1000000 / (now - fpsT0)));
            fpsCnt = 0;
            fpsT0 = now;
        }

        // 播放侧 5s 聚合（pop等待峰=生产慢；解码/blit=自身开销）
        pFrames++;
        pPopSum += popWait; if (popWait > pPopMax) pPopMax = popWait;
        {
            const uint32_t dm = ok ? (uint32_t)decMs : 0;
            pDecSum += dm; if (dm > pDecMax) pDecMax = dm;
            pBlitSum += blitMs; if (blitMs > pBlitMax) pBlitMax = blitMs;
        }
        if ((esp_timer_get_time() - pT0) >= 5000000) {
            ESP_LOGI(TAG, "play 5s: 帧%u pop等待峰%ums 解码均%.1f峰%ums blit均%.1f峰%ums 队列深%d",
                     (unsigned)pFrames, (unsigned)pPopMax,
                     (double)pDecSum / pFrames, (unsigned)pDecMax,
                     (double)pBlitSum / pFrames, (unsigned)pBlitMax,
                     gtwc::depth());
            pFrames = 0; pPopMax = 0; pDecMax = 0; pBlitMax = 0;
            pPopSum = 0; pDecSum = 0; pBlitSum = 0;
            pT0 = esp_timer_get_time();
        }
    }
}

#else
// ════════════════ 内嵌 NVF1 演示路径（原样保留，勿改） ════════════════

static void playback_task(void*) {
    const uint8_t* base = &_binary_nav_frames_bin_start[0];
    const size_t   len  = (size_t)((uintptr_t)&_binary_nav_frames_bin_end[0] -
                                   (uintptr_t)base);
    if (len < 24 || std::memcmp(base, "NVF1", 4) != 0) {
        ESP_LOGE(TAG, "不是 NVF1 容器（len=%u magic=%.4s）",
                 (unsigned)len, (const char*)base);
        vTaskDelete(nullptr);
        return;
    }
    s_nFrames = rd32(base + 4);
    s_w       = rd32(base + 8);
    s_h       = rd32(base + 12);
    s_fps     = rd32(base + 16);
    if (s_nFrames == 0 || s_w == 0 || s_h == 0) { ESP_LOGE(TAG, "容器头非法"); vTaskDelete(nullptr); return; }
    s_tab     = base + 24;
    s_data    = s_tab + (size_t)s_nFrames * 8;
    s_dataLen = len - (size_t)(s_data - base);
    ESP_LOGI(TAG, "NVF1：%u 帧 %ux%u @%ufps  数据 %u KB（均 %.1fKB/帧）",
             (unsigned)s_nFrames, (unsigned)s_w, (unsigned)s_h, (unsigned)s_fps,
             (unsigned)(s_dataLen / 1024), (double)s_dataLen / s_nFrames / 1024.0);

    s_disp = lv_display_get_default();
    if (!s_disp) { ESP_LOGE(TAG, "没有 LVGL display"); vTaskDelete(nullptr); return; }
    const int scrW = (int)lv_display_get_horizontal_resolution(s_disp);
    const int scrH = (int)lv_display_get_vertical_resolution(s_disp);
    // 直写模式下帧尺寸必须等于屏尺寸：blit 是按帧矩形推的，尺寸不符会推歪/推越界
    if (scrW != (int)s_w || scrH != (int)s_h) {
        ESP_LOGE(TAG, "帧 %ux%u 与屏幕 %dx%d 不符，直写不能成立",
                 (unsigned)s_w, (unsigned)s_h, scrW, scrH);
        vTaskDelete(nullptr);
        return;
    }

    // 环缓冲：h + RING_MARGIN 行。多出来的余量就是「竖直位移不搬像素」的本钱。
    s_ringH = (int)s_h + RING_MARGIN;
    s_ring = (uint16_t*)heap_caps_malloc((size_t)s_ringH * s_w * 2,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        ESP_LOGE(TAG, "环缓冲分配失败（%u KB PSRAM）",
                 (unsigned)(s_ringH * s_w * 2 / 1024));
        vTaskDelete(nullptr);
        return;
    }
    // 窗口外的余量区不会被推屏，但填充一次可杜绝「万一有像素没被 run 覆盖」
    // 时把随机内存推上屏（首帧 keyframe 全覆盖，之后靠边行 run；双保险）
    std::memset(s_ring, 0, (size_t)s_ringH * s_w * 2);
    s_oy = RING_MARGIN / 2;

    s_hudBuf = (uint16_t*)heap_caps_malloc(HUD_W * HUD_H * 2, MALLOC_CAP_8BIT);
    if (!s_hudBuf) ESP_LOGW(TAG, "HUD 缓冲分配失败，屏上无 fps（串口照常）");

    // ★ 直写的开关：LVGL 渲染整个旁路（不 refresh、不 flush、不碰 framebuffer）
    const esp_err_t e = esp_lv_adapter_set_dummy_draw(s_disp, true);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "dummy draw 开启失败：%s", esp_err_to_name(e));
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "直写+滑窗：环 %ux%d（窗口 %u 行，余量 %d 行）  %u KB PSRAM",
             (unsigned)s_w, s_ringH, (unsigned)s_h, RING_MARGIN,
             (unsigned)(s_ringH * s_w * 2 / 1024));

    const int periodMs = (s_fps >= 1 && s_fps <= 120) ? (int)(1000 / s_fps) : 33;
    if (s_hudBuf) hud_render((int)s_fps);   // 首轮就有内容，别先推一帧黑块
    ESP_LOGI(TAG, "开播：周期 %dms（目标 %ufps）", periodMs, (unsigned)s_fps);

    TickType_t last = xTaskGetTickCount();
    while (1) {
        uint64_t sumDec = 0, maxDec = 0, sumBlit = 0, maxBlit = 0;
        uint64_t sumWait = 0, sumReb = 0;
        uint32_t bad = 0, nSlide = 0, nRebase = 0;
        const int64_t loopT0 = esp_timer_get_time();

        for (uint32_t i = 0; i < s_nFrames; i++) {
            const uint8_t* tb = s_tab + (size_t)i * 8;
            const uint32_t off = rd32(tb), sz = rd32(tb + 4);
            if ((size_t)off + (size_t)sz > s_dataLen || sz < (uint32_t)gtam::kCastHeadBytes) {
                ESP_LOGE(TAG, "帧 %u 越界/过短 off=%u sz=%u", (unsigned)i, (unsigned)off, (unsigned)sz);
                bad++;
                vTaskDelayUntil(&last, pdMS_TO_TICKS(periodMs));
                continue;
            }
            const uint8_t* p = s_data + off;
            const bool isKey = (p[gtam::kCastOffFlags] & 1) != 0;
            const int16_t shX = (int16_t)(p[gtam::kCastOffShiftX] |
                                          (p[gtam::kCastOffShiftX + 1] << 8));
            const int16_t shY = (int16_t)(p[gtam::kCastOffShiftY] |
                                          (p[gtam::kCastOffShiftY + 1] << 8));
            // 只有「纯竖直位移」能滑窗：横向位移无法靠基址偏移实现（推屏驱动
            // 要求源按行紧密排布，水平滑动会让每行起点各不相同）
            const bool canSlide = !isKey && shX == 0 && shY != 0;

            if (isKey) {
                s_oy = RING_MARGIN / 2;      // 免费归位：反正整帧全写，不用搬
            } else if (canSlide) {
                int oy2 = s_oy - shY;        // 内容下移 sy>0 → 窗口基址上移
                if (oy2 < 0 || oy2 + (int)s_h > s_ringH) {
                    const int64_t tr = esp_timer_get_time();
                    std::memmove(s_ring + (size_t)(RING_MARGIN / 2) * s_w,
                                 s_ring + (size_t)s_oy * s_w,
                                 (size_t)s_h * s_w * 2);
                    s_oy = RING_MARGIN / 2;
                    oy2  = s_oy - shY;
                    sumReb += (uint64_t)(esp_timer_get_time() - tr);
                    nRebase++;
                }
                s_oy = oy2;
                nSlide++;
            }

            uint16_t* win = s_ring + (size_t)s_oy * s_w;

            // ① 解码（滑窗帧跳过内部 memmove，只打 run）—— 写全部在推屏之前
            const int64_t t0 = esp_timer_get_time();
            const bool ok = gtam::castDecode(p, sz, win, (int)s_w, (int)s_h,
                                             nullptr, nullptr, nullptr, nullptr,
                                             canSlide, s_pal);
            const int64_t t1 = esp_timer_get_time();

            // ② 推屏（异步：入队即返回，434KB 由 DMA 在后台 22ms 内推完）
            esp_err_t er = ESP_FAIL;
            if (ok)
                er = esp_lv_adapter_dummy_draw_blit(s_disp, 0, 0,
                                                    (int)s_w, (int)s_h, win, false);
            const int64_t t2 = esp_timer_get_time();
            // HUD 常驻：地图是全屏推的，每帧都会把它盖掉 → 必须每帧补推
            // （24×12 = 576B，SPI 开销可忽略；内容每轮才更新一次）
            if (s_hudBuf)
                esp_lv_adapter_dummy_draw_blit(s_disp, HUD_X, HUD_Y,
                                               HUD_X + HUD_W, HUD_Y + HUD_H,
                                               s_hudBuf, false);
            if (!ok || er != ESP_OK) {
                bad++;
                if (bad <= 3) ESP_LOGE(TAG, "帧 %u 失败 decode=%d blit=%s",
                                       (unsigned)i, (int)ok, esp_err_to_name(er));
            }

            const uint64_t dc = (uint64_t)(t1 - t0), bl = (uint64_t)(t2 - t1);
            sumDec += dc; if (dc > maxDec) maxDec = dc;
            sumBlit += bl; if (bl > maxBlit) maxBlit = bl;

            const int64_t waitT0 = esp_timer_get_time();
            vTaskDelayUntil(&last, pdMS_TO_TICKS(periodMs));
            sumWait += (uint64_t)(esp_timer_get_time() - waitT0);
        }

        const int64_t loopT1 = esp_timer_get_time();
        const double loopMs = (loopT1 - loopT0) / 1000.0;
        const double realFps = 1000.0 * s_nFrames / loopMs;

        ESP_LOGI(TAG,
                 "一轮 %u 帧 %.1fms → 实测 %.1f fps | 滑窗 %u 直接解 %u 失败 %u | "
                 "heap %u KB (min %u KB)",
                 (unsigned)s_nFrames, loopMs, realFps,
                 (unsigned)nSlide, (unsigned)(s_nFrames - nSlide), (unsigned)bad,
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                 (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM) / 1024));
        ESP_LOGI(TAG,
                 "分账 ms: decode %.2f (峰 %.2f) + blit %.2f (峰 %.2f) "
                 "| 归位 %u 次共 %.1fms | vTaskDelay %.2f | 每帧实付 %.2f → 余量 %.1f ms",
                 (double)sumDec / s_nFrames / 1000.0, (double)maxDec / 1000.0,
                 (double)sumBlit / s_nFrames / 1000.0, (double)maxBlit / 1000.0,
                 (unsigned)nRebase, (double)sumReb / 1000.0,
                 (double)sumWait / s_nFrames / 1000.0,
                 (double)(sumDec + sumBlit + sumReb) / s_nFrames / 1000.0,
                 periodMs - (double)(sumDec + sumBlit + sumReb) / s_nFrames / 1000.0);

        // HUD 内容每轮刷新一次（重绘 24×12 的 3 个字符，几十 us）
        if (s_hudBuf) hud_render((int)(realFps + 0.5));
    }
}

#endif  // GTAM_WIFI_CAST

extern "C" int nav_playback_start(void) {
    // 调色板换成大端（帧包约定；8 个数一次性，热路径无开销）
    for (int k = 0; k < gtam::kCastPaletteN; k++)
        s_pal[k] = gtam::castSwap16(gtam::kCastPalette[k]);
#if GTAM_WIFI_CAST
    BaseType_t r = xTaskCreatePinnedToCore(playback_cast_task, "nav_cast", 8192,
                                           nullptr, 1, &s_task, 1);
#else
    BaseType_t r = xTaskCreatePinnedToCore(playback_task, "nav_play", 8192,
                                           nullptr, 1, &s_task, 1);
#endif
    if (r != pdPASS || !s_task) {
        ESP_LOGE(TAG, "播放任务创建失败");
        return -1;
    }
    return 0;
}

#endif  // GTAM_NAV_PLAYBACK
