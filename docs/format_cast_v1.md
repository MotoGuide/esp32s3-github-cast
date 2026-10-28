# 投屏协议 v1（SCF1 · Screen Cast Frame）—— app 渲染 / ESP32 播放

2026-10-21 用户拍板架构：**app（iOS/Android）渲染导航界面 → WiFi（ESP32 SoftAP）
→ ESP32-S3 解码刷屏**。S3 降级为「优化过的播放器」，渲染算力全部搬到手机。
本文是双端（app 编码器 / ESP32 解码器）的共享契约；PC 侧 codec 已随附验证
（core/cast.h + tools/_tmp/test_cast.cpp）。

## 0. 为什么是索引色+RLE（而不是 JPEG/裸 RGB565）

- **GTA 地图天生适合**：~8 个固定色的大平色块 + 细 AA 边。palette run 一个百像素
  平色块只要 4 字节；AA 边走 literal run 逐像素 RGB565——**AA 不许关是红线**，
  纯索引色会丢抗锯齿，所以格式是三段式不是二段式（见 §3 run type）。
- **ESP32 解码 = 顺序内存写**：无 FPU 依赖、无查表随机访问，同族实测（EFC 类）
  ~20Mpix/s → 466² keyframe ≈ 11ms，delta 帧更低。JPEG 硬件解码 480P 要 72ms，
  且解完还要 YUV→RGB565 转换。RLE 赢在解码不在压缩率。
- 参考实测：a3d 组件（同板 CO5300 466²）FOX 动画全流程 54ms=18.5fps，其中纯
  blit 34.4ms——全屏推送常态天花板 ~20fps；**30fps 必须 delta 帧 + 局部刷屏**，
  这就是 §2/§3 差分机制的存在理由。

## 1. 拓扑与链路

```
手机 app（gtamap C++ core 渲染 → cast 编码 → UDP 分片）
        │ WiFi 2.4G（ESP32 SoftAP，单客户端）
        ▼
ESP32-S3（分片重组 → cast 解码 → 写 LVGL 画布 → 脏矩形局部刷屏）
```

- **SoftAP 模式**：不需要路由器，户外可用。SSID/密码编译期定死。
- **蜂窝不受影响**（用户第 2 点）：Android 默认行为——关联无外网的 WiFi 后流量
  自动走移动数据；iOS 机制同样支持（该 WiFi 被标记 unsatisfied，流量走蜂窝）。
  两端各有一条「无 internet 连接」提示属正常现象。**实测 TODO**（真机验证）。
- **反向通道**：ESP32 → app 只回 8B 心跳（fps/解码 ms/丢包计数），app 据此调
  keyframe 频率。控制面极小。
- **断链**（用户第 6 点）：ESP32 显示「无导航数据」占位；来数据即渲染。与本地
  瓦片方案断链死地图同级，无额外损失。

## 2. 传输层（UDP）

分片头 10B（LE）：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | magic u8 | 0x5C |
| 1 | ver u8 | 1 |
| 2 | type u8 | 0=数据分片 1=ESP32 心跳 |
| 3 | frameSeq u16 | 帧序号，单调递增 |
| 5 | chunkIdx u8 | 0..chunkCnt-1 |
| 6 | chunkCnt u8 | 本帧分片总数（≤255，帧 >355KB 才需多条） |
| 7 | payloadLen u16 | ≤1400 |

- 分片只按字节切帧字节流，不关心帧结构；ESP32 按 frameSeq 缓存重组。
- **丢片 = 整帧作废**，等下一个 keyframe。不重传（重传引入的延迟比丢帧贵）。
- keyframe 策略：每 30 帧强制一次，或上一帧差分率 >60% 时升级为 keyframe。
- 心跳（type 1）：`fps u8 / decMs u8 / drop u16LE / magic 0x5C`，8B，1Hz。

## 3. 帧格式（重组后的字节流）

帧头 16B（LE）：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | magic u8 | 0xC5 |
| 1 | ver u8 | 1 |
| 2 | flags u8 | bit0=keyframe（必须全覆盖画布） |
| 3 | nRuns u16 | run 数 |
| 5 | w u16 | 帧宽（466） |
| 7 | h u16 | 帧高（466） |
| 9 | dirtyX0/Y0/X1/Y1 u16×4 | run 覆盖 bbox（exclusive），供局部刷屏 |
| 17 | seq u32 | 与分片头 frameSeq 一致（跨层校验） |
| 21 | rsv u16 | 0 |

run 流（nRuns 个，屏幕行优先序）：

```
startDelta  varint(LEB128)   距上一 run 末的像素偏移（首个 run 距 0）
len         varint           ≥1
type        u8               0=palette 单色 / 1=color 单色 / 2=pixels 序列
payload     type0: 1B 调色板索引
             type1: 2B RGB565(LE)
             type2: len×2B RGB565(LE)
```

- **未覆盖像素保留上一帧内容**——delta 帧的机制。keyframe 的 run 必须覆盖
  w×h 全部像素（解码端按 absPos 累计校验，不允许跳空到覆盖外）。
- dirty bbox 由编码端统计；delta 帧变化碎（如只动一个车标）时 bbox 仍有效，
  真正碎到多矩形时 v2 改多 dirty 区。

## 4. 调色板 v1（固定 8 色，不随帧传）

| idx | RGB888 | RGB565 | 用途 |
|---|---|---|---|
| 0 | #1E1E1E | 0x18E3 | 陆地/底 |
| 1 | #B5B5B5 | 0xB5B6 | 道路 |
| 2 | #424242 | 0x4208 | 建筑 |
| 3 | #162434 | 0x1126 | 水 |
| 4 | #2E4030 | 0x2A06 | 绿化 |
| 5 | #0E0E0E | 0x0861 | casing |
| 6 | #050505 | 0x0020 | 桥缝 |
| 7 | #F2AC16 | 0xF562 | 路线/车标（占位，最终跟 UI 设计令牌走） |

- RGB565 值 = core/raster.h `rgb565()` 截断制实算，渲染器输出的 palette 色
  **逐位命中**（编码端线性扫 8 项即可，无需 64K 表）。
- 导航高亮/UI 元素若超 8 色 → v1.1：keyframe 带 16 色 palette 扩展（flags bit1）。

### 4.1 像素字节序：帧内一律大端（★ 2026-10-22 定，含一次真实事故）

上表的 RGB565 是**数值**（R 在高 5 位）。数值落到 2 个字节时还有顺序问题：
SPI 屏按大端收（先高字节），而 x86 / ESP32 都是小端 → 主机序直接推屏会整体翻字节。

**事故复盘**：直写通道上线后颜色全错，灰色（#1E1E1E / #B5B5B5，字节对称）看不出
变化，黄色路线、蓝色水面这些非对称色翻了。根因不在我们代码里——
`espressif__esp_lvgl_adapter/src/display/bridge/v9/lvgl_bridge_v9.c:2294`：

```c
if (profile.interface == ESP_LV_ADAPTER_PANEL_IF_OTHER &&
    lv_display_get_color_format(disp) == LV_COLOR_FORMAT_RGB565)
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_size(area));   // 推屏前偷偷换序
```

本 BSP（`esp32_s3_touch_amoled_1_75c.c:478`）两条条件都成立，所以走 LVGL 渲染
时颜色是对的；而直写通道 `dummy_draw_blit → esp_lcd_panel_draw_bitmap` 没有这一层。
`esp_lcd_panel_io` 的 `flags.swap_color_bytes` 也没在 `CO5300_PANEL_IO_QSPI_CONFIG`
里打开，驱动自己不做任何字节处理。

**约定**：帧包内所有像素（type1 literal 的 2B、palette 表项的实际字节）一律按
**大端**生成 —— 即屏驱动要的样子。解码端逐字节照抄即可推屏，热路径零成本。

- 生成端：`gen_nav_frames --endian=be|le`，**默认 be**。palette 必须与画布
  同步换序（`castSwap16` / `castSwapBuffer`，见 `core/cast.h`），否则 palette
  run 反查不中、解回错色。编码 / 平移检测 / 自检都在换序后的域内进行，
  逻辑不分叉（memcmp 与字节序无关）。
- 解码端：`castDecode(..., const uint16_t* pal)` 传大端调色板
  （`nav_playback.cpp` 启动时 `s_pal[k] = castSwap16(kCastPalette[k])`）。
- **若改回 LVGL 渲染路径**（它会 swap 一次）→ 必须用 `--endian=le`，否则翻两次。
- 验收：`tools/_tmp/chk_endian.cpp` —— BE 包解出的每个像素必须 ==
  `bswap16(LE 包对应像素)`。实测 100 帧 × 217156 px = **21,715,600 像素全 PASS**。
- 附：`build/nav_frames_le.bin` 是同一批画面的小端包，留作对拍基线。

## 5. 编码策略（app 侧）

1. 行内 RLE：同色连续段成 run；段内颜色在 palette → type0；不在 palette 但整段
   同色（少见，如混合出的一块同色）→ type1；异色 → type2，遇连续 ≥8 同色切出。
2. delta：与上一帧逐行 memcmp，相同行整体跳过（不产生 run，间距由下一 run 的
   startDelta 携带）。导航画面静态区占大头。
3. dirty bbox = run 覆盖范围（精确值，随帧统计）。
4. **PC 实测**（test_cast.exe，466² GTA 风合成帧，AA 边为逐像素异色——偏悲观）：
   keyframe **76.7KB（raw 434KB 的 17.7%）**、delta **2.6KB（keyframe 的 3.4%）**；
   编码 0.28ms / 解码 0.09ms（PC；ESP32 解码按 20Mpix/s 估 ~11ms）。
   真实导航内容平色占比更高，keyframe 预期优于此值。
5. 带宽账：30fps 全 keyframe = 2.3MB/s（SoftAP 撑得住）；delta 常态 78KB/s。

## 6. 解码与刷屏（ESP32 侧）

1. 重组 → 校验帧头（magic/ver/尺寸/seq）→ 逐 run 顺序写画布（就是
   s_canvasBuf 中心 466² 窗口，RGB565，PSRAM）。
2. run 写全部是顺序流式：palette/单色 run 是 fill loop，type2 是 memcpy——
   **cache line 友好**（PSRAM 顺序访问走 cache line，与 TileCache 同款经验）。
3. 双 canvas 缓冲：解到备用画布，整帧完成后指针 swap + `lv_obj_invalidate_area`
   只刷 dirty bbox。刷屏成本 = bbox 面积，不是全屏。
4. 帧率账（466²，预估）：keyframe 解码 ~11ms + 刷屏（局部，车周 100² ≈ 2ms；
   全屏最坏 34ms）→ delta 常态 15-25ms/帧 = 40-60fps 上限，30fps 有余量。
   网络抖动是主要风险，用心跳监控。

## 7. 验收（PC 已过，ESP32 端待上板）

- **位等价**：decode(encode(frame)) 与源帧逐字节一致（keyframe + delta 两种模式，
  含 AA 混合色 literal run）——core/cast.h + test_cast.cpp 已实现并通过。
- 负控有牙：magic 错/截断/调色板索引越界/run 长度造假 → 全部检出拒绝。
- ESP32 端：解码速度 ≥15Mpix/s、局部刷屏延迟、SoftAP 断连重连、心跳闭环。
  （ESP-IDF 编译由用户执行。）

## 8. 演进路线

- v1.1：keyframe 携带 16 色扩展 palette（flags bit1）。
- v2：run 级差分（同行跳过太粗时）+ 多 dirty 矩形。
- v3：ESP32→app 上行带触摸事件（把屏幕当触控板回传给 app）。
- 远期：SSL？不——SoftAP 封闭链路 + 帧数据无敏感信息，v1 明文。
