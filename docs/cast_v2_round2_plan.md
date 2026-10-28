# 投屏 v2 第二轮实施清单：帧号对拍 / 16MB flash 池 / 帧压缩

> 2026-10-28 晚，用户三条意见的落地设计。前史见 cast_pool_servo_plan.md（第一步已跑通）。
> App 侧 = Android 测试工程 `app/src/debug/.../WifiCastTestPanel.kt`（全圈 debug，生产零共享）；
> 设备侧 = 02_lvgl_demo_v9 `main/`。

## 0. 本轮查证的前置事实（设计依据）

| 事实 | 出处 |
|---|---|
| flash 32MB | sdkconfig `CONFIG_ESPTOOLPY_FLASHSIZE="32MB"` |
| 分区：nvs+phy+factory 8M+tiles 14M，0x1610000 后空闲仅 ~9.94MB | partitions.csv |
| **tiles 分区已废弃**：GTS1 自 2026-10-17 起走 `target_add_binary_data` 内嵌 app .rodata | gtam_glue.cpp:900 注释 + CMakeLists.txt:39 |
| app bin 实际 ~1.6MB（播放演示态），GTS1 包 6.36MB | build/lvgl_demo_v9.bin、gtam_glue.cpp:59 |
| **espressif__zlib 已在构建依赖**（PRIV_REQUIRES） | main/CMakeLists.txt:10 |
| 当前池 = PSRAM 4MB，16 桶×256KB，帧上限 252KB | wifi_cast.cpp Q_SLOT/Q_SLOTS |
| PSRAM 8MB 已用 ~4.7MB（池+解码环） ⇒ PSRAM 无 16MB 空间 | wifi_cast.cpp 内存账 |
| WiFi 实测 ~350KB/s ≈ 2fps（164KB/帧），拐弯段 delta≈keyframe | 10-28 实测 |

## 1. 意见① 帧号对拍（里程 → 帧号，整数 1:1）—— ✅ 已实施（2026-10-28 晚）

**动机**：伺服按里程对拍要过 `meterCmToIdx`（segStepCm 除法取整），App 端 0xD0 目标 =
虚拟车实时里程再钳 `playhead+12m`，两次换算两处舍入，且有跳帧嫌疑。帧号是纯整数，
收流顺序计数（`pop()` 的 localIdx / c_idx），天然 1:1。

**线协议（字节布局不变，字段语义替换；无兼容负担，App/固件同轮升级）**：

```
0xD0 10B: [0xD0][segId u32 LE][frameIdx u32 LE][flags u8]   ← meter_m → frameIdx
0xE3 10B: [0xE3][segId u32 LE][playheadFrame u32 LE][state u8] ← meter → playheadFrame
```

**App 改动（WifiCastTestPanel.kt）**：
1. beat 协程：虚拟车里程 → `carFrame = (carMeterBle * 100 / segStepCm).toLong()`（整数除法）；
   钳制改 `LEAD_FRAMES = 11`（原 LEAD_M=12m ÷ 1.11m/帧 ≈ 11 帧），且 clamp 到
   `frameCount-1`；目标单调不倒退逻辑保留。
2. 0xE3 解析（L347 附近）：`bb.int` 直接存 `playheadFrame`（atomic），删掉米→帧换算。
3. 池水位：`writeIdx − playheadFrame`（纯整数，原 L448 的 `ph*100/segStepCm` 删）。

**设备改动**：
1. `ble_conductor.cpp`：0xD0 解析处（L280-294）字段改名/注释 `meter_m → frame_idx`，
   日志同步。结构体布局不动（ble_conductor.h `ble_conductor_target_t`）。
2. `nav_playback.cpp` servo_loop：
   - `targetIdx = (int64_t)tgt.frame_idx` 直读（删 L463 `meterCmToIdx`，clamp≥0 保留）；
   - 0xE3 回显（L412）：`rep[5..8] = playIdx` 直接写帧号（删 `curSegM0cm+playIdx*stepCm` 换算）；
   - 段重置（L390）：`curSegM0cm` 相关可删（仅显示用）；frameIdx 是段内号，
     段重置后 App 从 0 重计 + 设备 generation 清 c_idx，两侧天然对齐。
3. 会话头 v2 字段不动（segStepCm/segM0m 保留，仅诊断显示）。

**验收**：0xD0/0xE3 帧号逐位对拍；servo log `play = target − LEAD_FRAMES` 邻域；
池够时无 STALL；看门狗判定改用帧号差。

## 2. 意见③ 帧压缩（SCF1 之上再 deflate；建议先做，理由见 §4）

**现状痛点**：SCF1 run 流对直行已高效（delta ~3KB），但 **拐弯段 delta≈keyframe
（164KB）**，n 帧拐弯直接吃爆 4MB 池（当前 STALL 主因之一）。

**方案：传输层压缩**。帧包 `[u32 len][SCF1]` → `[u32 clen][u32 rawLen][zlib deflate(SCF1)]`：

- **App 编码端**：`GtamRenderJni.castEncode()`（JNI，C++ 侧）之后接 deflate。
  建议在 C++ 侧做（JNI 桥内，与 PC 共用代码，三端可对拍）；Kotlin `java.util.zip.Deflater`
  兜底亦可。level 6 起步。
- **设备端**：tcp_task `recv_exact` 收压缩帧 → `uncompress()`（espressif__zlib，现成）
  inflate 到 4KB 暂存 → 逐段写入桶。**桶内存的永远是解压后的 SCF1 ⇒ 桶布局/解码/
  伺服路径零改动**。
- **红线核对**：deflate 无损，SCF1 逐位一致 ⇒ 渲染零损失红线不破（与「AA 不关」同级）。
- **预期收益**：keyframe 164KB→40~60KB（大色块+varint 冗余高）；拐弯段池耗 ÷3~4；
  WiFi 350KB/s 下 2fps→6~8fps。
- **成本**：inflate 窗口内存（zlib 组件可配 32K）；设备端 inflate ~5-15ms/帧（240MHz，
  须实测），tick 66ms 预算内可行（解压在生产者侧，不占解码预算）；App 侧编码时延增加实测。

**决策点（留给用户）**：传输层压缩（桶存明文 SCF1，改动最小）vs 存储层压缩（桶存
压缩帧、pop 时 inflate，池容量再 ×3~4）。**建议第一版只压传输层**；flash 池上马后
若 16MB 还不够再升存储层（pop 边界 inflate，decode 路径不变）。

## 3. 意见② flash 16MB 池

**partitions.csv 调整**（tiles 回收是空间来源）：

```
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, ,        8M,      # GTS1 6.36MB 内嵌 .rodata 的宿主，不能缩
castpool, data, spiffs,  ,        16M,     # @0x810000 → 0x1810000，余 ~8MB 备用
# tiles 已删除：GTS1 自 10-17 起内嵌 app .rodata，分区无人用
```

⚠️ 分区表变更 = **全量重烧**（分区表+app；GTS1 在 app bin 内随烧，无需单独烧 tiles）。
app 分区偏移未动（0x10000），Kconfig.projbuild 无需改。

**wifi_cast 后端替换（桶式算法一字不改，Q_SLOTS 16→64，q_buf 从 PSRAM 换 flash）**：

- **读（消费者）**：`esp_partition_mmap` 整分区一次映射（S3 MMU 足够 16MB），
  pop 路径照旧零拷贝读。
- **写（生产者）**：flash 不能写 mmap —— `recv_exact` 进 4KB PSRAM 暂存 →
  4KB 页边界先 `esp_partition_erase_range`（4KB sector）再 `esp_partition_write`。
- **★ 擦除是瓶颈**：4KB sector erase ~20-40ms ⇒ 裸写吞吐仅 ~100-130KB/s，
  **低于当前 350KB/s 到手速率** ⇒ flash 池**必须依赖帧压缩先落地**（压缩后流
  ~50-90KB/s，写入绰绰有余）。不引入 erase-ahead 帮工任务（第一期从简；若压缩后
  仍紧张再加：空闲期预擦写头前方 8~16 sector，写头逼近握手让路）。
- 桶发布/背压/清池/generation/q_head/q_tail 语义与 PSRAM 版完全一致。
- **寿命账**：16MB 全轮转一周 = 16MB × 5万次 P/E = 800GB 写入量；压缩后 675KB/s
  峰值仅追帧时出现，导航场景寿命无忧。
- **风险（须实测）**：ESP32-S3 flash 操作（尤其 erase）期间禁 cache，WiFi 收流/
  播放会有 ms 级停顿窗口；350KB/s→90KB/s 后 erase 频率低，预计可接受，实测确认。
  桶池分配失败路径（原 PSRAM calloc）改为分区查找失败报错。

## 4. 实施顺序建议（按依赖，与意见编号无关）

| 步 | 内容 | 为什么这个序 |
|---|---|---|
| ① | 帧号对拍（§1） | 最小改动 4 文件，立刻提升伺服精度，独立无依赖 |
| ② | 帧压缩·传输层（§2） | 收益最大（池容量+WiFi 帧率双 ×3~4），且是 flash 池的吞吐前提 |
| ③ | flash 16MB 池（§3） | 依赖②的写入吞吐；分区表变更需全量重烧一次，放最后 |

每步独立装机验收；App 侧全部在 debug 源集，生产 `:ble` / 生产链路零触碰。

## 5. 遗留提醒（不阻塞本轮）

- 杀软白名单（Android 工程目录）仍未落实，dex 锁已复发 4 次，重烧季建议先办。
- 存储层压缩（桶内压缩帧）作为 flash 池后的下一步备选，本清单不实施。

## 6. 渲染提速侦查（2026-10-28 晚，静态审查；动手前先看 App 阶段计时数据）

背景：压缩后 WiFi 只需 ~50KB/s（富余），瓶颈移回手机出帧率 ~1.5fps（拐弯段
keyframe 级 ~650ms/帧）。已排除：NDK debug -O0（gtam-render CMakeLists:28 已
`-O2 -g`）。App 填池循环已加「阶段均摊（渲染/编码/写网 ms）」日志（每 30 帧），
先跑一轮拿数据再定主攻。静态嫌疑清单（按可疑度）：

1. **Scene3 每帧全量重建**（render.h:580 `pool/faces/ground.clear()`）：idx 查询 →
   tile 解码（pf_dec 段）→ 投影 → 排序，每帧重来。设备端有 TileCache 缓解
   （02_lvgl_demo_v9/tile_cache.h），**App cast 路径没接**——候选方案 = 桥接层做
   位图缓存或局部重绘。
2. **drawRouteLayer 每帧全量投影整条路线**（route_layer.h:110-130）+ sp/done/rest
   三个 std::vector 每帧堆分配；只有「整条不可见」才跳过，无视口分段裁剪；
   旋转模式还在放大的 scratch 画布上画全量。4.5km 路线点数上千时每帧三次全量
   strokePolyline（casing+已走+未走）。
3. rotateBlit 全幅旋转（scratch ~660×660 读 + 466×466 写，2× 屏幕面积）。
4. castEncode 的 deflate（本轮已降 level 1，若阶段计时显示编码仍占大头再攻：
   移出渲染线程异步化 / 换 libdeflate）。

红线：任何优化不得改变输出像素（渲染零损失）——视口分段裁剪只能在 ±48px
扩展边距外丢段（画布外像素本来就没有）。

---

## 实施状态（2026-10-28 晚收尾）

- 意见① 帧号对拍：**已落地**（App beat/解析 + ble_conductor + nav_playback servo）。
  22:59 会话实测：0xD0 10Hz 稳发、伺服追踪、write 90→357 持续推进、全链路首次闭环。
  ★ 注意 22:10~22:44 间曾引入「裸块」bug（发送被包进独立 `{}` 永不执行）已修。
- 意见③ 帧压缩·传输层：**已落地并实测**（压缩率 37-44%，keyframe 125KB→50KB）。
- 意见② flash 16MB 池：**已落地（代码级，待真机验证）**。与原方案的偏差：
  - ★ 存储层压缩从「决策点可选」改为「必选落地」：flash 擦除吞吐是物理硬墙
    （4KB sector ~35ms；64KB 块擦优化后整池覆盖上限 ~220KB/s），明文入桶贴线、
    信封直存才有富余；桶内存线上 body 原样（压缩会话=信封），pop() 读出后解压
    回明文——解码/伺服零改动，无损红线不破。容量 ≈320 帧（拐弯段 ×3）。
  - 读路径用 esp_partition_read（非 mmap）：S3 flash data 映射窗口有限（PSRAM 同用
    MMU），16MB 全量 mmap 不可靠；压缩帧 ~50KB 读 + inflate ≈10ms，预算内。
  - 开桶先整桶擦（esp_partition_erase_range 256KB，驱动自动走 64KB 块擦 ~1s，
    锁外做，期间 TCP 背压吸收）；写 [len][body] 单次 esp_partition_write。
  - 分区表：删废 tiles，castpool data/spiffs 16M @0x810000（flash 实际 32MB）。
- 待办：真机验证 flash 池（分区表变更 ⇒ 需 erase_flash 全量重烧 + 烧分区表）；
  App「存储层压缩」会使 recv 5s 探针语义变化（信封直存，无解压账）。
