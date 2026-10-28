# WiFi 投屏 v2：预渲染帧池 + BLE 位置伺服方案

> 2026-10-28 定稿，**四审后冻结（v1.0）**——后续修订仅随 Spike 0 实测数据发生。
> 背景：iperf 实测 16.3Mbps 但空口波动（RSSI -62 / 信道漂移）可把管道打到
> 0.2Mbps，实时推流架构下波动直击播放画面。本方案把「数据面（WiFi 大流量）」与
> 「控制面（BLE 小指令）」分离，用帧池吸收波动。

---

## 1. 架构总览

```
App（单一事实源）
 ├─ 预渲染器：沿路线按【里程】采样渲染+编码 SCF1 帧（跑 ahead of 播放头）
 ├─ WiFi 推流：把帧推入设备池（现有链路/帧格式不动，池满即停）
 └─ BLE 对拍器：10Hz 下发「目标里程」 ← 设备回显「播放头里程」
                ↓
设备 02_lvgl_demo_v9
 ├─ 收流：现有 TCP + 槽位环（满则阻塞 = 背压，机制保留）
 ├─ 帧池：encoded SCF1 帧按【里程编址】存放（v1=PSRAM 4MB / v2=flash 16MB）
 ├─ 伺服播放：朝目标里程追帧，≤15fps 上限，到站保持，BLE 静默冻结
 └─ 解码直写路径不变（bypass LVGL，dummy_draw）
```

核心性质：**WiFi 波动只影响池子填充速度，不影响当前画面**。波动 = 池子净漏，
池深就是抗波动时间；BLE 控制面受 WiFi 挤压但可容忍（coex 分析与压测验收见 §4.1）。

---

## 2. 现状盘点（2026-10-28 代码事实）

| 项 | 现状 | 出处 |
|---|---|---|
| 收流→播放 | 已有定长槽 SPSC 环 12槽×256KB=3MB，满则阻塞收流（背压） | `wifi_cast.cpp:59-60` |
| 播放路径 | cast 模式 pop 即解即显；SCF1 27B 帧头含 seq@17 | `nav_playback.cpp:268+` |
| 解码开销 | decode 24ms（shift 帧含整屏 memmove 868KB）+ 分带 blit | `nav_playback.cpp:13-16` |
| PSRAM | 总 8MB；队列分配后实测余 3367KB，稳态 ~3.3MB | debug.log 实测 |
| flash 分区 | 32MB：nvs/phy 28KB + factory 8M + tiles 14M ≈ 22.1M 已用 | `partitions.csv` |
| 实际占用 | app bin 仅 1.2MB、切片包 tiles_pack.gts 仅 2.4MB → **大量虚位** | build 实测 |
| demo 固件 BLE | **没有**（main/ 无任何 bt 代码），需新建 | main/ 目录 |
| 生产 App BLE | 现成：服务 `4fafc201-…` 写 `beb5483e-…` 通知 `e1f2a3b4-…` | App ble 模块 |

### 2.1 池存储粒度（评审硬伤①，修正）

槽位环语义从**一槽一帧**改为**一桶多帧**。现设计 256KB 定长槽只放一帧（为容纳
keyframe），delta 帧均 ~35KB 时利用率仅 14%：16 槽 4MB 只能存 16 帧 ≈ **1.07s，
原池深账崩塌**。修正：

- **桶 = 256KB，桶内连续装多帧**（帧不跨桶；keyframe 装不下则换新桶，桶尾浪费 ~7%）
- 4MB = 16 桶 ≈ 110+ 帧 ≈ **7.3s @15fps**（帧均 35KB）；第二步 flash 池同理按桶
- pop：桶内游标逐帧出队，桶空再出桶——保住「定长结构」防越界 bug 类
  （2026-10-27 变长记录环已因此废弃，教训必须遵守）
- keyframe 实测尺寸进验收：>256KB 会卡桶（需编码端压 keyframe 质量或特批大桶）

**并发模型（评审复审新①）**：桶内游标**不共享**。收流侧独占「填桶阶段」
（帧逐个写入当前桶，桶满或 keyframe 装不下 → 整桶发布 = 推进 q_tail 一次）；播放侧
独占「消费阶段」（本地游标逐帧读，桶空才推进 q_head 一次）。桶内容读写两个阶段
永不同时发生，共享状态只有 q_head/q_tail 两个槽级游标——**无需 CAS/锁**；但
（三审修正）**跨核时内存序必须显式**：收流/播放任务若绑不同核，q_tail 发布 = release
写、q_head 读取 = acquire 读，否则桶内容可能对播放侧不可见。核绑定 Spike 0 期间定死：
**同核**——沿用原约定（同核任务切换经 FreeRTOS 调度，切换点自带屏障）；
**跨核**——跨核是并行执行不是上下文切换，FreeRTOS 的切换屏障管不到对方核的
load/store 重排，**必须显式 release/acquire，与 FreeRTOS 无关**（四审修正）。

**背压粒度说明（三审③）**：桶级背压偏保守——播放读到半桶时 q_head 不动，设备背压
按「满」阻塞，最大浪费 <1 桶（256KB ≈ 7 帧）。App 侧「池满停推」按帧级里程
（门限 90%）判断，粒度差远小于门限余量，可忽略；**调试时勿把「App 推流卡」误读为
「池满」**。

---

## 3. 帧编址：按里程，不按时间

预渲染帧沿路线**等里程间隔**采样。设备端无需每帧携带里程：

```
帧 i 的里程 = segM0 + i × segStep        （segM0/segStep 在会话头声明）
目标帧号 = (目标里程 - segM0) / segStep   （设备端一步算出）
```

- **采样步长**：15fps 档 ≈ 1.1m/帧（60km/h）；30fps 档 ≈ 0.55m/帧。只是会话头里的一个数。
- **车停 = 目标里程不变 = 画面保持**，无时钟漂移问题（这是选里程制不选时间制的根因）。
- **keyframe 间隔 K=32 帧**：段重置/追赶跳帧只能落在 keyframe 上（delta 链依赖前一帧解码环）。
- **TurnZoom 兼容**：zoom_smooth 由「距拐弯距离」驱动，里程采样天然提供，无需改引擎。

### 3.1 GPS 噪声 → 目标里程滤波（App 侧，四审②修正：真滤波，不用 keyframe 跳吸收噪声）

GPS 精度 5-10m ≫ segStep 1.1m。**KF_JUMP 的语义是「追赶跳帧」，不是「噪声抑制」**
——上一版让设备端用 keyframe 跳吸收 GPS 抖动，等于把 0.5~8m 区间的噪声放大成
keyframe 闪跳（每隔几百 ms 闪一次，比不滤波还差），滤波责任放错了层。修正：
**滤波完全在 App 侧做足**：

- **中值-5**（抑飞点）→ **一阶低通 τ≈0.5s**（压抖动）→ **滞回死区 1m**（滤后变化
  <1m 不发）
- 目标效果：下发增量以**真实位移**为主（60km/h @10Hz ≈1.7m/发 < KF_JUMP=4），
  设备端逐帧追即可
- 单次跳变 >8m（滤后仍跳）标记异常（重路由前兆，评估走段重置）
- 设备端**不加**目标滤波——「App 单一事实源」原则，滤波责任不进设备
- KF_JUMP=4 只服务真实大缺口（起播追赶/断流恢复），不参与噪声抑制

验收联动：GPS 噪声注入 ±8m → **滤后目标帧抖 ≤1 帧**（原「无 ±5 帧抖动」细化）。

### 会话头扩展（评审⑤修正：必须带长度前缀）

TCP 流无消息边界，v1 的 4B `{w,h}` 无长度字段——纯「字节追加」设备端无法判断后面
还有没有字段（评审⑤成立）。改为 **带长度前缀的 v2 头**：

```
Session v2: [magic u16=0x5743 'WC'][ver u8=2][len u8=20]
            [u16 w][u16 h][u8 mode][u16 flags]
            [u32 segId][u32 segM0 米][u16 segStep 厘米/帧][u32 frameCount][u16 kfInterval]
```

- 设备先读 4B（magic+ver+len）：magic 匹配 → 按 len 读全头；不匹配 → 把 4B 当
  v1 `{w,h}` 走实时模式（旧 App 首两字节 = w=466 LE 即 D2 01，与 magic 无碰撞）
- 帧流格式 `[u32 LE size][SCF1]` 完全不动。

---

## 4. BLE 对拍器协议（字节级）

复用生产 GATT 三件套（demo 固件新建同 UUID 服务，App 零适配成本）。
现有 E1/E2 通知保留，新增：

**下行（App→设备，写特征 `beb5483e-…`，10Hz 或里程变化 ≥0.5m 时发）**

```
[0xD0][segId u32 LE][targetMeter u32 LE][flags u8]    共 10B
flags: bit0=段重置(见 §7)  bit1=暂停  bit2=恢复
```

**上行（设备→App，notify 特征 `e1f2a3b4-…`，10Hz）**

```
[0xE3][segId u32 LE][playheadMeter u32 LE][state u8]   共 10B
state: 0=追帧中 1=到站保持 2=池空冻结 3=BLE静默冻结 4=段重置中
```

### 4.1 WiFi/BLE 共存（评审②，第一步必测）

ESP32-S3 上 WiFi/BLE 共用 2.4GHz 射频、时间分片共存——16Mbps 级 burst 会推迟 BLE
连接事件，「BLE 天然免疫」的表述**不成立**，修正为「**BLE 受挤但可容忍**」：稳态
池满时 WiFi 本来就静默，只有回填期两者并存，控制面 10Hz/10B 可扛秒级抖动。硬性动作：

- sdkconfig：`CONFIG_ESP_COEX_ENABLED` + `esp_coex_preference(ESP_COEX_PREFER_BLE)`；
  connection interval ≥30ms 给 WiFi 让路
- **共存压力测试分两场景**（评审复审新②采纳）：
  a) 稳态并发：满速回填 + 对拍并行，测 0xE3 丢率/延迟，验收线：回显连续丢失 <1s
  b) **回填期专项**（池空→满的恢复段，控制面最关键时期）：验收线
     **池深恢复斜率 ≥30 帧/s**（≈2× 消耗），量化 pacing 的恢复速度代价
  任一不达标 → App 回填限速（pacing）兜底

App 用 `写头里程 − 播放头里程` 自算池占用，**设备不需要上报水位**——「池满停推」
由 App 判断，WiFi 协议侧零改动（这个闭环是对拍器必须双向的原因）。

---

## 5. 伺服播放算法（设备端）

```
每 tick（66ms = 15fps）:
  if BLE 静默 > 2s:                  state=3, 冻结（优先级最高，见 §5.1）
  // ★ 先判段再算目标（四审修正）：段重置会换 segM0/segStep，
  //   用旧 segM0 算出的 targetIdx 是脏值，会误触 BT_BACK 倒退告警
  if segId != 当前段:                 state=4, 清池, 等 keyframe（超时 5s → state=2）; 本tick结束
  targetIdx = (lastTarget - segM0) / segStep
  if targetIdx <= playIdx - BT_BACK(=8):  state=1 + 倒退告警（不倒放，App 责任走段重置）
  elif targetIdx <= playIdx:          state=1, 到站保持（不重绘）
  else:
    缺口 = targetIdx - playIdx
    if 缺口 > KF_JUMP(=4): 直接跳最近 keyframe（只解 1 帧），playIdx = keyframe 号
    else:
      // tick 预算守卫（三审④修正）：按【剩余预算 ≥ 预估解码时间】逐帧决策，
      // 预估 = delta 24ms / keyframe 用实测值；keyframe 单帧可穿透「已耗<40ms」
      // 的朴素检查，必须看剩余额度
      while 缺口 > 0 且 (66ms - 本tick已耗) ≥ 预估解码时间(下一帧): 解码下一帧（不 blit 中间帧）
    blit 最新一帧 → playIdx += 本次实追帧数
  if targetIdx 不可达（池内无后续帧）:  state=2, 冻结等补给
```

- **KF_JUMP=4**（三审⑤采纳，原 8 偏大）：缺口 5 帧逐帧追要 ~0.53s 画面冻结后突跳，
  是卡顿不是追赶；KF_JUMP=4 冻结 ≤0.27s，观感可接受。缺口 >4 帧 ≈ 4.4s 车程走
  keyframe 跳，不解中间 delta。
- 追赶按 **剩余预算守卫** 逐帧决策（正常 delta 2×24ms+blit 8ms=56ms ≤ 66ms，留
  10ms 余量；keyframe/复杂 delta 超预算由守卫兜住顺延，永不超时掉拍）。
- **现有实时投屏模式保留为回退**（Session v1 路径），模式开关在 App 侧选择。

### 5.1 权威状态迁移表（设备端唯一事实源）

| 当前态 | 事件 | 动作 | 新态 |
|---|---|---|---|
| SILENT(3) | 收到 0xD0 目标 | 取目标查池：可满足→追帧；不可→等补给 | PLAY(0) / STALL(2) |
| SILENT(3) | 持续静默 | 保持最后帧上屏 | SILENT(3) |
| STALL(2) | 新帧入池且可追目标 | 恢复追帧 | PLAY(0) |
| STALL(2) | **持续无补给 >30s** | 0xE3 持续上报 state=2 + 记日志（等 App 处置） | STALL(2) |
| STALL(2) | BLE 转静默 | 冻结（state 编号让位 3） | SILENT(3) |
| PLAY(0) | 目标 ≤ 播放头 | 到站保持，不重绘 | HOLD(1) |
| PLAY(0) | 目标回退 1~8 帧 | 转 HOLD 保持（**不倒放**） | HOLD(1) |
| HOLD(1) | 目标前移 | 继续追帧 | PLAY(0) |
| HOLD(1) | 目标回退 ≤8 帧 | 保持（**不倒放**） | HOLD(1) |
| HOLD(1) | 目标回退 >8 帧 | 倒退告警 + 保持；App 应走段重置 | HOLD(1)+告警 |
| PLAY/HOLD | BLE 静默 >2s | 冻结 | SILENT(3) |
| PLAY/HOLD | 段重置（flags bit0） | 清池、废弃 delta 链、**清 paused 标志**、等 keyframe | RESET(4) |
| RESET(4) | keyframe 入池 | 解码起播 | PLAY(0) |
| RESET(4) | **5s 无 keyframe** | 冻结 + 0xE3 上报，App 重发段头 | STALL(2) |
| RESET(4) | **又收到段重置 bit0**（连续偏航） | 以新 segId 清池重来，超时计时器重置 | RESET(4) |
| 任意 | 暂停（flags bit1） | 保持当前帧，池照常回填 | HOLD(1)（记 paused） |
| paused | 恢复（flags bit2） | 按普通目标追帧（大跳走 KF 跳） | PLAY(0)/STALL(2) |

**兜底规则（三审①采纳，表成为"唯一事实源"的前提）**：未列出的 事件×状态 组合 =
**保持当前态 + ESP_LOG 记一条**；实现时遇到表外事件，先补表再补码。

**三条裁决规则**：
1. **SILENT(3) 与 STALL(2) 并存时静默优先**——没有目标里程无从谈池；恢复顺序 =
   先收目标、再查池（表中两行衔接）。
2. **不支持倒放是显式约束**（delta 链不可逆，倒放=灾难）：目标回退一律保持 + 告警，
   倒拖/回退路由由 App 走段重置；已列入 §9 明确不做。
3. **paused 是 HOLD 的子状态**，段重置将其清除；RESET 超时落在 STALL 时 paused
   已清，恢复须 App 重发 bit2。

---

## 6. 三步实施

### 第一步：PSRAM 池 4MB + BLE 伺服（15fps）

**Spike 0（先行，结论决定 BLE 走不走，评审④采纳：不放进实施清单排后面）**
sdkconfig 开 BT/NimBLE（最小配置：单连接、2 ACL buffer、关 host 日志、coex 开），
只起 GATT 空转，实测内部 RAM 账与显示 DMA 池影响。**通过线：内部余量 ≥20KB 且刷屏无 NO_MEM。**
不达标 → 对拍器退化到 **WiFi 微端口**（独立 TCP :7124，10Hz
10B 心跳+播放头，接受与推流共管道的抖动，coex 问题随之消失），池/伺服设计全部
不变，仅 §4 协议载体从 BLE 换 TCP。**改道微端口时补测「断流恢复 + 微端口回显
延迟」场景**（共管道被推流挤时回显是否 ≤1s，评审复审采纳），不只测稳态。

> **2026-10-28 执行改道**：不新写 NimBLE，**直接移植生产 motoguide1.43 的 Bluedroid
> 外设**（`components/ble_peripheral_bsp` → `main/ble_conductor.cpp`）——App 已验证
> 能连生产板，广播名/UUID 三件套/MTU 517/连接参数逐字节照抄，App 零改动。
> 裁剪：OTA GATTS 转发、boot_sequencer、设备名 NVS 编辑（固定 "Moto Guide"）。
> 新增：init 各阶段内存账 `[heap]` 打印 + 0xD0 最小解析。生产板已实证 WiFi+BLE
> 共存（OTA over WiFi + BLE 并行），coex 用默认策略 + `BLUEDROID_ESP_COEX_VSC`
> 即可，无需 `esp_coex_preference`。★ 移植要点：`esp_bt_controller_init()` 短暂
> 禁 PSRAM 缓存（生产 main.c:1164 教训）→ 初始化必须在 app_main 最开头。

**设备端（02_lvgl_demo_v9，Spike 0 通过后）**
1. `wifi_cast.cpp`：队列 12→16 桶（4MB，桶=多帧见 §2.1）；Session 头换 v2
   （magic+len 前缀）+ mode 分流；新增「清池」接口（generation 计数，收流/播放两侧对齐）。
2. `nav_playback.cpp`：cast 任务改为伺服循环（§5 算法）；pop 超时语义从「状态页」
   扩展为「保持/冻结」两态。
3. 新建 `ble_conductor.cpp/h`：NimBLE GATT（生产 UUID），0xD0 解析 + 0xE3 回显 10Hz；
   CMakeLists 加 `bt` 依赖。（Spike 0 不通过则本项与第 4 项整体替换为 WiFi 微端口）
4. sdkconfig：`CONFIG_BT_ENABLED` + NimBLE + coex（§4.1），buffer 按 Spike 0 实测定。

**App 端（Android 测试工程 gtam-render + WifiCastTestPanel）**
1. 预渲染推流器：沿路线 polyline 从 `writeMeter` 采样 → 走现有渲染+编码管线 →
   单写推送；`writeMeter - playheadMeter ≥ POOL_LIMIT(4MB×0.9)` 时停推，
   差值回落后续推。
2. BLE 对拍器：接生产 ble 模块，10Hz 发 0xD0、收 0xE3 算池位。
3. 会话头 v2 组包 + 模式选择（实时/池伺服）。

**验收**
- [ ] PC 桩端到端：预渲染 200 帧 → 池 → 目标里程扫描 → 播放帧号与公式一致（±0 帧）
- [ ] 断流：推满池后断 WiFi → 播放继续 7-10s → 冻结 → 恢复续推无跳变
- [ ] BLE 静默 2s 冻结 / 恢复续播（含 §5.1 两裁决规则场景：静默+池空并存、回退>8 帧）
- [ ] 共存压力 a：满速回填中对拍回显连续丢失 <1s（§4.1）
- [ ] 共存压力 b：回填期池深恢复斜率 ≥30 帧/s（§4.1）
- [ ] 首帧上屏 ≤2s（建池 ~30 帧 ≈1MB @16Mbps + 手机渲染/编码耗时，评审补充采纳）
- [ ] 长跑 30min：池环形回绕、无泄漏、播放头单调
- [ ] GPS 噪声注入（±8m 抖动）：播放无 ±5 帧级抖动（§3.1 滤波生效）
- [ ] 现有实时模式回归全绿

### 第二步：flash 16MB 池（20-30s 池深）

1. **重切分区**（`partitions.csv`）：factory 8M→4M（app 实测 1.2MB）、tiles 14M→6M
   （包实测 2.4MB，留一倍增长）→ 腾出 **16MB** `pool` 分区（`data, 0x40`）。
   ⚠️ 切片包 mmap 偏移跟随分区表，改表需重烧全量；本项目 factory 单槽无 OTA，
   无双 slot 冲突——**量产前分区表必须冻结、pool 一次划足**（评审补充成立，
   已出货设备走返厂重烧）。**partitions.csv 里给 pool 行加注释**：子类型 0x40
   勿改、起始偏移 4KB 扇区对齐（建议 64KB 块对齐以优化擦除，评审复审采纳）。
2. **池驱动**：PSRAM 池作为一级缓存，flash 作二级——写入路径 = PSRAM 攒批（≥64KB 或
   100ms）→ `esp_partition_write` 后台任务批量刷；播放只读 PSRAM + flash mmap。
3. **抖动治理**：擦除（64KB 块 30-200ms）全部在后台任务；写 flash 期间 cache 停摆
   （XIP 停顿）按毫秒级预算排——限定在播放 tick 空隙执行。**先 spike 实测**：
   64KB 批「擦+写」在 66ms tick 空隙是否容得下、decode/blit 热路径 IRAM 占比；
   容不下 → 批改小（32KB）或接受写入期偶发掉 1 帧。
4. **寿命账**（要认）：15fps×35KB ≈ 2GB/h ÷ 16MB 环 ≈ 125 遍擦写/h；10 万次寿命
   ≈ **800 小时导航**，全 32MB 磨损均衡可翻倍。日均 1h 约 2-4 年，够用但记入台账。
5. App：`POOL_LIMIT` 改 16MB×0.9，池深 20-30s；其余逻辑零改动。

### 第三步：30fps

- 会话头 `segStep` 1.1m→0.55m，帧率上限 66ms→33ms tick。
- 前提复核：① 供给 16Mbps vs 需求 0.8-1.2MB/s ✓ ② 解码预算 33ms：decode 24ms 偏紧，
  若超支走「shift 帧路径优化」或 24→20ms 的 memmove 优化 ③ 池深减半后波动容差回
  10-15s，由 flash 池兜底（20-30s 按 30fps 帧均算）。
- 若解码超支且不想动光栅器：退而求其次 20-24fps（步长 0.7-0.8m），一行参数。

---

## 7. 偏航 / 重路由语义（段号制）

- 帧编址 = `segId + 里程`，池内帧带段号（槽位扩展 4B 或随 generation 计数）。
- App 检测偏航 → 重算路线 → 新 segId 从当前位置起渲染 → BLE 发
  `[0xD0 segId=M2 m0=当前里程 flags=bit0]` → 设备清池、写指针跳到播放头前方。
- 冻结窗口 = 重算 + 渲染首段推送时间（目标 <2s），画面保持最后帧。
- 暂停（flags bit1）：播放头保持、池照常回填；恢复（bit2）：从保持点继续追目标；
  恢复时目标大跳 >KF_JUMP 同伺服 keyframe 跳规则（评审补充，补语义）。
- **第一步只定接口和段号字段，重路由 UI/检测不在本期**（用户拍板：暂不考虑偏航）。

---

## 8. 风险清单

| 风险 | 等级 | 对策 |
|---|---|---|
| NimBLE 挤爆内部 RAM（当前仅余 27KB/最大连续 8KB） | 高 | **Spike 0 先行**（§6），不通过即改道 WiFi 微端口对拍器，池/伺服设计零改动 |
| WiFi/BLE coex 互挤（回填期，评审②成立） | 中 | coex 偏置 BLE + connInterval ≥30ms + 回填限速兜底（§4.1）；压力测试定案 |
| 预渲染帧与实际 GPS 吸附点位漂移（采样 vs 真实轨迹） | 中 | 伺服按目标里程取帧，天然重对齐；残差=半步长（≤0.55m）不可见 |
| keyframe 间隔 vs 跳转粒度（K=32@1.1m≈35m，跳变观感） | 中 | K=32 起步，按实测观感在 16~64 间调（评审小问题采纳） |
| flash 写入挤占播放 tick（第二步） | 中 | 后台攒批 + tick 空隙执行 + 实测分账 |
| 池满判断依赖 BLE 回显质量 | 低 | 回显丢失时 App 按最坏值保守停推 |

## 9. 明确不做（本期）

- **不支持倒放/倒拖**（delta 链不可逆，评审复审点名）：目标里程回退一律「保持+告警」，
  回退路由/倒拖场景由 App 走段重置
- 偏航检测 UI / 自动重算触发（只留段号接口）
- flash 磨损均衡自研（直接用 esp_partition 顺序环 + 分区级均衡即可）
- 帧内 HUD 增量编码（HUD 照旧随帧全量下发）
