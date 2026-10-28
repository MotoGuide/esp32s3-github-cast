# gtam-cast — ESP32-S3 离线地图导航投屏（预渲染帧流 + BLE 伺服对拍）

把 Android 手机上的离线矢量地图**逐帧预渲染**成压缩帧流，经 WiFi(TCP) 推给
ESP32-S3 圆屏设备播放；播放进度用 BLE 以**帧号对拍**闭环伺服。手机只管出帧，
设备只管解码播放——本仓库是这套链路的完整参考实现（设备端收流/池/伺服 +
Android 测试端推流面板 + 协议文档），供研究复用。

> 手机端 ~110fps 预渲染、设备端 30fps 播放；测试实测全链路闭环
> （0xD0 目标 10Hz、伺服追踪、TCP 背压呼吸正常）。

## 核心思想

App 沿路线把**所有帧提前渲染好**，通过 WiFi 传给 ESP32，放进一个 16MB 的
**滚动帧池**；导航过程中，App 像控制伺服电机一样给设备发**对拍指令**——
什么时间播放哪一帧。设备端不做任何渲染，只负责解码播放。

因此设备的显示效果**理论上等价于预渲染帧的帧率**：设备屏幕的流畅度上限
由 App 的渲染质量决定，与设备自身算力无关——这就是「App 算、设备播」
的分工：再弱的 MCU 也能呈现手机级的地图动画效果。

## 目录结构

```
├── device/                      ESP32-S3 接收端（ESP-IDF 组件源码，供移植参考）
│   ├── wifi_cast.h/.cpp         WiFi 收流 + 桶式帧池 + 压缩信封解压（核心）
│   ├── ble_conductor.h/.cpp     BLE GATT：0xD0 对拍指令收 / 0xE3 播放头回显
│   ├── nav_playback.cpp         播放任务：池消费 → 解码 → 伺服状态机（STALL/追帧/静默冻结）
│   └── partitions.csv           32MB flash 分区表（castpool 16M 数据分区）
├── app-test/                    Android 测试端（仅 debug 源集测试代码，非生产代码）
│   ├── WifiCastTestPanel.kt     全链路测试面板：预渲染推流/限速/0xD0 心跳/0xE3 解析/阶段计时
│   └── cast-encoder/            SCF1 帧编码器（JNI C++，依赖 gtam 地图引擎，未随包发布）
│       ├── gtam_cast.h
│       └── gtam_cast.cpp        渲染 → SCF1 增量编码 → zlib deflate 信封
└── docs/
    ├── cast_pool_servo_plan.md      池 + 伺服方案（三步走）
    ├── cast_v2_round2_plan.md       二轮改造：帧号对拍 / flash 16M 池 / 帧压缩（含实测状态）
    └── format_cast_v1.md            SCF1 帧格式（27B 帧头 + varint run 流）
```

## 链路架构

```
┌─ Android（测试面板）──────────────┐      ┌─ ESP32-S3 ─────────────────────────┐
│ 地图引擎逐帧预渲染（渲染 7ms/帧）  │      │ TCP 收流 → 压缩信封校验             │
│ gtam_cast: SCF1 增量编码          │ TCP  │   → esp_partition_write 入桶       │
│   → zlib deflate 信封（无损）     │─────▶│   （桶存线上原样，ingest 不解压）    │
│ 300KB/s 灌流限速（保 BLE 时隙）   │      │ pop(): 读 flash → inflate → 明文    │
│                                  │      │   → 解码 → LVGL 上屏               │
│ BLE 0xD0 心跳 10Hz（无条件发）    │ BLE  │ 伺服状态机：目标=frame_idx          │
│   目标 = min(虚拟车, 播放头+11帧) │◀────│   PLAY/HOLD/STALL/SILENT/RESET     │
│                                  │ 0xE3 │   （追帧=keyframe 跳转，非逐帧快进）│
└──────────────────────────────────┘      └────────────────────────────────────┘
```

## 关键设计（含踩坑结论）

1. **两条链路严格解耦**：BLE 伺服链只回答「设备该播到哪一帧」（beat 10Hz
   **无条件发**，目标不前进就重发同值，兼任心跳）；WiFi 填池链只受池满背压
   自然节流，不掺任何人为判断。任何跨链路判定（如看门狗把播放头混进推流
   故障检测）都会造出新的互等死锁——这是用三次死锁换来的原则。
2. **帧号对拍，不用里程**：里程会跳变，0xD0/0xE3 全部改载段内 frame_idx，
   整数 1:1，伺服 `targetIdx` 直读。
3. **桶式帧池（flash 后端）**：64 桶 × 256KB = 16MB，桶内连续多帧、帧不跨桶；
   开桶先整桶擦（驱动自动 64KB 块擦），发布 = 写 bucketLen + 锁内推尾指针。
   桶里存**线上原样**（压缩会话 = deflate 信封直存），pop 读出后再解压——
   擦除吞吐是硬墙（~220KB/s），ingest 不解压才有灌入富余，容量也 ×3。
4. **传输层压缩无损**：v2 会话头 flags bit0 = 帧压缩开关；信封
   `[u32 clen][u32 rawLen][zlib(SCF1)]`，实测压缩率 37~44%
   （keyframe 125KB→50KB），zlib 保证逐位还原，渲染零损失。
5. **SILENT 保护**：BLE 静默 >6s 冻结伺服（阈值要远大于心跳间隔；
   TCP 灌流洪峰会挤掉 BLE 时隙——WiFi/BT 同用 2.4G，限速 300KB/s 后消失）。

## 已知坑（给后来者）

- Kotlin 裸块 `{ send() }` 是创建 lambda 后丢弃——发送被包进独立花括号会
  **静默不执行**，编译器只有 unused warning。
- BLE 128-bit UUID 在部分机型（华为兼容模式）按整串字节反序出现，
  匹配要兼容两种形态；NO_RESPONSE write 不产生 Nordic 日志。
- 压缩信封后 `enc[2]` 是 clen 字节不是 SCF1 flags，keyframe 计数会恒 0。
- 分区表变更 ⇒ 必须 `idf.py erase-flash` 全量重烧。

## 未随包发布的依赖

- gtam 离线矢量地图引擎（渲染 + SCF1 编码的数据源）：`gtam_cast.cpp` 的
  `#include "mapcore/..."` / `"icon/..."` 指向该引擎，此处仅作接口参考。
- Android 生产 App（BLE 客户端封装 `BleClientProvider` 等）：测试面板引用的
  生产类需自行替换/桩掉。
- LVGL demo 外壳（`nav_playback.cpp` 的出图依赖 `bsp_display` 等）。

## 硬件与构建

- 设备：ESP32-S3 + Octal PSRAM + 32MB flash + 1.75" 圆形 AMOLED（466×466），
  ESP-IDF v5.x；组件依赖 `esp_partition` / `espressif__zlib` / `lwip` / `bt`。
- 本仓库是**参考实现**而非可一键编译的工程（设备文件原属 LVGL demo 工程，
  Android 面板原属 debug 源集），按 `docs/` 的协议描述拼装到自己的工程即可。

## License

[GPL-3.0](LICENSE)（GNU General Public License v3.0）。
