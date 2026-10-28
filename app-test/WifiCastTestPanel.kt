package com.solo.guide.mode.test

import android.content.Context
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.net.wifi.WifiManager
import android.os.SystemClock
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import com.motonav.ble.BleClientProvider
import com.motonav.ble.BleDeviceManager
import com.motonav.core.model.LatLng
import com.motonav.gtamrender.GtamRenderJni
import com.solo.guide.gps.SimGpsProvider
import com.solo.guide.utils.AppLog
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import java.net.ConnectException
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.Executors

/**
 * 测试模式·WiFi 投屏面板（**debug 真身**；release 为空桩——源码级隔离惯例）。
 *
 * 两个动作：
 *  ① 连通性测试：连设备 TCP 发 16B 会话头 → 设备屏 WAIT→LINK OK→回 WAIT（肉眼判通）
 *  ② **WiFi 推流**：全链真跑——gtam 会话（引擎→渲染器）→ SCF1 编码（keyframe/delta
 *     自适应）→ TCP `[u32 长度][帧]` 推给设备全屏播放。可与"模拟推屏"并行（对拍）。
 *
 * ⚠️ 前置：手机开热点（2.4G）——设备以 STA 连热点，App 连设备 :7123。
 *    两种形态都支持：App 手机连别人热点（绑定 WIFI 网络）或 **App 手机自己就是
 *    热点主机**（Android 自己的 AP 不算 TRANSPORT_WIFI → 用不绑定的裸 socket）。
 *    设备 IP 自动发现：从网关/本机 AP 接口推网段前缀 → /24 并行探测 :7123。
 * ⚠️ 铁律（CLAUDE.md/gtamap-cast-plan）：per-socket 绑定，**严禁 bindProcessToNetwork**
 *    ——热点无互联网，绑进程会把整个 App 断网。
 * ⚠️ 线协议（与设备端 wifi_cast.cpp 对齐，全部小端）：
 *    会话头 16B = "GWC1" + u16 w + u16 h + u16 fps + 6B rsv；
 *    每帧 = u32 长度 + SCF1 帧字节。
 */
@Composable
fun WifiCastTestPanel(routePolyline: List<LatLng>, speedKmh: Float) {
    val context = LocalContext.current
    var status by remember {
        mutableStateOf("先开手机热点（2.4G）等设备连上，设备屏会显示它的 IP")
    }
    var testing by remember { mutableStateOf(false) }
    var bleTesting by remember { mutableStateOf(false) }
    var iperfing by remember { mutableStateOf(false) }
    var streaming by remember { mutableStateOf(false) }
    var poolStreaming by remember { mutableStateOf(false) }
    // sim 引用提出来：推流中调速滑条生效
    var simRef by remember { mutableStateOf<SimGpsProvider?>(null) }
    val scope = rememberCoroutineScope()
    // 虚拟车速（池模式）：滑条实时改，推流循环读
    var poolSpeed by remember { mutableStateOf(speedKmh) }

    Button(
        onClick = {
            if (streaming) return@Button
            streaming = !streaming
        },
        enabled = routePolyline.size >= 2 && !streaming && !poolStreaming,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Text("开始WiFi推流（引擎→SCF1→设备全屏）")
    }
    if (streaming) {
        OutlinedButton(
            onClick = { streaming = false },
            modifier = Modifier.fillMaxWidth(),
        ) { Text("停止推流") }
    }
    Spacer(Modifier.height(4.dp))
    // ── v2 预渲染推流（池+BLE 对拍）：与 v1 实时推流互斥，同一台设备一次只跑一种 ──
    Button(
        onClick = { if (!streaming && !testing && !iperfing) poolStreaming = !poolStreaming },
        enabled = routePolyline.size >= 2 && !poolStreaming,
        modifier = Modifier.fillMaxWidth(),
    ) { Text("预渲染推流（帧池+BLE对拍 15fps）") }
    if (poolStreaming) {
        OutlinedButton(
            onClick = { poolStreaming = false },
            modifier = Modifier.fillMaxWidth(),
        ) { Text("停止预渲染推流") }
    }
    Spacer(Modifier.height(4.dp))
    // ── BLE 连通性测试：纯 BLE 链路诊断（断生产→连设备→报服务表→0xD0/0xE3）──
    OutlinedButton(
        onClick = {
            if (bleTesting) return@OutlinedButton
            bleTesting = true
            scope.launch {
                status = runBleConnectivityTest(context)
                bleTesting = false
            }
        },
        enabled = !bleTesting && !streaming && !poolStreaming && !testing && !iperfing,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Text(if (bleTesting) "BLE 测试中…" else "BLE 连通性测试（不碰WiFi，报服务表）")
    }
    Spacer(Modifier.height(4.dp))
    OutlinedButton(
        onClick = {
            if (testing) return@OutlinedButton
            testing = true
            scope.launch {
                status = runWifiCastTest(context)
                testing = false
            }
        },
        enabled = !testing && !streaming && !poolStreaming,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Text(if (testing) "测试中…" else "连通性测试（只发会话头，看设备屏 LINK OK）")
    }
    Spacer(Modifier.height(4.dp))
    OutlinedButton(
        onClick = {
            if (iperfing) return@OutlinedButton
            iperfing = true
            scope.launch {
                status = runIperfTest(context)
                iperfing = false
            }
        },
        enabled = !iperfing && !streaming && !poolStreaming && !testing,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Text(if (iperfing) "灌流中（10s）…" else "iperf 测速（10s 灌流 :5001，量裸管道）")
    }
    Spacer(Modifier.height(4.dp))
    Text(status, style = MaterialTheme.typography.labelSmall)

    // 速度滑条 → 实时推流调 sim 速度 / 池推流调虚拟车速
    LaunchedEffect(speedKmh) {
        if (streaming) simRef?.setSpeed(speedKmh)
        poolSpeed = speedKmh
    }

    // ── 推流主循环 ──
    LaunchedEffect(streaming, routePolyline) {
        if (!streaming || routePolyline.size < 2) return@LaunchedEffect
        // 单线程执行器：gtam 会话 + socket 收发全收一条线程（PolylineEngine 单线程契约）
        val exec = Executors.newSingleThreadExecutor().asCoroutineDispatcher()
        var handle = 0L
        var sim: SimGpsProvider? = null
        var sock: Socket? = null
        // WifiLock：推流期间压住手机 WiFi 省电。STA 省电休眠会让设备端 AP 走
        // "为睡眠终端缓存帧"的路径（也是延迟杀手），流媒体 App 标配。
        // LOW_LATENCY 需 API 29+，低版本退 FULL_HIGH_PERF。
        val wm = context.getSystemService(Context.WIFI_SERVICE) as android.net.wifi.WifiManager
        val wifiLock = wm.createWifiLock(
            if (android.os.Build.VERSION.SDK_INT >= 29) WifiManager.WIFI_MODE_FULL_LOW_LATENCY
            else @Suppress("DEPRECATION") WifiManager.WIFI_MODE_FULL_HIGH_PERF,
            "gtam-cast"
        ).apply { setReferenceCounted(false); acquire() }
        try {
            withContext(exec) {
                // ① WiFi 网络查找（null = App 自开热点的主机形态，走裸 socket）
                val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
                val wifiNet = cm.allNetworks.firstOrNull {
                    cm.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
                }

                // ② TCP 连设备（网段自动发现 + /24 并行探测）+ 会话头
                sock = connectDeviceSocket(cm, wifiNet)
                val hdr = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN)
                    .put("GWC1".toByteArray(Charsets.US_ASCII))
                    .putShort(466).putShort(466).putShort(30)
                    .put(ByteArray(6))
                    .array()
                sock!!.getOutputStream().run { write(hdr); flush() }

                // ③ gtam 会话（引擎+渲染器；地图字节每次读，7MB ~35ms）
                val mapBytes = context.assets.open("jinan_260805.gtam").use { it.readBytes() }
                val pts = DoubleArray(routePolyline.size * 2)
                routePolyline.forEachIndexed { i, p ->
                    pts[2 * i] = p.latitude; pts[2 * i + 1] = p.longitude
                }
                handle = GtamRenderJni.castOpen(mapBytes, pts, 16.5)
                if (handle == 0L) error("gtam 会话创建失败（地图解析/路线过短）")

                // ④ 模拟 GPS 30fps（单源实验口径，与本地预览一致）
                sim = SimGpsProvider(routePolyline, speedKmh, tickIntervalMs = 33L).also {
                    it.start()
                    simRef = it
                }
                launch(exec) {
                    sim?.flow?.collect { st ->
                        if (st.hasFix) {
                            GtamRenderJni.castGps(
                                handle, st.lat, st.lng,
                                st.speedKmh.toDouble(), st.bearing.toDouble(),
                                SystemClock.elapsedRealtime(),
                            )
                        }
                    }
                }

                // ⑤ 推流循环：渲 → SCF1 编码 → [u32 LE 长度][帧] → TCP（阻塞发送=背压）
                //    三段计时（渲/编/发）——1.5fps 的账在谁头上，跑一轮就见分晓
                val out = sock!!.getOutputStream()
                // ★ 整帧单次 write：4B 长度头拼进预分配大缓冲一把写出（10-28 iperf 实测
                //   裸管道 16.3Mbps ≫ 推流实得 2.1Mbps——发送模式尽量贴近 iperf 的
                //   连续大块写入，消除「4B 小包+大包」两段式的窗口/ACK 交互开销）
                val pkt = ByteArray(4 + 256 * 1024)   // 设备 FRAME_MAX = 256KB-4
                var frames = 0
                var bytes = 0L
                val t0 = SystemClock.elapsedRealtime()
                while (isActive) {
                    val now = SystemClock.elapsedRealtime()
                    val tR = SystemClock.elapsedRealtime()
                    val rendered = GtamRenderJni.castRender(handle, now, 466, 466)
                    val renderMs = SystemClock.elapsedRealtime() - tR
                    if (!rendered) {
                        delay(10)   // 首帧定位前
                        continue
                    }
                    val tE = SystemClock.elapsedRealtime()
                    val enc = GtamRenderJni.castEncode(handle)
                    val encMs = SystemClock.elapsedRealtime() - tE
                    if (enc == null) { delay(5); continue }
                    val tS = SystemClock.elapsedRealtime()
                    pkt[0] = enc.size.toByte()
                    pkt[1] = (enc.size shr 8).toByte()
                    pkt[2] = (enc.size shr 16).toByte()
                    pkt[3] = (enc.size shr 24).toByte()
                    enc.copyInto(pkt, 4)
                    out.write(pkt, 0, 4 + enc.size)
                    out.flush()
                    val sendMs = SystemClock.elapsedRealtime() - tS

                    ++frames
                    bytes += enc.size + 4
                    val cost = SystemClock.elapsedRealtime() - now
                    status = "帧 $frames · 渲${renderMs} 编${encMs} 发${sendMs}ms · " +
                            "${bytes / 1024}KB · ${(frames * 1000 / (now - t0 + 1))}fps"
                    // 每 30 帧落一行文件日志（华为 logcat 抑制第三方 Log.i，文件才可见）
                    if (frames % 30 == 0) {
                        AppLog.i("WifiCast",
                            "推流 帧$frames enc=${enc.size}B 渲${renderMs} 编${encMs} 发${sendMs} 帧耗${cost}ms")
                    }
                    // 慢帧异常日志：抓间歇性停顿（>100ms = 偏离 46ms 健康线）
                    if (cost > 100) {
                        AppLog.i("WifiCast",
                            "慢帧 $frames 渲$renderMs 编$encMs 发$sendMs 帧耗${cost}ms enc=${enc.size}B")
                    }
                    val slack = 33 - (SystemClock.elapsedRealtime() - now)
                    if (slack > 0) delay(slack.toLong()) else delay(1)   // ★ delay 挂起防饿死 GPS collect
                }
            }
        } catch (e: Exception) {
            status = "❌ 推流中断: ${e.message}"
        } finally {
            // 结构化并发：到这里子协程（GPS collect）已全部结束 ⇒ 关闭顺序安全
            sim?.stop()
            simRef = null
            if (handle != 0L) GtamRenderJni.castClose(handle)
            runCatching { sock?.close() }
            runCatching { if (wifiLock.isHeld) wifiLock.release() }
            exec.close()
        }
    }

    // ── v2 预渲染推流主循环（帧池 + BLE 对拍；方案 docs/cast_pool_servo_plan.md）──
    // 2026-10-28 改版：BLE 对拍**直接走生产通道**（BleClientProvider.client），
    // 不再自建裸 GATT 客户端——生产 Nordic 在华为机上已验证（服务发现兼容/MTU/CCCD 全套踩平）。
    LaunchedEffect(poolStreaming, routePolyline) {
        if (!poolStreaming || routePolyline.size < 2) return@LaunchedEffect
        val exec = Executors.newSingleThreadExecutor().asCoroutineDispatcher()
        val outerScope = this                      // LaunchedEffect 作用域（0xD0 协程挂这）
        var beatJob: kotlinx.coroutines.Job? = null
        var handle = 0L
        var sock: Socket? = null
        // 推流计数器（catch/finally 也要读 → 提升到外层）
        var writeIdx = 0
        var frames = 0
        var bytes = 0L
        val playheadFrame = java.util.concurrent.atomic.AtomicLong(-1)
        val devState = java.util.concurrent.atomic.AtomicInteger(-1)

        // 看门狗已删（2026-10-28 晚，用户拍板）：write 在池满时被背压阻塞是正常现象，
        // 不是故障——设备消费恢复自然流动。write 真假死由设备端 RCVTIMEO 10s 假死检测兜底。
        // 教训：watch key 里混入 BLE 播放头 = 跨链路耦合，喂入端慢（~1.5fps）必被误杀。
        val wm = context.getSystemService(Context.WIFI_SERVICE) as WifiManager
        val wifiLock = wm.createWifiLock(
            if (android.os.Build.VERSION.SDK_INT >= 29) WifiManager.WIFI_MODE_FULL_LOW_LATENCY
            else @Suppress("DEPRECATION") WifiManager.WIFI_MODE_FULL_HIGH_PERF,
            "gtam-pool"
        ).apply { setReferenceCounted(false); acquire() }
        try {
            withContext(exec) {
                val client = BleClientProvider.client

                // ── ⓪ BLE 对拍走生产通道：要求生产已连接（蓝牙页连好），最多等 10s ──
                if (!client.ready) {
                    status = "等待生产 BLE 通道就绪（请到蓝牙页连接设备）…"
                    val ok = kotlinx.coroutines.withTimeoutOrNull(10_000L) {
                        while (!client.ready) delay(300); true
                    }
                    if (ok != true) error("生产 BLE 未连接——先去蓝牙页连上 \"Moto Guide\"，再点本按钮")
                }
                // 挂 0xE3 原始通知（设备伺服回显；BLE 回调线程写入，Atomic 保证可见性）
                client.rawDevStatusNotify = { raw ->
                    if (raw.size >= 10 && raw[0] == 0xE3.toByte()) {
                        val bb = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN)
                        bb.position(1)
                        bb.int                                // segId 丢弃（测试单段恒 1）
                        playheadFrame.set(bb.int.toLong())    // [5..8] playheadFrame（帧号对拍）
                        devState.set(bb.get().toInt())        // [9] state
                    }
                }

                // ── ② TCP 连设备 + Session v2 头（27B；字节布局对齐 wifi_cast.cpp）──
                val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
                val wifiNet = cm.allNetworks.firstOrNull {
                    cm.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
                }
                sock = connectDeviceSocket(cm, wifiNet)
                val out = sock!!.getOutputStream()

                // ── ③ gtam 会话 + 池模式策略开关（TurnZoom 关 / 直出航向 / kf=32）──
                val mapBytes = context.assets.open("jinan_260805.gtam").use { it.readBytes() }
                val pts = DoubleArray(routePolyline.size * 2)
                routePolyline.forEachIndexed { i, p ->
                    pts[2 * i] = p.latitude; pts[2 * i + 1] = p.longitude
                }
                handle = GtamRenderJni.castOpen(mapBytes, pts, 16.5)
                if (handle == 0L) error("gtam 会话创建失败")
                GtamRenderJni.castSetTurnZoom(handle, false)      // 逐帧 mpp 动画毁 delta
                GtamRenderJni.castSetKeyInterval(handle, 32)      // = kfInterval，伺服跳转入口
                GtamRenderJni.castSetDirectHeading(handle, true, 0.0)  // 5° 量化直出航向
                // ★ 传输层压缩（二轮意见③）：encode 产物变 deflate 信封（无损），
                //   与下面 v2 头 flags bit0 成对——设备按会话头决定解不清帧
                GtamRenderJni.castSetCompress(handle, true)

                val routeLenM = GtamRenderJni.castRouteLength(handle)
                val segStepCm = 111                                // 15fps @ 60km/h ≈ 1.111m/帧
                val totalFrames = (routeLenM * 100 / segStepCm).toInt()
                if (totalFrames < 2) error("路线太短（${routeLenM}m）")

                // 会话头 v2：[magic 0x5743][ver=2][len=23][w u16][h u16][mode=1][flags u16]
                //            [segId u32][segM0 u32][segStepCm u16][frameCount u32][kfInterval u16]
                val hdr = ByteBuffer.allocate(4 + 23).order(ByteOrder.LITTLE_ENDIAN)
                    .put(0x43).put(0x57)                           // magic LE = 0x5743 ('C','W')
                    .put(2).put(23)
                    .putShort(466).putShort(466)
                    .put(1)                                        // mode=1 池伺服
                    .putShort(1)                                   // flags bit0=帧压缩（deflate 信封）
                    .putInt(1)                                     // segId（测试单段恒 1）
                    .putInt(0)                                     // segM0 = 0
                    .putShort(segStepCm.toShort())
                    .putInt(totalFrames)
                    .putShort(32)                                  // kfInterval
                out.write(hdr.array()); out.flush()
                status = "v2 头已发（${totalFrames} 帧 × ${segStepCm}cm）· BLE 生产通道就绪"
                AppLog.i("WifiCast", "v2头已发 route=${routeLenM}m frames=$totalFrames step=${segStepCm}cm")

                // ── ③.8 BLE 对拍独立协程：0xD0 10Hz 跑主线程，与 TCP 填池写完全解耦。
                //    ★ 帧号对拍（2026-10-28 二轮意见①）：0xD0 载荷从「目标里程」改为
                //    「目标段内帧号」——纯整数 1:1，无 segStepCm 除法舍入、无跳帧嫌疑。
                val LEAD_FRAMES = 11L              // 0xD0 目标超前播放头的引导量（原 LEAD_M=12m ≈11 帧）
                //    16:56 死锁根因：0xD0 原在填池外层发，内层 out.write 被池满背压卡住
                //    → 外层回不去 → >2s 无 0xD0 → 设备伺服冻结（BLE 静默保护）→ 池永不排液
                //    → TCP 永远背压 → 互等死锁。对拍节奏独立后，背压只影响填池不影响伺服。
                //    17:25 STALL 根因：目标=虚拟车实时位置，但 WiFi 吞吐(~2fps)喂不饱
                //    15fps 码率 → 池播完时目标远在池内容之外 → 设备 STALL 空等不放帧
                //    → 池不排液 → 又背压。故目标钳制 = min(虚拟车帧, 播放头帧+引导量)，
                //    单调不倒退：设备永远「播播放头前面一点点」，有帧就连续播。
                beatJob = outerScope.launch {
                    var last = SystemClock.elapsedRealtime()
                    var carMeterBle = 0.0     // 虚拟车积分（米，上限）
                    var sentFrame = 0L        // 实际下发目标帧号（单调）
                    while (isActive) {
                        delay(50)
                        val now = SystemClock.elapsedRealtime()
                        carMeterBle = (carMeterBle + poolSpeed / 3.6 * (now - last) / 1000.0)
                                .coerceAtMost(routeLenM)
                        last = now
                        val carFrame = (carMeterBle * 100 / segStepCm).toLong()
                                            .coerceAtMost(totalFrames - 1L)
                        val phF = playheadFrame.get()      // 设备真实播放头（0xE3，帧号）
                        val want = if (phF >= 0) minOf(carFrame, phF + LEAD_FRAMES)
                                   else carFrame
                        if (want > sentFrame) sentFrame = want
                        // ★ 心跳无条件发（2026-10-28 晚死锁定案）：play 冻住 → target=
                        //   play+11 也冻住 → 若只在前进时发，0xD0 停流 → 设备判 BLE 静默
                        //   冻结伺服 → 池满 → q_reserve 阻塞 → TCP 背压 → write 卡死
                        //   → 互等。重发同值目标对设备无害（伺服只吃 frame_idx 数值）。
                        //   ★★ 血泪：曾写成裸 { ... } 块——Kotlin 语句位置的 lambda 字面量
                        //   只创建不调用，send 从未执行，设备 SILENT 卡死两轮（22:25/22:33）。
                        runCatching {
                            val d0 = ByteBuffer.allocate(10).order(ByteOrder.LITTLE_ENDIAN)
                                .put(0xD0.toByte()).putInt(1)      // segId
                                .putInt(sentFrame.toInt())         // frameIdx（段内帧号）
                                .put(0).array()                    // flags
                            BleClientProvider.client.send(d0)
                        }
                    }
                }

                // ── ④ 预渲染+推送循环：虚拟车（里程积分）+ 池水位闭环 ──
                //    池占用 = 写头帧号 − 设备播放头帧号（0xE3 回显），满窗停推；
                //    TCP 背压（设备池满 → 不 recv → 窗口收窄 → send 阻塞）是兜底。
                val pkt = ByteArray(4 + 256 * 1024)
                val poolTargetFrames = 90          // 在途窗（设备池 ~110 帧留余量）
                val frameMs = 66L                  // 合成时钟 15fps：动画时间基准
                writeIdx = 0                       // 段内下一帧号（写头）
                var carMeter = 0.0                 // 虚拟车里程（滑条速度积分）
                frames = 0; bytes = 0L; var keyframes = 0
                // 分阶段计时（2026-10-28 定位 1.5fps 瓶颈）：近 30 帧各阶段均摊 ms
                var msRender = 0L; var msEncode = 0L; var msWrite = 0L; var stageN = 0
                // ★ 灌流平滑限速（2026-10-28 夜）：渲染实测 7ms/帧 ⇒ 0.8s 灌满 2.3MB 池，
                //   TCP 洪峰挤掉 BLE 时隙（WiFi/BT 同 2.4G 共存），0xD0 出 5s+ 大洞 →
                //   设备 SILENT 冻结不消费 → 池满背压死等。限速后 ~8s 匀速填满，
                //   BLE 有呼吸窗；稳态消费 15fps 远低于此限，不伤追帧。
                val RATE_BPS = 300L * 1024
                var rateT0 = 0L
                var lastMs = SystemClock.elapsedRealtime()
                val synBase = lastMs               // 合成时钟基准（引擎只看差值）
                status = "BLE 连接中…（v2 头已发：${totalFrames} 帧 × ${segStepCm}cm）"
                while (isActive) {
                    val now = SystemClock.elapsedRealtime()
                    val dt = (now - lastMs).coerceAtLeast(0)
                    lastMs = now
                    carMeter += poolSpeed / 3.6 * dt / 1000.0
                    if (carMeter > routeLenM) carMeter = routeLenM

                    // 池水位：播放头帧号（0xE3 直读，帧号对拍；未收到前按 0 处理）
                    val ph = playheadFrame.get()
                    val playIdx = if (ph >= 0) ph.toInt() else 0

                    // 填池：渲→编→推（TCP send 阻塞=天然背压）
                    while (isActive && writeIdx < totalFrames &&
                           writeIdx - playIdx < poolTargetFrames) {
                        val meterM = writeIdx * segStepCm / 100.0
                        val pos = GtamRenderJni.castRouteInfo(handle, meterM)
                        if (pos == null) { writeIdx = totalFrames; break }
                        // ★ 航向 5° 量化：相邻帧同格不旋转（delta 保命），跨格只 1 帧旋转
                        val bearingQ = (Math.round(pos[2] / 5.0) * 5.0) % 360.0
                        val syn = synBase + writeIdx * frameMs
                        GtamRenderJni.castGps(handle, pos[0], pos[1], poolSpeed.toDouble(), bearingQ, syn)
                        GtamRenderJni.castUpdateDirectHeading(handle, bearingQ)
                        val tR0 = SystemClock.elapsedRealtime()
                        if (!GtamRenderJni.castRender(handle, syn, 466, 466)) { writeIdx++; continue }
                        val tR1 = SystemClock.elapsedRealtime()
                        val enc = GtamRenderJni.castEncode(handle)
                        val tR2 = SystemClock.elapsedRealtime()
                        if (enc == null) { writeIdx++; continue }
                        pkt[0] = enc.size.toByte()
                        pkt[1] = (enc.size shr 8).toByte()
                        pkt[2] = (enc.size shr 16).toByte()
                        pkt[3] = (enc.size shr 24).toByte()
                        enc.copyInto(pkt, 4)
                        out.write(pkt, 0, 4 + enc.size)
                        out.flush()
                        val tR3 = SystemClock.elapsedRealtime()
                        msRender += tR1 - tR0; msEncode += tR2 - tR1; msWrite += tR3 - tR2
                        ++stageN; ++writeIdx; ++frames; bytes += enc.size + 4
                        if (rateT0 == 0L) rateT0 = tR0
                        // 限速：已发字节的时间预算超前于真实时钟 → 睡掉差额
                        val wantMs = bytes * 1000L / RATE_BPS
                        val elMs = tR3 - rateT0
                        if (wantMs > elMs) delay(wantMs - elMs)
                        // ★ 压缩信封后 enc[2] 是 clen 字节，非 SCF1 flags——kf 计数停用
                        //（真 keyframe 数看设备端 servo 5s 日志的 kfDec 行）
                        // ★ 阶段账必须在内循环打：write 一旦被背压卡死，外层永远走不到
                        if (frames % 30 == 0 && stageN > 0) {
                            AppLog.i("WifiCast",
                                "阶段均摊(近${stageN}帧): 渲染${msRender / stageN}ms " +
                                        "编码${msEncode / stageN}ms 写网${msWrite / stageN}ms")
                            msRender = 0; msEncode = 0; msWrite = 0; stageN = 0
                        }
                    }

                    val depth = writeIdx - playIdx
                    status = "池推流 ${writeIdx}/${totalFrames} 帧 · 池深$depth · 车${carMeter.toInt()}m · " +
                            "播放头${if (ph >= 0) "${ph}帧" else "无"} · " +
                            "dev态${devState.get()} · ${(bytes / 1024)}KB · kf$keyframes"
                    if (frames % 30 == 0) {
                        AppLog.i("WifiCast",
                            "池推流 write=$writeIdx play=${ph}帧 深$depth " +
                                    "车${carMeter.toInt()}m 帧$frames ${bytes / 1024}KB kf$keyframes")
                        if (stageN > 0) {
                            AppLog.i("WifiCast",
                                "阶段均摊(近${stageN}帧): 渲染${msRender / stageN}ms " +
                                        "编码${msEncode / stageN}ms 写网${msWrite / stageN}ms")
                            msRender = 0; msEncode = 0; msWrite = 0; stageN = 0
                        }
                    }

                    // 播完（写完且设备播到最后 1 帧）→ 收工
                    if (writeIdx >= totalFrames && ph >= 0 &&
                        playIdx >= totalFrames - 2) {
                        status = "✅ 池播完：$frames 帧 ${bytes / 1024}KB（kf$keyframes）"
                        break
                    }
                    delay(30)
                }
            }
        } catch (e: Exception) {
            status = "❌ 预渲染推流中断: ${e.message}"
            AppLog.i("WifiCast", "池推流异常 ${e.javaClass.simpleName}: ${e.message} " +
                    "(writeIdx=$writeIdx frames=$frames bytes=${bytes / 1024}KB play=${playheadFrame.get()}帧 dev=${devState.get()})")
        } finally {
            beatJob?.cancel()
            runCatching { BleClientProvider.client.rawDevStatusNotify = null }   // 摘 0xE3 挂钩
            if (handle != 0L) GtamRenderJni.castClose(handle)
            runCatching { sock?.close() }
            runCatching { if (wifiLock.isHeld) wifiLock.release() }
            exec.close()
        }
    }
}

/** iperf 测速（TCP 灌流 10s → 设备 :5001 精简 iperf server）：
 *  64KB 块持续 write，write 阻塞时内核发缓冲满=真实背压——测的是「无线管道裸吞吐」，
 *  不含渲染/编码。设备串口 [iperf] 行与这里互为印证；iperf2 客户端（Termux 等）
 *  `iperf -c <设备IP> -p 5001 -t 10 -i 2` 打同一端口，结果可交叉验证。 */
private suspend fun runIperfTest(context: Context): String = withContext(Dispatchers.IO) {
    val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    val wifiNet = cm.allNetworks.firstOrNull {
        cm.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
    }
    val sock = try {
        connectDeviceSocket(cm, wifiNet, 5001)
    } catch (e: Exception) {
        return@withContext "❌ 测速连不上设备 :5001——${e.message}"
    }
    // WifiLock：测的是管道不是省电策略（与推流同款）
    val wm = context.getSystemService(Context.WIFI_SERVICE) as WifiManager
    val wifiLock = wm.createWifiLock(
        if (android.os.Build.VERSION.SDK_INT >= 29) WifiManager.WIFI_MODE_FULL_LOW_LATENCY
        else @Suppress("DEPRECATION") WifiManager.WIFI_MODE_FULL_HIGH_PERF,
        "gtam-iperf"
    ).apply { setReferenceCounted(false); acquire() }
    try {
        val out = sock.getOutputStream()
        val buf = ByteArray(64 * 1024)          // 64KB 大块：把 write 系统调用开销压到忽略
        val durMs = 10_000L
        val t0 = SystemClock.elapsedRealtime()
        var total = 0L; var markAt = 0L; var markBytes = 0L
        while (true) {
            out.write(buf)
            total += buf.size
            val el = SystemClock.elapsedRealtime() - t0
            if (el - markAt >= 2000) {
                val m = (total - markBytes) * 8.0 / ((el - markAt) / 1000.0) / 1e6
                AppLog.i("WifiCast", "iperf ${el / 1000}s: ${(total - markBytes) / 1024}KB/2s = %.1f Mbps".format(m))
                markAt = el; markBytes = total
            }
            if (el >= durMs) break
        }
        runCatching { sock.shutdownOutput() }   // FIN 带着内核尾包出去；设备端打总账
        val secs = (SystemClock.elapsedRealtime() - t0) / 1000.0
        "✅ iperf 10s 发送 ${total / 1024 / 1024}MB = %.1f Mbps（以设备串口 [iperf] 总账为准）".format(total * 8.0 / secs / 1e6)
    } catch (e: Exception) {
        "❌ 灌流中断: ${e.message}"
    } finally {
        runCatching { sock.close() }
        runCatching { if (wifiLock.isHeld) wifiLock.release() }
    }
}

/**
 * BLE 连通性测试（2026-10-28 改版：走**生产通道**，不再自建裸 GATT 客户端）：
 * 报生产连接状态 → ready 则发一拍 0xD0 → 挂 1.5s 看 0xE3 回环。
 */
private suspend fun runBleConnectivityTest(context: Context): String {
    val sb = StringBuilder()
    val bdm = BleDeviceManager.getInstance()
    sb.append("生产BLE state=${bdm.bleState.value} connected=${bdm.isConnected.value} ")
    sb.append("设备=${bdm.deviceName.value.ifEmpty { "<无>" }}/${bdm.deviceMac.value.ifEmpty { "-" }}；")
    val client = BleClientProvider.client
    if (!client.ready) return sb.append("❌ 通道未就绪——先去蓝牙页连接设备").toString()
    sb.append("ready✓ 队列=${client.getQueueSize()}；")
    // 发一拍 0xD0 试链路（无会话时设备不回 0xE3 属正常）
    val gotE3 = java.util.concurrent.atomic.AtomicBoolean(false)
    val lastE3 = java.util.concurrent.atomic.AtomicReference<String>("")
    client.rawDevStatusNotify = { raw ->
        if (raw.size >= 10 && raw[0] == 0xE3.toByte()) {
            gotE3.set(true)
            val bb = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN)
            bb.position(1)
            lastE3.set("seg${bb.int} play${bb.int}帧 state${bb.get().toInt()}")
        }
    }
    try {
        val d0 = ByteBuffer.allocate(10).order(ByteOrder.LITTLE_ENDIAN)
            .put(0xD0.toByte()).putInt(1).putInt(0).put(0).array()
        client.send(d0)
        sb.append("0xD0已发；")
        delay(1500)
        sb.append(if (gotE3.get()) "0xE3回环✓ ($lastE3)" else "未收到0xE3(无会话时属正常)")
    } finally {
        client.rawDevStatusNotify = null
    }
    return sb.toString()
}

/** 连通性测试：找 WiFi 网络 → 绑定 socket → TCP 连 → 发会话头 → 挂 5s → 断开 */
private suspend fun runWifiCastTest(context: Context): String = withContext(Dispatchers.IO) {
    // ① 找 WiFi 网络；找不到 ≠ 出错——App 手机自己开热点时，自己的 AP 不算
    //    TRANSPORT_WIFI，此时用不绑定的裸 socket（本机 AP 网段路由在主表里）
    val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    val wifiNet = cm.allNetworks.firstOrNull {
        cm.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
    }

    // ② 自动发现设备（网段前缀 + /24 并行探测 :7123）
    val sock = try {
        connectDeviceSocket(cm, wifiNet)
    } catch (e: Exception) {
        return@withContext "❌ 连不上设备——确认设备已连上热点且屏上显示 IP（${e.message}）"
    }
    try {
        // ③ 会话头 16B（小端；rsv 6 字节补齐到 16，设备端 recv_exact(16)）
        val hdr = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN)
            .put("GWC1".toByteArray(Charsets.US_ASCII))
            .putShort(466)     // w
            .putShort(466)     // h
            .putShort(30)      // fps
            .put(ByteArray(6)) // rsv
            .array()
        sock.getOutputStream().run { write(hdr); flush() }

        // ④ 挂 5 秒：让设备 LINK OK 页肉眼可见（此时不关连接，关了设备立即回 WAIT）
        Thread.sleep(5000)
        "✅ 通！会话头已发——设备屏应显示过 LINK OK，现已断开回等待页"
    } catch (e: Exception) {
        when (e) {
            is SocketTimeoutException -> "❌ 发送超时——设备屏在 WAIT 页吗？看设备串口 gtamwc 日志"
            is ConnectException -> "❌ 拒绝连接——设备 TCP 没起？看设备串口 gtamwc 日志（TCP 监听 :7123）"
            else -> "❌ ${e.javaClass.simpleName}: ${e.message}"
        }
    } finally {
        runCatching { sock.close() }   // finally 关：保住上面 5s 的 LINK 展示窗口
    }
}

/** 从网关（App 连热点=客户端形态）或本机 AP 接口地址（App 自开热点=主机形态）
 *  推设备所在网段前缀，如 "192.168.43"。 */
private fun deviceSubnetPrefix(cm: ConnectivityManager, wifiNet: android.net.Network?): String? {
    if (wifiNet != null) {
        cm.getLinkProperties(wifiNet)?.routes?.firstOrNull { it.isDefaultRoute }?.gateway
            ?.hostAddress?.takeIf { it.count { c -> c == '.' } == 3 }?.let { return it.substringBeforeLast('.') }
    }
    // 热点主机形态：枚举本机接口找 AP 网段（如 swlan0/ap0 的 192.168.43.x）
    val nifs = runCatching { java.net.NetworkInterface.getNetworkInterfaces() }.getOrNull() ?: return null
    for (nif in nifs.asSequence()) {
        for (addr in nif.inetAddresses) {
            val h = addr.hostAddress ?: continue
            if (addr is java.net.Inet4Address && !addr.isLoopbackAddress &&
                (h.startsWith("192.168.") || h.startsWith("172.20.10."))
            ) return h.substringBeforeLast('.')
        }
    }
    return null
}

/** 设备 IP 自动发现：网段 /24 全扫 [port]（64 并发 × 600ms，全网段 ~2s）。
 *  多个探测成功时取第一个，其余关闭。绑定 wifiNet（客户端形态）或裸 socket（主机形态）。 */
private fun connectDeviceSocket(cm: ConnectivityManager, wifiNet: android.net.Network?, port: Int = 7123): Socket {
    val prefix = deviceSubnetPrefix(cm, wifiNet)
        ?: error("推不出网段——确认设备已连上热点（屏上应有 IP）")
    val pool = Executors.newFixedThreadPool(64)
    val found = java.util.concurrent.ConcurrentLinkedQueue<Socket>()
    val latch = java.util.concurrent.CountDownLatch(253)
    try {
        for (i in 2..254) {
            pool.execute {
                var s: Socket? = null
                try {
                    s = wifiNet?.socketFactory?.createSocket() ?: Socket()
                    s.connect(InetSocketAddress("$prefix.$i", port), 600)
                    s.tcpNoDelay = true   // 关 Nagle：帧流要低延迟，不凑包
                    found.add(s)
                } catch (e: Exception) {
                    runCatching { s?.close() }
                } finally {
                    latch.countDown()
                }
            }
        }
        // 首个成功即返回（不等全网段扫完）；6s 上限兜底
        val deadline = System.currentTimeMillis() + 6000
        while (found.isEmpty() && System.currentTimeMillis() < deadline) {
            if (latch.await(100, java.util.concurrent.TimeUnit.MILLISECONDS)) break
        }
    } finally {
        pool.shutdownNow()
    }
    val hit = found.poll() ?: error("网段 $prefix.x 没探到设备 :$port（真值以设备屏 IP 为准）")
    while (true) {                       // 关掉其余并发成功的连接
        val extra = found.poll() ?: break
        runCatching { extra.close() }
    }
    return hit
}
