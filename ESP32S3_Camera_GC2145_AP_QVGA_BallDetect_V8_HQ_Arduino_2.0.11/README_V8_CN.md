# ESP32S3 GC2145 钢球识别图传 V8 HQ

适配环境：

- 幻尔 ESP32S3-Cam / GC2145
- Arduino-ESP32 2.0.11
- Board：ESP32S3 Dev Module
- PSRAM：OPI PSRAM
- 分辨率：QVGA 320×240
- RGB565 单缓冲，帧缓冲位于 PSRAM
- AP 热点：`HW_ESP32S3CAM`
- 网页：`http://192.168.5.1`

## V8 默认画质

```cpp
#define SOFTWARE_JPEG_QUALITY 72
```

这是在用户实测 JPEG 质量 70 能稳定运行的基础上，小幅提高画质后的默认值。

## 相比 V7 的优化

1. 钢球识别由每帧全 ROI 扫描，改为每 2 帧识别一次；未识别帧继续绘制上一次有效红框。
2. 跟踪有效时只扫描上次钢球中心左右 42 像素，每 8 帧执行一次完整管道 ROI 扫描，兼顾速度和失锁恢复。
3. JPEG 网络发送块由 8 KB 增大到 24 KB，减少每帧 HTTP 分块调用次数。
4. 移除每个网络分块后的主动 `taskYIELD()`，改为每帧发送完成后让出一次任务。
5. JPEG 持久缓冲提高到 80 KB，降低质量 72 时因单帧过大而丢帧的概率。
6. 内部 RAM 保留量提高到 96 KB；连续内部 RAM 不足时自动把 JPEG 缓冲放入 PSRAM，避免挤压 Wi-Fi/TCP。
7. 开启 GC2145 的 BPC、WPC、Raw GMA、Lens Correction，并设置轻度锐化和对比度增强。这些属于传感器侧处理，几乎不增加 ESP32 每帧 CPU 开销。
8. 保留 V7 的 320×240、单缓冲、Start/Stop、固定 ROI、红色钢球框、旧连接释放和单视频流保护。

## 可调参数

文件：`CameraWebServer_MaxFPS/stream_profile.h`

```cpp
#define SOFTWARE_JPEG_QUALITY 72
#define BALL_DETECT_EVERY_N_FRAMES 2
#define BALL_FULL_SCAN_EVERY_N_FRAMES 8
#define BALL_TRACK_SEARCH_RADIUS 42
```

- 画质仍不够：将质量改为 75；不建议直接超过 80。
- 帧率仍低：先将质量恢复到 70，而不是降低分辨率。
- 钢球移动很快导致红框跟不上：将检测间隔改为 1，或搜索半径增大到 55。
- 钢球运动较慢且优先帧率：可将检测间隔改为 3。

## 说明

该版本已进行源码静态语法检查和 ZIP 完整性检查，但无法替代幻尔实物板上的实际帧率、画质和识别测试。质量 72 的实际提升取决于画面复杂度、光照、手机浏览器和供电稳定性。
