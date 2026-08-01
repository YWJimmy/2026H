# K230D 与 STM32 视觉 UART 通信说明

## 接线

K230D BOX 的 PH2.0 接口1复用为 UART1，程序使用 IO40 发送、IO41 接收。
STM32F407 使用 USART6，PG9 接收、PG14 发送。

```text
K230D UART1_TXD / IO40  -> STM32 USART6_RX / PG9
K230D UART1_RXD / IO41  <- STM32 USART6_TX / PG14
K230D GND               <-> STM32 GND
```

两端参数均为 `115200, 8N1`，无硬件流控。当前协议是 K230D 单向发送检测结果，
因此最小接线只需要 K230D TX、STM32 RX 和共地；反向线为后续命令或握手预留。

K230D BOX 接口定义参考：
<https://wiki.alientek.com/docs/Boards/Kendryte/DNK230D/start-guide/k230d-box-introduction/>

## 协议

每帧为 ASCII 文本，以 `\r\n` 结束。

```text
SB,found,x1,y1,x2,y2,cx,cy,score_milli\r\n
```

- `found=1`：检测到或由短时跟踪器预测到小钢球。
- `found=0`：其余八个数值必须全部为零。
- `x1,y1,x2,y2`：`1280x960` 原图坐标系中的边界框。
- `cx,cy`：边界框中心，必须等于两端点坐标的整数平均值。
- `score_milli`：检测置信度乘 1000，范围 `0..1000`；短时预测帧为 0。

示例：

```text
SB,1,100,200,140,240,120,220,873
SB,0,0,0,0,0,0,0,0
```

STM32 解析器会拒绝字段数错误、非数字、越界坐标、中心不一致、无目标但非零、
分数超过 1000 和超长帧，并在下一行重新同步。

## 程序入口

- K230D：`k230d-steel-ball/steel_ball_yolov8_v2_full_w8a8_stable.py`
- STM32 UART BSP：`User/bsp/bsp_vision_uart.c`
- STM32 协议解析：`User/module/vision_protocol.c`
- STM32 最新结果管理：`User/module/vision.c`
- STM32 安全测试：`User/test/test_vision_uart.c`

需要验证通信时，将 `PROJECT_TEST_MODE` 临时设为 `TEST_MODE_VISION_UART`。
该模式不驱动电机，并通过 USART1 调试口每 100 ms 输出一行 `VU,...` 状态。
合并远端巡线更新后，仓库默认保持 `TEST_MODE_LINE_FOLLOW`。主要字段：

- `SEQ`：已解析有效帧序号；持续增加说明链路和解析都正常。
- `FOUND/CX/CY/SCORE`：最新协议帧数据。
- `VALID`：最新帧距当前不超过 100 ms。
- `RX`：USART6 已接收字节数。
- `VF/IF`：有效帧/无效帧计数。
- `PO/RO`：协议行缓冲/串口环形缓冲溢出计数。
- `UE/RR/LE`：UART 错误数、接收重启数和最后错误码。

## 上电验证

1. 断电完成交叉接线和共地，再同时给两块板上电。
2. 在 K230D 运行稳定脚本，确认 IDE 终端出现 `[STEEL-UART] ready` 和 `SB,...`。
3. 给 STM32 下载当前工程，通过 USART1 调试口查看 `VU,...`。
4. 确认 `RX`、`SEQ`、`VF` 持续增加，`IF/PO/RO/UE` 保持为 0。
5. 遮挡和移开小球，确认 `FOUND` 在 1/0 间变化，坐标与 K230D 终端一致。

若 `RX=0`，先查 TX/RX 是否交叉、是否共地及是否同时重启；若 `RX` 增长但
`SEQ=0`，重点核对波特率、帧尾和 `SB` 字段格式。
