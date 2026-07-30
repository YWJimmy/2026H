# K230D 与 STM32 视觉串口通信 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 K230D BOX 通过 PH2.0 接口1的 UART1 向 STM32F407 USART6 持续发送 `SB` 小球检测帧，并由 STM32 非阻塞接收、校验和发布最新结果。

**Architecture:** K230D 使用 IO40/IO41 复用为 UART1，输出以 `\n` 结尾的 ASCII 帧。STM32 BSP 层只负责 USART6 中断接收和有界环形缓冲，Module 层使用独立纯 C 解析器完成逐字节组帧、字段校验和最新结果管理；测试模式只打印通信状态，不驱动电机。

**Tech Stack:** CanMV MicroPython、STM32F407 HAL、Keil MDK-ARM、C99 主机解析测试

---

### Task 1: 纯 C `SB` 协议解析器

**Files:**
- Create: `User/module/vision_protocol.h`
- Create: `User/module/vision_protocol.c`
- Create: `host_tests/test_vision_protocol.c`

- [x] **Step 1: 写失败测试**：覆盖有效有球/无球/预测帧、分片输入、噪声恢复、字段数错误、越界坐标和超长帧。
- [x] **Step 2: 运行失败测试**：`gcc -std=c99 -Wall -Wextra -Werror -IUser/module host_tests/test_vision_protocol.c User/module/vision_protocol.c -o host_tests/test_vision_protocol.exe`，预期在实现前编译失败。
- [x] **Step 3: 实现解析器**：固定 96 字节行缓冲，不动态分配；仅接受 `SB,<found>,x1,y1,x2,y2,cx,cy,score`，坐标范围为 `1280x960`，分数范围 `0..1000`。
- [x] **Step 4: 运行测试**：执行生成的测试程序，预期输出 `vision_protocol: PASS`。

### Task 2: STM32 USART6 非阻塞接收

**Files:**
- Create: `User/bsp/bsp_vision_uart.h`
- Create: `User/bsp/bsp_vision_uart.c`
- Create: `User/module/vision.h`
- Create: `User/module/vision.c`
- Modify: `Core/Src/main.c`
- Modify: `Core/Src/stm32f4xx_it.c`
- Modify: `Core/Inc/stm32f4xx_it.h`
- Modify: `2026H.ioc`
- Modify: `MDK-ARM/2026H.uvprojx`

- [x] **Step 1: 实现 BSP**：USART6 每字节中断接收，ISR 只写入 256 字节环形缓冲并重新挂接接收；统计溢出和 UART 错误。
- [x] **Step 2: 实现 Module**：主循环排空环形缓冲并送入解析器，记录序号、时间戳、有效帧、无效帧和数据新鲜度。
- [x] **Step 3: 接通 IRQ/回调**：启用 `USART6_IRQn`，在全局 UART 回调中同时分发巡线 UART4 和视觉 USART6。

### Task 3: 安全通信测试模式

**Files:**
- Create: `User/test/test_vision_uart.h`
- Create: `User/test/test_vision_uart.c`
- Modify: `User/test/test_config.h`
- Modify: `User/test/test_runner.c`
- Modify: `MDK-ARM/2026H.uvprojx`

- [x] **Step 1: 新增模式**：`TEST_MODE_VISION_UART` 初始化视觉接收但不驱动电机。
- [x] **Step 2: 输出诊断**：周期打印 `found/cx/cy/score/age/rx/overflow/valid/invalid/error`，便于实机确认通信。
- [x] **Step 3: 提供安全入口**：视觉 UART 模式可独立启用；合并远端巡线更新后，仓库默认模式遵循最新 `main`，保持 `TEST_MODE_LINE_FOLLOW`。

### Task 4: K230D UART1 发送

**Files:**
- Modify: `k230d-steel-ball/steel_ball_yolov8_v2_full_w8a8_stable.py`
- Modify: `k230d-steel-ball/README.md`

- [x] **Step 1: 初始化 UART1**：IO40=`UART1_TXD`、IO41=`UART1_RXD`、115200 8N1。
- [x] **Step 2: 发送协议帧**：每次检测只格式化一次，以 `\r\n` 结尾写入 UART，同时保留 IDE 调试输出。
- [x] **Step 3: 清理资源**：退出时调用 UART `deinit()`，初始化失败时打印异常并安全退出。

### Task 5: 验证

**Files:**
- Modify: `docs/superpowers/plans/2026-07-30-k230d-stm32-vision-uart.md`

- [x] **Step 1: 主机解析测试**：GCC 使用 `-Wall -Wextra -Werror` 编译并运行。
- [x] **Step 2: K230D 脚本检查**：Python AST/语法检查 UART1、IO40/IO41 和 `uart.write()`。
- [x] **Step 3: Keil 构建**：工程必须 `0 Error(s)`，记录与本功能有关的 warning。
- [x] **Step 4: 范围检查**：确认 Git 差异只包含通信实现、测试和文档。
