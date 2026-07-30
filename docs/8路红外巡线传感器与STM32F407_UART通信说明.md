---
title: "8路红外巡线传感器与 STM32F407ZGT6 UART 通信说明"
subtitle: "面向项目成员与 AI 的统一技术文档"
author: "2026H 项目"
date: "2026-07-30"
lang: zh-CN
---

# 目录

1. 文档定位与信息等级
2. 模块快速认识
3. 工作原理与数据含义
4. 模块接口与引脚
5. 安装与机械布置
6. 一键学习与校准
7. UART 通信基础
8. UART 手动模式协议
9. 模拟值与阈值数据帧
10. 厂家资料中的协议歧义
11. STM32F407ZGT6 的 UART 选择与接线
12. STM32CubeMX 配置
13. 推荐软件分层与驱动流程
14. 测试计划与故障排查
15. 项目起步方案
16. AI 可直接读取的事实表
17. 禁止 AI 擅自假定的事项
18. 资料来源与文档边界


# 文档定位

本文档用于统一说明 **Hiwonder LineFollower_8CH v1.0 8 路红外巡线传感器**的硬件特性、学习校准、GPIO/I²C/UART 接口、UART 数据协议，以及其与 **STM32F407ZGT6** 使用 UART4 或 UART5 通信时的接线、CubeMX 配置、软件架构和测试方法。

本文同时服务于两类读者：

1. **项目成员**：能够按步骤完成安装、校准、接线、配置和故障排查。
2. **AI 或代码生成工具**：能够区分厂家资料明确给出的事实、项目推荐方案，以及仍需实测确认的内容，避免把推测当成协议事实。

## 信息等级约定

文中使用以下标记：

- **【厂家事实】**：可由随模块提供的 PDF 资料直接支持。
- **【芯片事实】**：可由 STMicroelectronics 的 STM32F405/407 数据手册支持。
- **【项目建议】**：为本项目的软件和硬件设计建议，不代表厂家协议强制要求。
- **【待实测】**：现有资料不完整、存在歧义或相互矛盾，必须通过实物测试确认。

> AI 使用要求：生成驱动或控制代码时，不得把“项目建议”或“待实测”内容改写为厂家已确认事实。

# 1. 模块快速认识

## 1.1 模块名称与定位

- 产品名称：8 路巡线传感器
- 板上标识：`Hiwonder LineFollower_8CH v1.0`
- 探头数量：8 路红外发射/接收探头
- 板载功能：红外采样、内部 MCU 处理、一键学习、通道指示灯、GPIO/I²C/UART 数据输出
- 典型用途：差速小车、麦克纳姆轮小车、履带车、阿克曼小车等的黑线或其他灰度目标识别

【厂家事实】模块通过红外探头发射并接收反射光，反射越强时内部采样输出越大，反射越弱时内部采样输出越小；板载 MCU 会处理各通道信号，并把结果通过输出接口发送给主控。

## 1.2 主要规格

| 项目 | 厂家资料给出的规格 |
|---|---:|
| 供电电压 | DC 5 V |
| 工作电流 | 85 mA |
| 工作温度 | -10 ℃ ～ 60 ℃ |
| 标称有效探测距离 | 0.5 cm ～ 8 cm |
| 输出接口 | 8 路 GPIO、I²C、UART |
| I²C 7 位地址 | `0x5D` |
| 输出数据 | 数字结果和模拟结果 |
| 产品尺寸 | 107 × 31.2 × 10.57 mm |
| 安装方式 | M2.5 或 M3 铜柱、螺丝 |

【待实测】“0.5 cm ～ 8 cm”是厂家给出的有效量程，不等于整个范围内都适合高速巡线。最终安装高度应通过真实赛道、真实环境光和真实车速测试确定。

# 2. 工作原理与数据含义

## 2.1 红外反射原理

每一路探头包含红外发射和接收部分。不同材料、颜色、表面粗糙度和环境光条件会导致不同的反射强度。模块内部会形成每一路的模拟采样值，并结合学习得到的背景值和目标值，生成识别阈值与数字判断结果。

典型白底黑线场景中，白色通常反射较强，黑色通常反射较弱，但软件不应把“黑一定小、白一定大”写死为绝对规则。模块支持学习不同灰度目标，实际关系应由标定结果和实测数据决定。

## 2.2 三类可用数据

模块可向主控提供三类核心信息：

| 数据类型 | 数据内容 | 典型用途 |
|---|---|---|
| 数字状态 | 8 个通道是否识别到学习目标 | 高频巡线、路口判断、快速控制 |
| 模拟值 | 8 个通道的原始/处理后强度值，每路 16 位 | 加权位置计算、灰度变化分析、标定诊断 |
| 阈值 | 8 个通道内部使用的判断阈值，每路 16 位 | 检查学习效果、定位误判原因 |

【项目建议】正式巡线控制应优先使用数字状态或模拟值；阈值主要用于启动诊断、标定确认和调试，不必在每个控制周期读取。

# 3. 模块接口与引脚

![模块接口与排针位置（摘自厂家资料第 2 页）](line_doc_assets/pinout_crop.png){width=92%}

## 3.1 独立 UART 接口

模块左侧提供一个 4 针 UART 接口：

| 模块引脚 | 正确含义 |
|---|---|
| 5V | 5 V 电源输入 |
| GND | 电源地 |
| TX | 模块 UART 发送数据线 |
| RX | 模块 UART 接收数据线 |

【说明】厂家资料中将 RX 描述为“接受时钟线”，这应视为文字错误。UART 是异步串行通信，RX 的功能是接收数据，不是时钟。

## 3.2 独立 I²C 接口

| 模块引脚 | 含义 |
|---|---|
| 5V | 5 V 电源输入 |
| GND | 电源地 |
| SDA | I²C 数据线 |
| SCL | I²C 时钟线 |

I²C 7 位地址固定为 `0x5D`。

## 3.3 综合排针

综合排针顺序在厂家图中标记为：

```text
5V  GND  S1  S2  S3  S4  S5  S6  S7  S8  TX  RX  SDA  SCL
```

其中 `S1`～`S8` 是并行数字 GPIO 输出。

## 3.4 GPIO 与 UART/I²C 的极性关系

【厂家事实】GPIO 模式下，检测到目标颜色时对应通道输出低电平；厂家同时注明，GPIO 返回结果与 I²C/UART 读取到的电平信号相反。

因此，按资料关系可推导：

- GPIO：目标颜色通常对应 `0`；
- I²C/UART 数字结果：目标颜色通常对应 `1`。

【待实测】第一次接入时必须用黑线逐路滑过 S1～S8，确认 UART 中 `1` 是否确实表示“检测到目标”。BSP 层应提供统一极性转换，禁止上层算法直接依赖模块原始极性。

# 4. 安装与机械布置

## 4.1 安装方向

模块应横向安装，使 8 路探头覆盖赛道宽度：

```text
                   小车前进方向
                        ↑

     S1  S2  S3  S4  S5  S6  S7  S8
     ●   ●   ●   ●   ●   ●   ●   ●
     ───────────────────────────────
                赛道横截面
```

【待实测】厂家资料未规定 S1 必然是车辆左侧或右侧。实际方向取决于模块朝向、安装面和观察角度。软件必须允许通过配置反转通道顺序。

推荐定义：

```c
logical_left_to_right[0..7]
```

由适配层把物理 `S1..S8` 映射为统一的“车辆左到右”顺序。

## 4.2 高度与姿态

- 学习时高度必须与正式运行高度一致。
- 模块应尽量与地面平行。
- 8 个探头到地面的距离应基本一致。
- 应避免车体震动、安装板弯曲和轮胎跳动导致探头高度明显变化。
- 应在真实赛道材料、真实光照和真实车速下验证。

## 4.3 环境影响

可能影响识别的数据包括：

- 阳光或强红外光源；
- 镜面反光地面；
- 黑色胶带表面褶皱；
- 灰尘和污渍；
- 探头高度变化；
- 模块倾斜；
- 不同批次赛道材料。

【项目建议】标定数据应记录测试日期、赛道材料、环境光、安装高度和模块方向，便于复现问题。

# 5. 一键学习与校准

模块在正常识别前需要学习背景颜色和目标颜色。

## 5.1 学习背景颜色

以“白底黑线”为例，白色是背景：

1. 模块上电并保持稳定。
2. 将 8 个探头全部放在白色背景区域。
3. 保持正式安装高度和姿态。
4. 长按白色学习按钮。
5. 最外围四盏 LED 闪烁，表示正在学习背景。
6. 等到所有 LED 开始闪烁，表示背景学习成功。
7. 松开按钮。

## 5.2 学习目标颜色

1. 将 8 个探头全部放在目标颜色区域，例如足够宽的黑色区域。
2. 短按一次白色学习按钮。
3. 最内侧四盏 LED 闪烁，表示正在学习目标颜色。
4. 所有 LED 恢复正常速率闪烁，表示学习成功。
5. 若所有 LED 快速闪烁，表示学习失败，需要重新学习。

## 5.3 学习后的验证

按以下顺序验证：

1. 全部放在背景上，确认无明显误触发。
2. 用目标线依次经过 S1、S2、…、S8。
3. 检查板上通道 LED 是否与位置一致。
4. 通过 UART 读取状态字节，检查 bit0～bit7 的变化顺序。
5. 分别读取背景和目标下的 8 路模拟值。
6. 读取 8 路阈值，确认阈值位于两类采样值之间或至少能稳定区分两类表面。

# 6. UART 通信基础

## 6.1 串口参数

厂家示例程序采用：

```text
波特率：115200 bit/s
数据位：8 bit
校验位：None
停止位：1 bit
硬件流控：None
简称：115200, 8N1
```

【待实测】厂家概述称 UART 支持自定义波特率，但所提供资料未给出修改波特率的具体命令。因此本项目第一版驱动固定使用 115200，不自行设计未公开的配置命令。

## 6.2 TX/RX 接线原则

```text
模块 TX  ─────→  STM32 RX
模块 RX  ←─────  STM32 TX
模块 GND ──────  STM32 GND
模块 5V  ──────  5 V 电源
```

TX 和 RX 必须交叉连接，电源地必须共地。

## 6.3 上电后的工作模式配置

模块 UART 支持四种初始化模式：

| 上电后发送的配置字节 | 模式 |
|---:|---|
| `0x00` | 手动配置/手动查询模式 |
| `0x01` | 自动发送数字电平值 |
| `0x02` | 自动发送模拟值 |
| `0x03` | 自动发送阈值 |

【厂家事实】运行模式配置后无法在当前上电周期重新配置。需要更换模式时，应复位或重新上电传感器，再重新发送模式字节。

重要影响：

- 只复位 STM32，不一定会让传感器退出原模式。
- 如果 STM32 重启而传感器未掉电，STM32 再次发送模式配置可能不被接受。
- 调试时建议 STM32 与传感器同时上电，或给传感器保留可控复位/电源开关能力。

# 7. UART 手动模式协议

## 7.1 初始化

模块刚上电后，STM32 发送一个字节：

```text
00
```

表示进入手动查询模式。

## 7.2 手动查询指令

| STM32 发送 | 请求内容 | 厂家资料说明的返回 |
|---:|---|---|
| `0x01` | 读取数字状态 | 返回 1 Byte |
| `0x02` | 读取 8 路模拟值 | 按 UART 数据帧返回 |
| `0x03` | 读取 8 路阈值 | 按 UART 数据帧返回 |

## 7.3 数字状态字节

发送 `0x01` 后返回 1 字节。厂家示例按以下方式解析：

```text
bit0 -> S1
bit1 -> S2
bit2 -> S3
bit3 -> S4
bit4 -> S5
bit5 -> S6
bit6 -> S7
bit7 -> S8
```

解析代码：

```c
for (uint8_t i = 0; i < 8; i++) {
    state[i] = (state_byte >> i) & 0x01U;
}
```

例如返回 `0x18`：

```text
0x18 = 0001 1000b
bit3 = 1 -> S4 有效
bit4 = 1 -> S5 有效
```

【待实测】通道在车辆左右方向上的物理顺序和状态有效极性必须通过实物确认。

# 8. 模拟值与阈值数据帧

![UART 模式与数据帧定义（摘自厂家快速上手第 11 页）](line_doc_assets/protocol_crop.png){width=92%}

## 8.1 数据帧结构

对 `0x02` 模拟值请求和 `0x03` 阈值请求，厂家示例读取 `8 × 2 + 5 = 21` 字节：

| 字节索引 | 内容 |
|---:|---|
| 0 | 帧头 `0x55` |
| 1 | 帧头 `0xAA` |
| 2 | 指令：`0x02` 或 `0x03` |
| 3 | 有效数据长度：`0x10`，即 16 字节 |
| 4 | S1 低 8 位 |
| 5 | S1 高 8 位 |
| 6 | S2 低 8 位 |
| 7 | S2 高 8 位 |
| … | … |
| 18 | S8 低 8 位 |
| 19 | S8 高 8 位 |
| 20 | 校验字节 |

帧可概括为：

```text
55 AA CMD 10 D0L D0H D1L D1H ... D7L D7H CHECK
```

## 8.2 字节序

每一路数据为 16 位无符号数，低字节在前、高字节在后，即小端格式：

```c
value[i] = (uint16_t)frame[4 + i * 2]
         | ((uint16_t)frame[5 + i * 2] << 8);
```

## 8.3 校验算法

厂家协议图对校验的描述为：

> 除去帧头 2 字节，其他字节相加后取反。

据此，接收帧校验应计算：

```text
CHECK = ~(CMD + LEN + DATA0 + DATA1 + ... + DATAN) 的低 8 位
```

参考实现：

```c
static uint8_t line_checksum(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t sum = 0U;

    /* 跳过 frame[0]、frame[1] 两个帧头；最后一字节是接收校验值 */
    for (uint16_t i = 2U; i < frame_len - 1U; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }

    return (uint8_t)(~sum);
}
```

接收端至少应验证：

1. 帧头是否为 `0x55 0xAA`；
2. 指令字段是否与本次请求一致；
3. 数据长度是否为 `0x10`；
4. 校验值是否正确；
5. 接收字节数是否完整。

# 9. 厂家资料中的一处协议歧义

厂家“快速上手”资料明确写明：手动模式配置为 `0x00` 后，读取指令 `0x01`、`0x02`、`0x03` 分别返回状态、模拟值和阈值。

但 STM32 UART 教程又写道：由于 UART 工作模式需要重新上电才能改变，因此示例无法同时打印电平值、模拟值和阈值。

这两处说明存在表面矛盾：

- 解释 A：手动模式下可以连续发送 1/2/3 查询，STM32 示例只是程序组织上一次仅测试一种数据；
- 解释 B：模块内部对手动查询类型也存在额外限制，资料没有完整说明。

【待实测】在驱动正式定型前，应执行以下实验：

```text
模块重新上电
发送 00
发送 01，检查是否返回 1 字节
发送 02，检查是否返回 21 字节
发送 03，检查是否返回 21 字节
再次发送 01，检查是否仍返回 1 字节
```

只有完成该测试后，才能确认手动模式是否允许三类数据交替查询。AI 生成代码时必须保留这个验证状态，不得自行假定。

# 10. 与 STM32F407ZGT6 的 UART 选择

## 10.1 UART4 可选引脚

【芯片事实】STM32F407 的 UART4 使用复用功能 AF8，可选：

| 功能 | 引脚 | 复用 |
|---|---|---|
| UART4_TX | PA0 | AF8 |
| UART4_RX | PA1 | AF8 |
| UART4_TX | PC10 | AF8 |
| UART4_RX | PC11 | AF8 |

## 10.2 UART5 引脚

【芯片事实】UART5 使用：

| 功能 | 引脚 | 复用 |
|---|---|---|
| UART5_TX | PC12 | AF8 |
| UART5_RX | PD2 | AF8 |

## 10.3 选择建议

### 推荐方案 A：UART4，PC10/PC11

```text
PC10 -> UART4_TX
PC11 -> UART4_RX
```

优点：TX/RX 同属 GPIOC，布线与配置直观。

### 备选方案 B：UART5，PC12/PD2

```text
PC12 -> UART5_TX
PD2  -> UART5_RX
```

优点：当 UART4 已被占用时可独立使用。

### 备选方案 C：UART4，PA0/PA1

```text
PA0 -> UART4_TX
PA1 -> UART4_RX
```

适合 PC10/PC11 已分配给其他外设的情况。

## 10.4 典型复用冲突

- PC10、PC11、PC12、PD2 与 SDIO 的部分数据/时钟/命令信号存在复用关系。
- PA0、PA1 也可能被 ADC、定时器、以太网或其他功能占用。

【项目建议】最终选型必须以当前 `.ioc` 文件和整车引脚表为准，不能只根据“UART4/UART5 可用”下结论。

# 11. STM32F407 接线表

## 11.1 UART4 PC10/PC11

| 传感器 | STM32F407ZGT6 |
|---|---|
| 5V | 5 V 电源 |
| GND | GND |
| TX | PC11 / UART4_RX |
| RX | PC10 / UART4_TX |

## 11.2 UART5 PC12/PD2

| 传感器 | STM32F407ZGT6 |
|---|---|
| 5V | 5 V 电源 |
| GND | GND |
| TX | PD2 / UART5_RX |
| RX | PC12 / UART5_TX |

## 11.3 电平兼容性

【厂家事实】模块供电电压为 5 V。

【待实测】厂家资料没有给出 UART TX/RX 的逻辑电平规格，也没有明确说明 TX 高电平是 3.3 V 还是 5 V。因此：

1. 不应仅根据“模块使用 5 V 供电”推断 UART 一定输出 5 V；
2. 也不应仅根据厂家示意图直接假定所有 STM32 引脚都可安全接收；
3. 首次接线前应使用示波器或万用表测量模块 TX 空闲高电平；
4. 若确认模块 TX 超出所选 STM32 引脚允许范围，应使用合适的电平转换；
5. STM32 TX 的 3.3 V 高电平能否被模块 RX 稳定识别，也应实测验证。

# 12. STM32CubeMX 配置

以下以 UART4 PC10/PC11 为例。

## 12.1 外设启用

```text
Connectivity -> UART4 -> Asynchronous
```

选择：

```text
PC10 = UART4_TX
PC11 = UART4_RX
```

## 12.2 参数

| 参数 | 设置 |
|---|---|
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Direction | Receive and Transmit |
| Hardware Flow Control | None |
| Oversampling | 16 |

## 12.3 中断和 DMA

分阶段建议：

1. **协议初测**：阻塞式 `HAL_UART_Transmit` + `HAL_UART_Receive`，最容易观察错误。
2. **BSP 初版**：中断接收或带超时的非阻塞状态机。
3. **正式控制**：根据控制周期与 CPU 占用决定是否使用 DMA。

数字状态只有 1 字节，21 字节模拟帧也很短，因此 DMA 不是“必须”。选择标准应是软件架构和实时性，而不是帧长本身。

# 13. 推荐软件分层

```text
Core/Drivers（CubeMX 生成）
    usart.c / usart.h

User/bsp
    bsp_line_uart.c
    bsp_line_uart.h
        - UART 收发
        - 模式初始化
        - 帧头/长度/校验检查
        - 超时与错误码

User/driver 或 module
    line_sensor.c
    line_sensor.h
        - 物理通道映射
        - 有效极性统一
        - 数字/模拟/阈值统一数据结构
        - 对上层屏蔽 UART/I²C/GPIO 差异

User/module
    line_process.c
    line_process.h
        - 加权位置
        - 误差计算
        - 丢线、全黑、路口状态
        - 过滤与可信度

User/app
    app_line_follow.c
        - 固定周期调度
        - 速度规划
        - 转向与底盘闭环调用

User/test
    test_line_uart.c
        - 单模块测试
        - 原始帧打印
        - 校验与错误统计
```

## 13.1 统一数据结构建议

```c
typedef enum {
    LINE_STATUS_OK = 0,
    LINE_STATUS_TIMEOUT,
    LINE_STATUS_BAD_HEADER,
    LINE_STATUS_BAD_COMMAND,
    LINE_STATUS_BAD_LENGTH,
    LINE_STATUS_BAD_CHECKSUM,
    LINE_STATUS_NOT_INITIALIZED
} line_status_t;

typedef struct {
    uint8_t  target_mask;      /* 统一约定：1 表示检测到目标 */
    uint16_t analog[8];
    uint16_t threshold[8];
    uint32_t sequence;
    uint32_t timestamp_ms;
    line_status_t status;
} line_sensor_sample_t;
```

## 13.2 通道映射建议

```c
typedef struct {
    uint8_t physical_to_logical[8];
    bool uart_one_means_target;
} line_sensor_config_t;
```

这样更换模块、翻转安装方向或切换 GPIO/UART 后，上层位置算法不需要修改。

# 14. 推荐 UART 驱动流程

## 14.1 初始化状态机

```text
系统上电
  ↓
等待传感器电源稳定
  ↓
清空 STM32 UART 接收缓冲
  ↓
发送模式字节 0x00
  ↓
记录 initialized = true
  ↓
执行首个 0x01 查询验证通信
```

【项目建议】等待时间初版可使用保守值，例如 50～100 ms，但该数值不是厂家明确参数，后续应实测缩短。

## 14.2 读取数字状态

```text
检查 initialized
  ↓
清理过期接收数据
  ↓
发送 0x01
  ↓
在超时内接收 1 字节
  ↓
按 bit0..bit7 解析 S1..S8
  ↓
执行极性和通道映射
  ↓
输出统一 target_mask
```

## 14.3 读取 21 字节帧

```text
发送 0x02 或 0x03
  ↓
查找 0x55 0xAA 帧头
  ↓
读取 CMD、LEN
  ↓
验证 LEN == 0x10
  ↓
读取 16 字节数据 + 1 字节校验
  ↓
验证 CMD 与请求一致
  ↓
验证校验
  ↓
按小端解析 8 个 uint16_t
```

【项目建议】正式版不要假设一次 UART 接收调用必然从帧头开始。即使当前采用请求-响应，也应保留丢弃错误字节和重新寻找帧头的能力。

# 15. 最小 HAL 测试逻辑

以下代码用于验证思路，不代替最终 BSP：

```c
uint8_t mode = 0x00U;
uint8_t cmd  = 0x01U;
uint8_t mask = 0U;

HAL_Delay(100U);
HAL_UART_Transmit(&huart4, &mode, 1U, 20U);
HAL_Delay(2U);

HAL_UART_Transmit(&huart4, &cmd, 1U, 20U);
if (HAL_UART_Receive(&huart4, &mask, 1U, 20U) == HAL_OK) {
    printf("LINE,MASK=0x%02X\r\n", mask);
} else {
    printf("LINE,RX_TIMEOUT\r\n");
}
```

首次测试只读取数字状态，不要同时加入电机控制、PID、OLED 刷新等功能，以免问题难以定位。

# 16. 测试计划

## 16.1 电气测试

- [ ] 模块 5 V 电源稳定
- [ ] STM32 与模块共地
- [ ] 模块 TX 空闲电平已测量
- [ ] TX/RX 交叉连接
- [ ] 逻辑分析仪确认波特率约为 115200
- [ ] STM32 上电后确实发送 `0x00`

## 16.2 协议测试

- [ ] `0x01` 返回 1 字节
- [ ] bit0～bit7 与 S1～S8 一一对应
- [ ] 目标有效极性已确认
- [ ] `0x02` 返回 `55 AA 02 10 ... CHECK`
- [ ] `0x03` 返回 `55 AA 03 10 ... CHECK`
- [ ] 校验算法通过真实帧验证
- [ ] 手动模式能否连续交替查询 1/2/3 已确认
- [ ] STM32 单独复位、传感器不断电时的行为已确认

## 16.3 光学与机械测试

- [ ] 白底背景学习成功
- [ ] 黑线目标学习成功
- [ ] 每一路 LED 响应正常
- [ ] 高度变化对模拟值的影响已记录
- [ ] 环境光变化测试通过
- [ ] 模块物理左右方向已记录
- [ ] 车辆振动时无频繁误判

## 16.4 软件稳定性测试

- [ ] UART 超时不会卡死主循环
- [ ] 错帧不会更新有效样本
- [ ] 连续错误有计数器
- [ ] 可检测传感器掉线
- [ ] 恢复通信后可重新初始化
- [ ] 调试打印不会阻塞控制周期

# 17. 常见故障排查

## 17.1 完全没有返回数据

依次检查：

1. 是否共地；
2. TX/RX 是否交叉；
3. 是否为 115200、8N1；
4. 传感器是否真正上电；
5. STM32 是否发出了模式字节；
6. 传感器是否在 STM32 重启前已经锁定其他模式；
7. UART 引脚是否被其他外设占用；
8. CubeMX 是否生成并调用对应 `MX_UARTx_Init()`。

## 17.2 状态有变化但左右相反

- 物理安装方向与软件假设相反；
- S1～S8 的车辆左右映射错误。

解决：只修改通道映射表，不修改上层位置算法。

## 17.3 状态全反

- UART 中的有效极性与当前软件约定相反；
- GPIO 极性被错误套用到 UART。

解决：在驱动适配层统一取反，不在多个算法文件中分散处理。

## 17.4 模拟帧偶尔校验失败

检查：

- 接收是否从帧中间开始；
- 是否固定接收 21 字节但缓冲区中残留旧数据；
- 是否错误地把帧头加入校验和；
- 是否遗漏按 8 位溢出后取反；
- UART 线路是否过长或干扰严重；
- 调试串口与传感器串口是否误用同一实例。

## 17.5 学习成功但实际巡线误判

- 学习高度与运行高度不一致；
- 学习时环境光与比赛环境差异过大；
- 目标区域不够宽，部分探头没有完整学习目标；
- 地面反光；
- 模块倾斜或震动；
- 阈值虽然生成，但目标和背景的模拟值间隔过小。

# 18. 本项目推荐的最终起步方案

```text
传感器：Hiwonder LineFollower_8CH v1.0
供电：5 V，共地
主控：STM32F407ZGT6
优先串口：UART4
优先引脚：PC10(TX)、PC11(RX)
串口参数：115200, 8N1
工作模式：0x00 手动模式
第一阶段数据：0x01 数字状态
第二阶段数据：0x02 模拟值
阈值读取：仅启动诊断或调试时使用 0x03
软件接口：BSP 与算法解耦
统一约定：上层始终使用“1 = 检测到目标”
通道顺序：上层始终使用“从车辆左到右”
```

# 19. AI 可直接读取的事实表

```yaml
module:
  vendor_marking: "Hiwonder LineFollower_8CH v1.0"
  sensor_count: 8
  supply_voltage_v: 5
  working_current_ma: 85
  working_temperature_c: [-10, 60]
  nominal_distance_cm: [0.5, 8]
  dimensions_mm: [107, 31.2, 10.57]
  interfaces: [GPIO_8CH, I2C, UART]
  i2c_7bit_address: 0x5D

calibration:
  background:
    action: "long_press_key"
    led_behavior: "outer_four_flash_then_all_flash"
  target:
    action: "short_press_key"
    led_behavior_success: "inner_four_flash_then_normal_flash"
    led_behavior_failure: "all_fast_flash"
  requirement: "calibration_height_must_match_runtime_height"

uart:
  baud_rate: 115200
  data_bits: 8
  parity: none
  stop_bits: 1
  mode_config:
    manual: 0x00
    auto_state: 0x01
    auto_analog: 0x02
    auto_threshold: 0x03
  mode_change_requires_sensor_reset_or_power_cycle: true
  manual_commands:
    read_state: 0x01
    read_analog: 0x02
    read_threshold: 0x03
  state_response:
    length_bytes: 1
    bit_mapping:
      bit0: S1
      bit1: S2
      bit2: S3
      bit3: S4
      bit4: S5
      bit5: S6
      bit6: S7
      bit7: S8
  analog_threshold_frame:
    total_length_bytes: 21
    header: [0x55, 0xAA]
    command_index: 2
    data_length_index: 3
    data_length_value: 0x10
    data_start_index: 4
    checksum_index: 20
    endian: little
    checksum: "invert_low8(sum(command + length + all_data_bytes))"

polarity:
  gpio_target_level: low
  gpio_is_opposite_to_i2c_uart: true
  uart_one_means_target: "likely_from_vendor_relation_but_must_verify"

stm32f407:
  uart4:
    af: 8
    pin_options:
      - {tx: PA0, rx: PA1}
      - {tx: PC10, rx: PC11}
  uart5:
    af: 8
    pin_options:
      - {tx: PC12, rx: PD2}
  project_preference:
    peripheral: UART4
    tx: PC10
    rx: PC11

open_questions:
  - "UART logic high voltage is not specified by vendor documents"
  - "Whether manual mode can alternate commands 0x01/0x02/0x03 without sensor reset must be verified"
  - "Physical left-right order of S1..S8 depends on installation"
  - "Exact power-on stabilization delay is not specified"
```

# 20. 禁止 AI 擅自假定的事项

1. 不得假定模块 UART 逻辑电平一定是 3.3 V 或 5 V。
2. 不得假定 S1 永远位于车辆左侧。
3. 不得假定 UART 原始 `1` 已经通过实测确认代表目标。
4. 不得假定手动模式一定可以无限交替读取 1/2/3，必须保留实测结论。
5. 不得把标称 0.5～8 cm 当作推荐安装高度。
6. 不得在上层巡线算法中直接写死模块极性和物理通道顺序。
7. 不得把厂家 STM32 示例中的 `huart2` 写死到本项目；本项目使用 UART4 或 UART5。
8. 不得把调试用阻塞式 HAL 收发直接当作最终实时控制架构。

# 21. 资料来源与范围

## 厂家资料

- 《1. 8路巡线传感器介绍.pdf》：产品原理、规格、接口和引脚。
- 《2. 快速上手.pdf》：学习流程、I²C 寄存器、UART 模式和协议帧。
- 《3. 多种协议主控通信教程.pdf》：Arduino、ESP32、STM32 等平台的通信示例与解析思路。

## 芯片资料

- STMicroelectronics, *STM32F405xx / STM32F407xx Datasheet*, DS8626：UART4/UART5 的复用引脚与 AF8 映射。

## 文档边界

本文没有得到模块内部固件源代码，也没有得到完整的 UART 电气指标、上电时序参数和全部异常响应定义。因此相关项目必须通过逻辑分析仪和实物测试补全，并把结论更新到本文“待实测”项目中。
