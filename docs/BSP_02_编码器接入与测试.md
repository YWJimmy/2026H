# BSP 02：MG513X 编码器接入与测试

## 1. 本次实现范围

本次仅新增：

```text
User/bsp/bsp_encoder.c
User/bsp/bsp_encoder.h
```

当前硬件映射：

| 位置 | 定时器 | A相 | B相 |
|---|---|---|---|
| 左轮 | TIM3 | PC6 / TIM3_CH1 | PC7 / TIM3_CH2 |
| 右轮 | TIM4 | PD12 / TIM4_CH1 | PD13 / TIM4_CH2 |

编码器型号为 MG513X，暂按 3.3 V 供电。A、B 相输出类型尚未确认，当前 CubeMX GPIO 配置为 `No Pull`。

## 2. 计数定义

本项目统一采用：

```text
车轮完整旋转一圈 = 1468 个计数
```

这里的 1468 已经是 STM32 定时器 `Encoder Mode TI1 and TI2` 四倍频解码后的结果，上层不得再次乘以 4。

头文件中定义为：

```c
#define BSP_ENCODER_COUNTS_PER_REV ((int32_t)1468)
```

## 3. CubeMX 配置要求

### 左编码器 TIM3

```text
Combined Channels = Encoder Mode
Encoder Mode       = TI1 and TI2
Prescaler          = 0
Counter Period     = 65535
IC1 Filter         = 6
IC2 Filter         = 6
PC6                = TIM3_CH1
PC7                = TIM3_CH2
```

### 右编码器 TIM4

```text
Combined Channels = Encoder Mode
Encoder Mode       = TI1 and TI2
Prescaler          = 0
Counter Period     = 65535
IC1 Filter         = 6
IC2 Filter         = 6
PD12               = TIM4_CH1
PD13               = TIM4_CH2
```

PD12、PD13 同时具有 FMC 地址线复用功能。当前使用 TIM4 后，不要再启用 FMC 或板载外部 SRAM。

## 4. 接入 Keil 工程

1. 将两个文件复制到仓库的 `User/bsp/`。
2. 在 Keil 的 `User/bsp` 分组中添加 `bsp_encoder.c`。
3. 在工程 Include Paths 中加入：

```text
../User/bsp
```

4. 在 `main.c` 或后续 `App_Init()` 中包含：

```c
#include "bsp_encoder.h"
```

5. 必须在 CubeMX 定时器初始化之后调用：

```c
MX_TIM3_Init();
MX_TIM4_Init();

if (!BSP_Encoder_Init())
{
    Error_Handler();
}
```

## 5. BSP 接口

```c
bool BSP_Encoder_Init(void);
bool BSP_Encoder_IsInitialized(void);
void BSP_Encoder_Reset(void);

int16_t BSP_Encoder_GetLeftDelta(void);
int16_t BSP_Encoder_GetRightDelta(void);

int32_t BSP_Encoder_GetLeftTotal(void);
int32_t BSP_Encoder_GetRightTotal(void);
```

每次调用 `GetLeftDelta()` 或 `GetRightDelta()` 时：

1. 读取对应 16 位硬件计数器；
2. 自动处理计数器回绕；
3. 应用方向修正；
4. 返回本周期增量；
5. 更新对应软件累计计数。

因此，应在固定控制周期中各调用一次，不要在多个中断和主循环中重复读取同一侧。

## 6. 临时测试方法

建议先架空车轮，关闭电机输出，手动转动车轮。

测试代码示意：

```c
static uint32_t last_tick = 0U;

if ((HAL_GetTick() - last_tick) >= 10U)
{
    int16_t left_delta;
    int16_t right_delta;

    last_tick += 10U;

    left_delta = BSP_Encoder_GetLeftDelta();
    right_delta = BSP_Encoder_GetRightDelta();

    /* 将增量和累计值通过 DEBUG 串口输出。 */
}
```

### 检查项目

1. 只转左轮时，只有左侧计数变化。
2. 只转右轮时，只有右侧计数变化。
3. 向前方向转动车轮时，计数应为正。
4. 反向转动时，计数应为负。
5. 车轮完整旋转一圈，累计计数绝对值应接近 1468。

## 7. 方向修正

如果某侧向前转动时计数为负，只修改 `bsp_encoder.c` 中的宏：

```c
#define BSP_ENCODER_LEFT_REVERSED  0U
#define BSP_ENCODER_RIGHT_REVERSED 0U
```

将错误一侧改为 `1U`。上层代码和接口不需要修改。

## 8. 常见问题

### 完全不计数

依次检查：

- 编码器是否有 3.3 V 供电；
- 编码器和 STM32 是否共地；
- A、B 相是否确实连接到对应引脚；
- `BSP_Encoder_Init()` 是否返回 `true`；
- A、B 相是否存在 0～3.3 V 跳变；
- TIM3、TIM4 是否仍为 Encoder Mode。

### 静止时自行计数

可能原因：

- 编码器输出为开集电极，但没有上拉；
- 电机线与编码器线干扰严重；
- 接地不可靠；
- 输入滤波值偏小。

由于输出类型尚未确认，出现浮空时优先用示波器或逻辑分析仪检查。若确认是开集电极输出，应增加外部上拉电阻，再决定是否在 CubeMX 中启用内部上拉。

### 高速时漏计数

可将 `IC1 Filter` 和 `IC2 Filter` 从 6 逐步减小。修改后必须重新测试静止噪声。

### 一圈不是 1468

先确认测试的是车轮输出轴完整一圈，并记录实际值。不要在 BSP 中擅自乘以 4，因为当前 1468 已定义为硬件四倍频后的每圈计数。

## 9. 16 位增量读取限制

TIM3、TIM4 均为 16 位计数器。当前增量算法要求两次读取之间的真实计数变化绝对值小于 32768。

按每圈 1468 计数计算，相当于一次采样间隔内不能超过约 22 圈。正常 1～10 ms 控制周期远低于这一限制。
