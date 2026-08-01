# BSP 03：USART1 DMA 调试输出与编码器计数测试

## 1. 本次新增文件

```text
User/
├─ bsp/
│  ├─ bsp_debug_uart.c
│  └─ bsp_debug_uart.h
└─ test/
   ├─ test_encoder.c
   └─ test_encoder.h
```

`bsp_encoder.c/.h` 沿用上一阶段文件。

## 2. 串口配置

调试串口固定使用：

```text
USART1_TX = PA9
USART1_RX = PA10
波特率    = 115200
数据位    = 8
停止位    = 1
校验位    = None
```

### CubeMX 中必须增加 USART1_TX DMA

当前工程的 `.ioc` 尚未配置 USART1 TX DMA。打开 CubeMX：

```text
USART1
└─ DMA Settings
   └─ Add → USART1_TX
```

推荐参数：

```text
Direction                 = Memory To Peripheral
Mode                      = Normal
Peripheral Increment      = Disable
Memory Increment          = Enable
Peripheral Data Width     = Byte
Memory Data Width         = Byte
Priority                  = Low
FIFO Mode                 = Disable
```

STM32F407 上由 CubeMX 自动分配合法的 DMA2 Stream/Channel 即可，不要手动覆盖为冲突通道。

生成代码后必须确保：

```c
huart1.hdmatx != NULL
```

否则 `BSP_DebugUart_Init()` 会返回 `false`。

## 3. DMA 调试 printf

正式接口为：

```c
int BSP_Debug_Printf(const char *format, ...);
```

示例：

```c
BSP_Debug_Printf("ENC,L=%ld,R=%ld\r\n", left, right);
```

特点：

- 不重定向全局 `printf()`；
- 使用固定静态队列，不使用动态内存；
- 调用后立即返回，不等待串口发送完成；
- 默认最多缓存 8 条消息；
- 单条消息最长 159 个有效字符；
- 队列满时丢弃新消息并增加丢弃计数；
- 可通过 `BSP_DebugUart_GetDroppedCount()` 查询丢弃数量。

## 4. HAL 回调转发

必须在项目现有的 HAL 回调中加入：

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_DebugUart_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_DebugUart_ErrorCallback(huart);
}
```

建议将回调放在 `main.c` 的 `USER CODE` 区域，或者后续建立统一的 UART 回调分发文件。不要在多个 `.c` 文件中重复定义同名 HAL 回调。

## 5. main.c 接入

头文件：

```c
/* USER CODE BEGIN Includes */
#include "bsp_debug_uart.h"
#include "test_encoder.h"
/* USER CODE END Includes */
```

初始化顺序：

```c
MX_GPIO_Init();
MX_DMA_Init();
MX_USART1_UART_Init();
MX_TIM3_Init();
MX_TIM4_Init();

if (!Test_Encoder_Init())
{
    Error_Handler();
}
```

主循环：

```c
while (1)
{
    Test_Encoder_Update();
}
```

## 6. 串口输出格式

启动帧：

```text
TEST,ENCODER,START,CPR=1468,PERIOD_MS=100
```

周期数据：

```text
ENC,LD=12,LT=1468,RD=11,RT=1465
```

字段：

| 字段 | 含义 |
|---|---|
| LD | 左编码器最近 100 ms 增量 |
| LT | 左编码器累计计数 |
| RD | 右编码器最近 100 ms 增量 |
| RT | 右编码器累计计数 |

MG513X 当前定义为硬件四倍频后每轮一圈 1468 计数。

## 7. 测试方法

1. 架空小车，先不要驱动电机。
2. 打开串口助手，设置 115200、8N1。
3. 手动向前转动左轮一圈。
4. 检查 `LT` 的绝对值是否接近 1468。
5. 手动向前转动右轮一圈。
6. 检查 `RT` 的绝对值是否接近 1468。
7. 若向前转动时计数为负，修改 `bsp_encoder.c` 中对应的方向反转宏。
8. 静止时观察 `LD`、`RD` 是否长期保持 0；若自增，检查接地、供电、屏蔽和 TIM 输入滤波。

## 8. 注意事项

- `Test_Encoder_Update()` 必须在主循环中持续运行。
- 不要在 100 ms 周期内再次调用编码器 Delta 接口，否则会改变本测试的统计区间。
- DMA 发送队列适合调试信息，不保证实时控制数据零丢失。
- 正式控制周期中的编码器采样后续应由底盘模块统一管理，测试代码只用于硬件验证。
