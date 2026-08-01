# MG90S 舵机两极限往返测试

## 测试行为

程序在两个 BSP 极限之间循环切换：

```text
1000 us
→ 保持 2 秒
→ 2000 us
→ 保持 2 秒
→ 1000 us
→ 持续循环
```

程序为非阻塞实现，主循环持续调用：

```c
Test_Servo_Update();
```

## 串口输出

```text
TEST,SERVO,LIMIT_START,MIN_US=1000,MAX_US=2000,HOLD_MS=2000
SERVO,LIMIT=MIN,PULSE_US=1000,EN=1
SERVO,LIMIT=MAX,PULSE_US=2000,EN=1
SERVO,LIMIT=MIN,PULSE_US=1000,EN=1
```

## main.c 接入

```c
#include "bsp_debug_uart.h"
#include "test_servo.h"
```

初始化：

```c
if (!Test_Servo_Init())
{
    Error_Handler();
}
```

主循环：

```c
while (1)
{
    Test_Servo_Update();
}
```

保留现有 USART1 DMA 回调转发。

## 调整保持时间

修改：

```c
#define TEST_SERVO_LIMIT_HOLD_MS 2000U
```

例如每个极限保持 1 秒：

```c
#define TEST_SERVO_LIMIT_HOLD_MS 1000U
```

## 安全检查

第一次运行前，建议先解除舵机与摆杆的刚性连接，确认 1000～2000 us 不会使机构撞击机械限位。若机构实际范围更小，应先修改 `bsp_servo.h` 中的最小、最大脉宽。
