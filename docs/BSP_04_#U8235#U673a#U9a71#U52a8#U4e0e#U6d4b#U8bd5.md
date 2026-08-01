# BSP 04：MG90S 舵机 BSP 与安全范围测试

## 1. 本次新增文件

```text
User/
├─ bsp/
│  ├─ bsp_servo.c
│  └─ bsp_servo.h
└─ test/
   ├─ test_servo.c
   └─ test_servo.h
```

## 2. 当前硬件配置

- 舵机：MG90S
- 信号：PA8 / TIM1_CH1 / `SERVO_PWM`
- 供电：独立 5 V
- STM32 与舵机电源必须共地
- PWM：50 Hz
- TIM1：PSC=167，ARR=19999
- 计数分辨率：1 us/计数

## 3. BSP 设计

BSP 不使用浮点数，只使用整数微秒：

```c
bool BSP_Servo_Init(void);
bool BSP_Servo_Enable(void);
void BSP_Servo_Disable(void);

bool BSP_Servo_SetPulseUs(uint16_t pulse_us);
bool BSP_Servo_SetOffsetUs(int16_t offset_us);

uint16_t BSP_Servo_GetPulseUs(void);
```

逻辑脉宽范围：

```text
1000～2000 us
中位：1500 us
```

超出范围时自动限幅。

初始化后不启动 PWM。只有调用：

```c
BSP_Servo_Enable();
```

才会真正输出波形。

## 4. 方向修正

`bsp_servo.c` 中：

```c
#define BSP_SERVO_REVERSED 0U
```

若机械安装导致控制方向相反，改成：

```c
#define BSP_SERVO_REVERSED 1U
```

上层接口和控制逻辑无需修改。

## 5. 测试动作

测试使用安全的小范围动作：

```text
1500 us
→ 1450 us
→ 1500 us
→ 1550 us
→ 1500 us
```

每个位置保持 2 秒，然后循环。

串口输出示例：

```text
TEST,SERVO,START,MIN_US=1000,CENTER_US=1500,MAX_US=2000,STEP_MS=2000
SERVO,STEP=0,PULSE_US=1500,EN=1
SERVO,STEP=1,PULSE_US=1450,EN=1
SERVO,STEP=2,PULSE_US=1500,EN=1
SERVO,STEP=3,PULSE_US=1550,EN=1
SERVO,STEP=4,PULSE_US=1500,EN=1
```

## 6. main.c 接入

将原编码器测试替换为舵机测试。

### Includes

```c
/* USER CODE BEGIN Includes */
#include "bsp_debug_uart.h"
#include "test_servo.h"
/* USER CODE END Includes */
```

### 初始化

```c
/* USER CODE BEGIN 2 */
if (!Test_Servo_Init())
{
    Error_Handler();
}
/* USER CODE END 2 */
```

### 主循环

```c
/* USER CODE BEGIN 3 */
Test_Servo_Update();
/* USER CODE END 3 */
```

保留现有 UART DMA 回调：

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

## 7. Keil 工程

在 `User/bsp` 分组加入：

```text
bsp_servo.c
```

在 `User/test` 分组加入：

```text
test_servo.c
```

现有 Include Paths 已包含：

```text
../User/bsp
../User/test
```

## 8. 安全注意事项

1. 舵机不得使用 STM32 的 3.3 V 引脚供电。
2. 舵机 5 V 电源应有足够电流能力。
3. 舵机电源地必须与 STM32 GND 共地。
4. 第一次测试先拆开连杆，或确保摆杆不会撞限位。
5. 当前测试只在 1450～1550 us 范围运动。
6. 确认方向和机构安全后，再逐步扩大可用脉宽范围。
