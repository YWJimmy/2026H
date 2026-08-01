# CubeMX配置：OLED I2C1 400 kHz中断模式

适配基线：

```text
71b5a7d1e0bcc058727a274625c27db0d91672b5
```

参考OLED：

```text
SSD1306
128x64
4针I2C
7位地址0x3C
HAL地址0x78
```

## 1. I2C1参数

进入：

```text
Connectivity
→ I2C1
→ Parameter Settings
```

配置：

```text
I2C Speed Mode = Fast Mode
Clock Speed = 400000 Hz
Fast Mode Duty Cycle = Tlow/Thigh = 2
Addressing Mode = 7-bit
Dual Address = Disable
General Call = Disable
Clock Stretching = Enabled
```

保持引脚：

```text
PB6 = I2C1_SCL，User Label = OLED_SCL
PB7 = I2C1_SDA，User Label = OLED_SDA
```

GPIO建议：

```text
Mode = Alternate Function Open Drain
Pull = No Pull
Speed = Very High
AF = AF4 I2C1
```

OLED模块板上应有SCL/SDA上拉。若实物没有上拉，需要外接约4.7 kΩ到3.3 V。

## 2. I2C1中断

进入：

```text
I2C1
→ NVIC Settings
```

勾选：

```text
I2C1 event interrupt
I2C1 error interrupt
```

建议优先级：

```text
I2C1_EV_IRQn：Preemption 3，Sub 0
I2C1_ER_IRQn：Preemption 3，Sub 0
```

本版本不使用I2C1 DMA。

## 3. 按键配置

当前提交已经正确配置：

```text
KEY_UP = PA0，GPIO_Input，Pull-down，高电平按下
KEY0   = PE4，GPIO_Input，Pull-down，高电平按下
```

不需要启用EXTI。按键使用主循环轮询和30 ms非阻塞消抖。

## 4. 生成代码后检查

`Core/Src/i2c.c`：

```c
hi2c1.Init.ClockSpeed = 400000;
```

`Core/Src/stm32f4xx_it.c`：

```c
extern I2C_HandleTypeDef hi2c1;

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}
```

`Core/Inc/stm32f4xx_it.h`：

```c
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
```

`Core/Src/i2c.c`的I2C1 MSP初始化应包含：

```c
HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3, 0);
HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
```
