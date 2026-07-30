# CubeMX按键修正：KEY0=PE4，KEY_UP=PA0

适配基线：

```text
follow
643d2126f0d859bc5d6ef4cfbb27f14e7ec3a89e
```

## 1. 删除错误的PG15按键

在Pinout中找到PG15：

```text
PG15 / START_KEY / GPIO_EXTI15
```

右键选择：

```text
Reset_State
```

随后进入：

```text
System Core -> NVIC
```

确认不再启用：

```text
EXTI line[15:10] interrupts
```

## 2. 配置KEY_UP

在芯片图中点击PA0：

```text
GPIO_Input
```

在GPIO设置中配置：

```text
User Label = KEY_UP
GPIO mode = Input mode
Pull-up/Pull-down = Pull-down
```

KEY_UP按下时为高电平。

## 3. 配置KEY0

在芯片图中点击PE4：

```text
GPIO_Input
```

在GPIO设置中配置：

```text
User Label = KEY0
GPIO mode = Input mode
Pull-up/Pull-down = Pull-up
```

KEY0按下时为低电平。

## 4. 不配置按键中断

本版本通过主循环轮询按键并做30 ms非阻塞消抖，因此：

```text
不要将PA0配置为GPIO_EXTI0
不要将PE4配置为GPIO_EXTI4
不要启用EXTI0_IRQn
不要启用EXTI4_IRQn
```

## 5. 生成后检查

`Core/Inc/main.h`应出现：

```c
#define KEY_UP_Pin GPIO_PIN_0
#define KEY_UP_GPIO_Port GPIOA

#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
```

`Core/Src/gpio.c`应出现类似：

```c
GPIO_InitStruct.Pin = KEY_UP_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLDOWN;
HAL_GPIO_Init(KEY_UP_GPIO_Port, &GPIO_InitStruct);

GPIO_InitStruct.Pin = KEY0_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStruct);
```

不应再出现：

```c
START_KEY_Pin
START_KEY_GPIO_Port
HAL_NVIC_EnableIRQ(EXTI15_10_IRQn)
```

## 6. 当前代码用途

```text
KEY0 / PE4：
第一次按下开始巡线
第二次按下停止巡线
自动停车后再次按下重新开始

KEY_UP / PA0：
当前版本保留，暂不使用
```
