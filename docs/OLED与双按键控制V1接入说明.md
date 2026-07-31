# OLED与双按键控制 V1 接入说明

## 一、范围

本版本只完成：

```text
SSD1306 OLED硬件I2C1驱动
OLED掉线检测、总线恢复和自动重连
KEY_UP选择键
KEY0确认键
任务2～6菜单
二次确认启动流程
OLED/按键专项测试
```

不启动：

```text
底盘
巡线
舵机
滚球
正式任务状态机
```

## 二、文件

新增：

```text
User/bsp/bsp_oled.c
User/bsp/bsp_oled.h
User/bsp/bsp_key.c
User/bsp/bsp_key.h

User/module/task_menu_ui.c
User/module/task_menu_ui.h

User/test/test_oled_key.c
User/test/test_oled_key.h
```

替换：

```text
User/test/test_config.h
User/test/test_runner.c
```

## 三、main.c回调

在USER CODE Includes增加：

```c
#include "bsp_oled.h"
```

在USER CODE 4增加：

```c
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BSP_Oled_TxCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    BSP_Oled_ErrorCallback(hi2c);
}
```

BSP内部只接受I2C1，不会把I2C2的MPU6050错误当成OLED错误。

## 四、菜单操作

### SELECT页

```text
KEY_UP短按：
2 → 3 → 4 → 5 → 6 → 2

KEY0短按：
进入ARMED
```

没有KEY_UP长按功能。

### ARMED页

```text
KEY0短按：
产生启动请求并进入RUNNING占位页

KEY_UP短按：
返回SELECT
```

### RUNNING占位页

当前没有正式任务逻辑：

```text
KEY0短按：
产生停止请求并进入FINISHED
```

### FINISHED页

```text
KEY0短按：
返回SELECT
```

## 五、OLED掉线恢复

在线时：

```text
每秒重发第0页作为心跳
```

传输发生NACK、总线错误或超过20 ms未完成：

```text
标记离线
全部页面保持dirty
等待1秒
关闭I2C1
PB6输出9个恢复时钟
产生STOP
重新初始化I2C1
重发SSD1306初始化命令
重发完整1024字节帧缓冲区
```

拔掉OLED后，按键和菜单状态仍会继续运行；重新接上OLED后，屏幕会恢复当前页面。

## 六、测试模式

默认：

```c
#define PROJECT_TEST_MODE TEST_MODE_OLED_KEY
```

预期串口启动日志：

```text
TEST,OLED_KEY,START,OLED=SSD1306_128X64_ADDR_0X3C,I2C1=400KHZ_IT
KEY,SELECT=KEY_UP_PA0_HIGH,CONFIRM=KEY0_PE4_HIGH,LONG_PRESS=DISABLED
```

状态日志：

```text
UI,TASK=3,NAME=3 BALL SEQ,STATE=SELECT,OLED=1
OLED,ON=1,ST=ONLINE,DIRTY=0x00,ERR=0,DISC=0,RECON=0,RETRY=0,...
KEY,UP=0,K0=0,UP_CNT=1,K0_CNT=0
```

掉线后：

```text
OLED,ON=0,ST=RETRY,ERR=1,DISC=1,...
```

重新连接后：

```text
OLED,ON=1,ST=ONLINE,RECON=1,...
```
