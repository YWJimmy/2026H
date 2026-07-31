# 任务3：首次合格停住计时 + 完成后防滑保持 v4

## 解决的问题

v3 在钢球第一次停入 -5 cm ±1 cm 后立即锁定 PASS 和任务用时，但同时把舵机固定为中位脉宽。若摆杆存在微小坡度，钢球会在完成后继续缓慢滑动，例如从 -5 cm 滑到 -3 cm。

v4 保留“第一次合格停住即停止计时”的规则，但不关闭控制：

- 第一次满足 `-60 mm <= X <= -40 mm`，且原始/滤波速度均不超过 2 px/帧时，立即锁定 PASS 和 `TOTAL_MS`；
- OLED 立刻显示该次任务用时，此后不会重新计时或撤销 PASS；
- 完成后进入低幅度 `FINISHED_ANTI_DRIFT` 保持；
- 根据漂移速度、位置偏差学习小范围静态保持偏置；
- 输出限制在中位附近 ±110 us，并限制每帧最大变化 18 us；
- ±5 mm 内且速度仅为 ±1 px/帧时不追逐视觉噪声；
- 按 K0 返回菜单时才回中、关闭舵机。

## OLED 完成页面

```text
TASK 3 FINISHED
RESULT: PASS
TIME MS: xxxx
NEG5 HOLD ACTIVE
K0:MENU
```

## 串口观察

完成瞬间：

```text
BALL_BAL,RESULT=PASS,REASON=FIRST_STOP_WITH_ANTI_DRIFT_HOLD,TOTAL_MS=...
```

完成后：

```text
STATE=FINISHED,MODE=FINISHED_ANTI_DRIFT,PULSE=...,HOLD_BIAS=...
```

若钢球向 -3 cm 方向滑动（`X_MM` 增大、`SPD` 为正），`HOLD_BIAS` 和脉宽应逐渐减小，推动钢球向 -5 cm 方向恢复。结果时间应保持不变。

## 主要修改文件

- `User/test/test_ball_balance.c`
- `User/module/task_menu_ui.c`
- `host_tests/test_task3_optimized.c`

## 使用

完整包解压后打开 `MDK-ARM/2026H.uvprojx`，执行 `Project -> Rebuild all target files`。
