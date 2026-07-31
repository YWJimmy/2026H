# 赛题要求 3：运行时间与停点精度优化版

基础工程：`0ebde21cefbcc9e61478d0c2ece279020827bf6e`

保留控制基线：`7a35ce0d04769a6a9d843d9d9decdd1c5e9b9983`

本版本针对实机日志继续优化，不再使用循环演示状态机。

## 实机日志暴露的问题

1. 原程序在 O 点与 -5 cm 各固定保持 5000 ms，任务天然不可能在 5 s 内完成。
2. +5 cm 进入 40 px 死区后立即切换，但反向制动太弱，钢球仍继续向右，峰值可超过 +6 cm。
3. 钢球以较高速度进入 -5 cm 死区时，程序立即将舵机置 1650 us；未消除的动能使钢球继续滚到约 -6.6 cm。
4. 过冲后，小误差修正不足，并等待 15 帧才进行一次卡滞增强，导致稳定时间很长。

## 新控制流程

```text
WAIT_O（最多 300 ms）
  -> TO_POS5
  -> TURN_BRAKE（强反向制动，直至右向速度归零）
  -> TO_NEG5（远处巡航，按速度预测提前制动）
  -> SETTLE_NEG5（先刹速度，再修位置）
  -> FINISHED（继续闭环保持 -5 cm，按 K0 后退出）
```

## 关键改动

- 去除两个 5 s 固定等待和循环回 O。
- 总运行硬限制为 5000 ms。
- +5 cm 到达窗口收紧到 30 px，随后立即输出 1420 us 反向制动。
- -5 cm 进入 38 px 捕获窗后，不再按位置死区直接回中。
- 捕获阶段首先根据速度方向刹车：
  - 向左滚动：脉宽提高，范围约 1770~1935 us；
  - 向右回弹：脉宽降低，范围约 1365~1530 us。
- 仅当 `|ERR| <= 20 px` 且 `|SPD| <= 2 px/frame` 连续 6 帧时判定成功。
- 低速但仍有静差时使用小幅位置修正；连续 5 帧不动才使用 3 帧有界 kick。
- 自然完成后 OLED 进入 DONE，仍保持 -5 cm；按 K0 后舵机回中并关闭，返回菜单。

## 按键启动

1. `KEY_UP` 选到 `3 BALL SEQ`。
2. 第一次 `KEY0` 进入 ARMED。
3. 第二次 `KEY0` 启动。
4. 运行中按 `KEY0` 人工停止。
5. 自然完成后继续保持 -5 cm；完成页按 `KEY0` 返回菜单。

## 新日志

启动：

```text
BALL_BAL,START,CTRL=ONE_SHOT_PREDICT_CAPTURE,...
```

状态变化：

```text
BALL_BAL,STATE_CHANGE,STATE=TO_POS5,...
BALL_BAL,STATE_CHANGE,STATE=TURN_BRAKE,...
BALL_BAL,STATE_CHANGE,STATE=TO_NEG5,...
BALL_BAL,STATE_CHANGE,STATE=SETTLE_NEG5,...
```

成功：

```text
BALL_BAL,RESULT=PASS,REASON=STABLE_NEG5,TOTAL_MS=...,
POS5_MS=...,FINAL_ERR_PX=...,POS5_OVER_PX=...,NEG5_OVER_PX=...
```

判据：

- `TOTAL_MS <= 5000`
- `FINAL_ERR_PX` 绝对值不大于 20 px
- `POS5_OVER_PX`、`NEG5_OVER_PX` 建议均不大于约 43 px（约 1 cm）

## 修改文件

```text
User/test/test_ball_balance.c
User/test/test_ball_balance.h
User/test/test_oled_key.c
User/module/task_menu_ui.c
host_tests/test_task3_optimized.c
```
