# 任务3：首次停入 ±1 cm 即完成 + OLED显示用时（v3）

## 完成判据

最终目标为 `cx=447`、物理坐标 `-50 mm`。最终验收直接使用视觉协议的物理坐标，容许范围为 `-60 mm～-40 mm`；43 px只用于提前进入最终刹车阶段。

当钢球第一次同时满足以下条件时，当帧立即判定完成：

- `abs(physical_x_mm - (-50)) <= 10`
- `abs(filtered_speed) <= 2 px/frame`
- `abs(raw_speed) <= 2 px/frame`

完成后舵机固定为1650 us，不再根据后续视觉误差反复调整。

## OLED完成页

显示：

- RESULT: PASS / FAIL
- TIME MS: 实际任务用时
- PASS时显示 NEG5 IN +/-1CM
- K0:MENU

任务用时从按键启动任务3、控制器初始化时开始，到首次停入最终容许窗时结束。
