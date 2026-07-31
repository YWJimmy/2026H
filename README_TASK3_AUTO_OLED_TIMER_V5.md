# Task3 V5：OLED 自动大数字计时

基于 `2026H_0eb_7a_Task3_AntiDriftHold_v4`，参考 Git 提交
`bf89793ccb1d823cf16cbe0898b5071ebbcc2df7` 加入 OLED 自动计时显示。

## 行为

- 第二次按 KEY0 后显示 `00.0`，等待 O 点准备。
- 控制器从 O 点真正开始向 +5 cm 运动时，计时自动启动。
- OLED 以 28x46 七段大数字实时显示 `秒.十分之一秒`。
- 钢球第一次在 -5 cm ±1 cm 范围内停住时，时间冻结并显示 PASS。
- PASS 后继续执行 V4 防滑保持，冻结成绩不会改变。
- K0 停止任务并返回菜单。

## 修改文件

- `User/bsp/bsp_oled.c`
- `User/bsp/bsp_oled.h`
- `User/test/test_ball_balance.c`
- `User/test/test_oled_key.c`
- `User/module/task_menu_ui.c`
- `User/module/task_menu_ui.h`

## 计时范围

`O 点实际出发 -> 首次合格停入 -5 cm ±1 cm`。
等待视觉识别及 O 点初始确认的时间不计入正式成绩。
