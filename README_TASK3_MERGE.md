# 任务3融合工程：0ebde21 + 7a35ce0 + 按键菜单

## 基线与来源

- 完整工程基线：`0ebde21cefbcc9e61478d0c2ece279020827bf6e`
- 任务3控制来源：`7a35ce0d04769a6a9d843d9d9decdd1c5e9b9983`
- GitHub 对比显示两者之间只有 `User/test/test_ball_balance.c` 发生变化。
- 已核对 `7a35ce0` 原始任务3文件的 Git blob：`a433865c37042834adb9dcfe524d0839c3e02400`。

## 融合原则

1. 保留 `0ebde21` 的全部工程、巡线、底盘、OLED、按键以及 `Vision/vision_protocol` 数据链。
2. 任务3采用 `7a35ce0` 的控制参数和状态逻辑：
   - 目标：`653 -> 869 -> 447`（O -> +5 cm -> -5 cm）
   - +5 cm 到达后立即折返
   - -5 cm 使用双倍提前制动增益
   - 推动脉宽限制为 1550~1750 us
   - 刹车脉宽允许 1300~2000 us
   - 死区 40 px，卡滞 15 帧后连续 8 帧增加 100 us
3. 不引入 `7a35ce0` 分支的旧 UART 检测接口，而是把同一控制算法接入 `0ebde21` 的 `VisionStatus_t`，因此保留物理坐标换算、协议校验和 ROI 数据链。
4. 任务3只由菜单启动，不再上电直接运行。

## 按键操作

1. 上电进入菜单，默认选中任务2。
2. 按一次 `KEY_UP / PA0`，选中 `3 BALL SEQ`。
3. 按一次 `KEY0 / PE4`，进入 `ARMED`。
4. 再按一次 `KEY0 / PE4`，启动任务3。
5. 运行中按 `KEY0 / PE4`，停止任务并关闭舵机 PWM，进入 `DONE`。
6. 在 `DONE` 页面按 `KEY0` 返回菜单。

启动成功时串口应出现：

```text
UI,EVENT=TASK_STARTED,TASK=3,CTRL=7A35CE0_PORT
BALL_BAL,START,MODE=CYCLE_0_+5_-5_0,...
```

## 修改文件

- `User/test/test_ball_balance.c`
- `User/test/test_oled_key.c`
- `User/module/task_menu_ui.c`

`test_config.h` 在 `0ebde21` 中已经默认使用 `TEST_MODE_OLED_KEY`，无需修改。

## 已完成验证

- `7a35ce0` 原始控制文件由 `858de92` 加官方提交差异重建，Git blob 完全匹配 `a433865...`。
- 适配后的任务3通过主机 C99 回归：O 保持 -> +5 -> 立即 -5 -> 保持 -> O。
- 菜单调度与 OLED 菜单文件通过 `-Wall -Wextra -Werror` 类型/语法检查。
- Keil 工程中 65 个源码路径全部存在，8 个头文件包含目录全部存在。

当前 Linux 环境无法运行 Keil ARMCC。下载后请打开 `MDK-ARM/2026H.uvprojx`，执行 `Project -> Rebuild all target files`。
