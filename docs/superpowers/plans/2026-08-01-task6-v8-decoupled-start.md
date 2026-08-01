# Task6 V8 Decoupled Start Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不引入球位置到底盘速度耦合的前提下建立可执行的温和预载，使 Task6 至少恢复到 V4 实车稳定性，并为后续压入全程 +/-10 mm 建立可信日志闭环。

**Architecture:** 以 V4 的视觉闭环推进、启动增益和舵机斜率为稳定基线；仅保留 V6 已验证的静止底盘时间戳旁路，使 70 us 预载能真实执行。底盘启动速度只按时间连续上升，禁止根据瞬时球误差或速度跳变，从而切断“球运动 -> 底盘加减速 -> 球惯性”的耦合回路。

**Tech Stack:** STM32F407 bare-metal C、Keil MDK-ARM、Python 3.12 主机契约测试、Task6 UART 实车回放。

---

### Task 1: 锁定 V8 源码契约

**Files:**
- Create: `host_tests/validate_task6_decoupled_start_v8.py`
- Test: `host_tests/compare_task6_logs.py`

- [ ] **Step 1: 写入当前源码必然失败的 V8 契约测试**

测试必须断言：

```python
assert "CTRL=T6_DECOUPLED_START_V8" in task6
assert "Task6_UpdateStartSpeedRamp" in task6
assert "Task6_UpdateStartSpeedGuard" not in task6
assert "TASK6_START_GUARD_HARD_ERROR_MM" not in config
assert macro("T4_BALL_START_HOLD_PRELOAD_US", main) == 70
assert macro("T4_BALL_START_RAMP_FF_LIMIT_US", main) == 120
assert macro("T4_BALL_START_RAMP_PRELOAD_FULL_US", main) == 90
assert "T4Ball_ApplyFeedforwardIncrement" not in main
```

- [ ] **Step 2: 运行测试并确认红测**

Run:

```powershell
python host_tests\validate_task6_decoupled_start_v8.py
```

Expected: `AssertionError`，因为当前源码仍是 `T6_FRAME_SYNC_SERVO_V7`。

- [ ] **Step 3: 保留真实日志差分红测**

Run:

```powershell
python host_tests\compare_task6_logs.py `
  D:\xwechat_files\wxid_r7axuynpekjx22_89e4\msg\file\2026-08\test_task6_20260801_1422_单个片段(1).txt `
  D:\xwechat_files\wxid_r7axuynpekjx22_89e4\msg\file\2026-08\test_task6_20260801_1542.txt `
  --require-not-worse
```

Expected: `RESULT=WORSE`，证明 V7 相对 V4 的实车回归仍可复现。该命令只能由新的 V8 实车日志变绿，静态测试不能替代它。

### Task 2: 恢复闭环带宽并降低启动控制激进度

**Files:**
- Modify: `Core/Src/main.c`
- Test: `host_tests/validate_task6_decoupled_start_v8.py`

- [ ] **Step 1: 恢复 V4 启动瞬态控制尺度**

将启动位置、阻尼和单次斜率恢复为：

```c
#define T4_BALL_START_HOLD_POSITION_SCALE_MILLI     ((int32_t)700)
#define T4_BALL_START_HOLD_DAMP_SCALE_MILLI         ((int32_t)1000)
#define T4_BALL_START_RAMP_POSITION_SCALE_MILLI     ((int32_t)750)
#define T4_BALL_START_RAMP_DAMP_SCALE_MILLI         ((int32_t)1150)
#define T4_BALL_START_HOLD_SLEW_US                  ((int32_t)10)
#define T4_BALL_START_RAMP_SLEW_US                  ((int32_t)16)
#define T4_BALL_START_RAMP_RECOVER_SLEW_US          ((int32_t)42)
```

- [ ] **Step 2: 使用 V5 温和预载值和 V6 可执行时间戳修复**

```c
#define T4_BALL_START_HOLD_PRELOAD_US               ((int32_t)70)
#define T4_BALL_START_HOLD_FF_LIMIT_US              ((int32_t)90)
#define T4_BALL_START_HOLD_FF_STEP_US               ((int32_t)18)
#define T4_BALL_START_RAMP_FF_MIN_SCALE_MILLI       ((int32_t)650)
#define T4_BALL_START_RAMP_FF_LIMIT_US              ((int32_t)120)
#define T4_BALL_START_RAMP_FF_STEP_US               ((int32_t)24)
#define T4_BALL_START_RAMP_PRELOAD_FULL_US          ((int32_t)90)
```

删除按误差和速度把前馈提升至 220 us 的自适应 guard 计算；启动斜坡只保留按时间平滑释放的 90 us 预载。保留 `START_HOLD` 对静止底盘时间戳的旁路。

- [ ] **Step 3: 撤销 V7 帧同步执行路径**

删除 `s_t4_ball_applied_feedforward_us` 和 `T4Ball_ApplyFeedforwardIncrement()`。同一视觉序列但底盘状态更新时，恢复对完整目标的受限推进：

```c
desired_delta_us =
    s_t4_ball_status.learned_bias_us +
    s_t4_ball_status.acceleration_feedforward_us +
    s_t4_ball_status.position_control_us +
    s_t4_ball_status.damping_control_us;
return T4Ball_WritePulse(
    T4_BALL_SERVO_CENTER_US + desired_delta_us,
    maximum_step_us);
```

### Task 3: 将底盘启动改成纯时间连续斜坡

**Files:**
- Modify: `User/action/task6_lap_target.c`
- Modify: `User/action/task6_lap_target_config.h`
- Modify: `host_tests/analyze_task6_log.py`

- [ ] **Step 1: 删除反应式速度门控配置**

删除 `TASK6_START_GUARD_SOFT_*`、`TASK6_START_GUARD_HARD_*`。设置：

```c
#define TASK6_START_PRELOAD_MIN_MS            ((uint32_t)300U)
#define TASK6_START_PRELOAD_TIMEOUT_MS        ((uint32_t)700U)
#define TASK6_START_RAMP_MS                   ((uint32_t)2200U)
#define TASK6_START_RAMP_MAX_MS               ((uint32_t)5000U)
#define TASK6_START_CENTER_SPEED_MM_S         ((int32_t)60)
#define TASK6_START_MIN_SPEED_MM_S            ((int32_t)50)
```

- [ ] **Step 2: 用纯时间函数替换速度 guard**

```c
static bool Task6_UpdateStartSpeedRamp(uint32_t now_ms)
{
    int32_t progress_milli = Task6_ProgressMilli(
        now_ms - s_start_ramp_ms,
        TASK6_START_RAMP_MS);
    int32_t center_speed_mm_s = Task6_LerpI32(
        TASK6_START_CENTER_SPEED_MM_S,
        LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,
        progress_milli);
    int32_t minimum_speed_mm_s = Task6_LerpI32(
        TASK6_START_MIN_SPEED_MM_S,
        LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S,
        progress_milli);
    return LineFollowControl_SetBaseSpeedRangeMmps(
        center_speed_mm_s,
        minimum_speed_mm_s);
}
```

- [ ] **Step 3: 更新固件身份和诊断**

启动标识改为 `CTRL=T6_DECOUPLED_START_V8`。`T6GAIN` 输出 `PRE=<us>,COUPLED=0`；`T6CTRL` 输出 `CHASE=1`，确保下一份日志能证明运行的是 V8。

### Task 4: 主机验证与 Keil 构建

**Files:**
- Modify: `report/TASK6_DECOUPLED_START_V8_REPORT_CN.txt`
- Output: `MDK-ARM/2026H/2026H.hex`

- [ ] **Step 1: 运行 V8、V6 时间戳和日志分析回归**

Run:

```powershell
python host_tests\validate_task6_decoupled_start_v8.py
python host_tests\test_task6_log_analyzer.py
```

Expected: 两项均 PASS；V8 测试同时确认静止预载时间戳旁路仍在。

- [ ] **Step 2: Keil 全量重编译**

Run:

```powershell
& D:\Keil_v5\UV4\UV4.exe -r MDK-ARM\2026H.uvprojx -j0
```

Expected: `0 Error(s)`；记录程序体积、唯一/全部警告、HEX 时间戳和 SHA256。

- [ ] **Step 3: 写入实车验收边界**

报告必须明确：主机和构建通过不等于实车通过。下一份日志需包含 `T6_DECOUPLED_START_V8`，并用 `compare_task6_logs.py` 对 V4 基线验证 `RESULT=NOT_WORSE`，最终还需 `MAX_ERR_MM<=10`、`VIOL=0`、圈时不超过 30 s。

### Self-review

- [x] 覆盖了 V7 的三项已证实回归：帧同步带宽损失、220 us 强前馈、球误差驱动底盘速度跳变。
- [x] 保留了唯一独立且有执行路径证据的 V6 修复：静止底盘时间戳下预载继续推进。
- [x] 未改停车、巡线、A 点检测、圈时锁存和 Task2 配置。
- [x] 所有固件参数均给出确切值，没有占位符。
- [x] 当前目录的 `.git` 元数据无效，因此计划不包含不可执行的 commit 步骤。
