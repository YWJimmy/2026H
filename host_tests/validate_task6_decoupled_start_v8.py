#!/usr/bin/env python3
"""Static contract checks for Task 6 decoupled-start controller V8."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
HEADER = (ROOT / "Core/Inc/task4_main_ball.h").read_text(encoding="utf-8")
TASK6 = (ROOT / "User/action/task6_lap_target.c").read_text(encoding="utf-8")
CONFIG = (ROOT / "User/action/task6_lap_target_config.h").read_text(
    encoding="utf-8"
)
ANALYZER = (ROOT / "host_tests/analyze_task6_log.py").read_text(
    encoding="utf-8"
)


def macro(name: str, text: str) -> int:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+"
        rf"\(\((?:u?int32_t)\)(-?\d+)U?\)\s*$",
        text,
        re.MULTILINE,
    )
    if not match:
        raise AssertionError(f"missing/unparseable macro: {name}")
    return int(match.group(1))


# Stable V4 feedback scaling and slew limits.
assert macro("T4_BALL_START_HOLD_POSITION_SCALE_MILLI", MAIN) == 700
assert macro("T4_BALL_START_HOLD_DAMP_SCALE_MILLI", MAIN) == 1000
assert macro("T4_BALL_START_RAMP_POSITION_SCALE_MILLI", MAIN) == 750
assert macro("T4_BALL_START_RAMP_DAMP_SCALE_MILLI", MAIN) == 1150
assert macro("T4_BALL_START_HOLD_SLEW_US", MAIN) == 10
assert macro("T4_BALL_START_RAMP_SLEW_US", MAIN) == 16
assert macro("T4_BALL_START_RAMP_RECOVER_SLEW_US", MAIN) == 42

# Moderate active preload, with no adaptive escalation to 220 us.
assert macro("T4_BALL_START_HOLD_PRELOAD_US", MAIN) == 70
assert macro("T4_BALL_START_HOLD_FF_LIMIT_US", MAIN) == 90
assert macro("T4_BALL_START_HOLD_FF_STEP_US", MAIN) == 18
assert macro("T4_BALL_START_RAMP_FF_MIN_SCALE_MILLI", MAIN) == 650
assert macro("T4_BALL_START_RAMP_FF_LIMIT_US", MAIN) == 120
assert macro("T4_BALL_START_RAMP_FF_STEP_US", MAIN) == 24
assert macro("T4_BALL_START_RAMP_PRELOAD_FULL_US", MAIN) == 90
for removed in (
    "T4_BALL_START_GUARD_ERROR_START_MM",
    "T4_BALL_START_GUARD_ERROR_US_PER_MM",
    "T4_BALL_START_GUARD_VEL_START_PX_S",
    "T4_BALL_START_GUARD_VEL_DIVISOR",
    "T4_BALL_START_GUARD_MAX_US",
):
    assert removed not in MAIN

# Preserve the V6 execution-path repair for stationary physical preload.
assert "chassis_sample_advanced" in MAIN
assert "if ((!chassis_sample_advanced) &&" in MAIN
assert "TASK4_MAIN_BALL_TRANSIENT_START_HOLD" in MAIN
assert "Task4MainBall_IsStartupPreloadReady" in HEADER
assert "Task4MainBall_IsStartupPreloadReady" in TASK6

# Revert V7 frame-sync bandwidth restriction to the proven V4 chase path.
assert "s_t4_ball_applied_feedforward_us" not in MAIN
assert "T4Ball_ApplyFeedforwardIncrement" not in MAIN
normal_same_frame_matches = list(re.finditer(
    r"if \(vision_status->sequence == s_t4_ball_last_vision_sequence\)\s*"
    r"\{(?P<body>.*?)\n\s*\}\s*"
    r"s_t4_ball_last_vision_sequence = vision_status->sequence;",
    MAIN,
    re.DOTALL,
))
assert len(normal_same_frame_matches) >= 2, "normal same-frame branch not found"
same_frame_body = normal_same_frame_matches[-1].group("body")
assert "position_control_us" in same_frame_body
assert "damping_control_us" in same_frame_body
assert "T4Ball_WritePulse" in same_frame_body

# Chassis acceleration is a time-only profile; ball feedback cannot change it.
assert macro("TASK6_START_PRELOAD_MIN_MS", CONFIG) == 300
assert macro("TASK6_START_PRELOAD_TIMEOUT_MS", CONFIG) == 700
assert macro("TASK6_START_RAMP_MS", CONFIG) in (2200, 10000)
assert macro("TASK6_START_CENTER_SPEED_MM_S", CONFIG) == 60
assert macro("TASK6_START_MIN_SPEED_MM_S", CONFIG) == 50
assert "TASK6_START_GUARD_" not in CONFIG
assert "Task6_UpdateStartSpeedGuard" not in TASK6
assert "Task6_UpdateStartSpeedRamp" in TASK6
speed_ramp = re.search(
    r"static bool Task6_UpdateStartSpeedRamp\(uint32_t now_ms\)\s*"
    r"\{(?P<body>.*?)\n\}",
    TASK6,
    re.DOTALL,
)
assert speed_ramp, "time-only startup ramp function not found"
ramp_body = speed_ramp.group("body")
assert "s_ball.error_mm" not in ramp_body
assert "s_ball.filtered_velocity_px_s" not in ramp_body

# The next hardware log must prove both identity and decoupling.
assert any(
    marker in TASK6
    for marker in (
        "CTRL=T6_DECOUPLED_START_V8",
        "CTRL=T6_SLOW_RAMP_V9",
        "CTRL=T6_CURVE_SAFE_CRUISE_V10",
        "CTRL=T6_CONSTANT_CRUISE_V11",
        "CTRL=T6_LOST_HOLD_V12",
    )
)
assert "COUPLED=0" in TASK6
assert "CHASE=1" in TASK6
assert "DECOUPLED_START" in ANALYZER

print("TASK6 DECOUPLED START V8 VALIDATION PASS")
print("stationary preload: 70 us, executable with stale chassis timestamp")
print(
    "startup chassis profile: time-only 60/50 -> cruise over "
    f"{macro('TASK6_START_RAMP_MS', CONFIG)} ms"
)
print("ball feedback does not alter chassis speed during startup")
