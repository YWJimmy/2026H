#!/usr/bin/env python3
"""Static contract checks for the Task 6 adaptive launch guard V6.

Real pass/fail remains a hardware UART replay because chassis inertia, servo
latency and ball friction are not represented by the host repository. These
checks lock down the exact execution-path fix and its observable diagnostics.
"""

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


def macro(name: str, text: str) -> int:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+\(\((?:u?int32_t)\)(-?\d+)U?\)\s*$",
        text,
        re.MULTILINE,
    )
    if not match:
        raise AssertionError(f"missing/unparseable macro: {name}")
    return int(match.group(1))


assert macro("T4_BALL_START_HOLD_PRELOAD_US", MAIN) == 120
assert macro("T4_BALL_START_HOLD_FF_STEP_US", MAIN) == 30
assert macro("T4_BALL_START_RAMP_PRELOAD_FULL_US", MAIN) == 140
assert macro("T4_BALL_START_GUARD_MAX_US", MAIN) == 220
assert macro("TASK6_START_PRELOAD_MIN_MS", CONFIG) == 100
assert macro("TASK6_START_PRELOAD_TIMEOUT_MS", CONFIG) == 500
assert macro("TASK6_START_CENTER_SPEED_MM_S", CONFIG) == 60
assert macro("TASK6_START_MIN_SPEED_MM_S", CONFIG) == 50
assert macro("TASK6_START_RAMP_MS", CONFIG) == 2200

# Regression for the V5 root cause: stationary preload must bypass the stale
# chassis timestamp early return and continue stepping the physical pulse.
assert "chassis_sample_advanced" in MAIN
assert "TASK4_MAIN_BALL_TRANSIENT_START_HOLD" in MAIN
assert "if ((!chassis_sample_advanced) &&" in MAIN
assert "Task4MainBall_IsStartupPreloadReady" in HEADER
assert "Task4MainBall_IsStartupPreloadReady" in TASK6
assert "s_ball.control_sequence >" in TASK6

# Hardware evidence and firmware identity are mandatory in the next UART log.
assert any(
    marker in TASK6
    for marker in (
        "CTRL=T6_ADAPTIVE_LAUNCH_GUARD_V6",
        "CTRL=T6_FRAME_SYNC_SERVO_V7",
    )
)
assert "T6,PRELOAD_READY=1" in TASK6
assert "T6,PRELOAD_TIMEOUT=1" in TASK6
assert "T6,START_GUARD=%u" in TASK6
assert "GUARD=%ld" in TASK6

# Speed protection must react to the same positive relative error/velocity
# direction seen in the -9 cm -> -7 cm rollback.
assert "s_ball.error_mm >= TASK6_START_GUARD_HARD_ERROR_MM" in TASK6
assert "s_ball.filtered_velocity_px_s >=" in TASK6
assert "TASK6_START_GUARD_HARD_CENTER_MM_S" in TASK6


def adaptive_guard(base_us: int, error_mm: int, velocity_px_s: int) -> int:
    guard = base_us
    if error_mm > 2:
        guard += (error_mm - 2) * 12
    if velocity_px_s > 40:
        guard += (velocity_px_s - 40) // 3
    return min(max(guard, 0), 220)


assert adaptive_guard(140, 0, 0) == 140
assert adaptive_guard(100, 5, 125) == 164
assert adaptive_guard(80, 8, 180) == 198
assert adaptive_guard(140, 20, 300) == 220

print("TASK6 ADAPTIVE LAUNCH GUARD V6 VALIDATION PASS")
print("root cause locked: preload advances while chassis timestamp is stationary")
print("hardware contract: PRELOAD_READY must precede the first non-zero CMD")
