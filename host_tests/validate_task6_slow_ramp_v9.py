#!/usr/bin/env python3
"""Static contract for the Task 6 10-second time-only startup ramp V9."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
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


assert macro("TASK6_START_PRELOAD_MIN_MS", CONFIG) == 300
assert macro("TASK6_START_PRELOAD_TIMEOUT_MS", CONFIG) == 700
assert macro("TASK6_START_RAMP_MS", CONFIG) == 10000
assert macro("TASK6_START_RAMP_MAX_MS", CONFIG) == 12000
assert macro("TASK6_START_CENTER_SPEED_MM_S", CONFIG) == 60
assert macro("TASK6_START_MIN_SPEED_MM_S", CONFIG) == 50
assert "TASK6_START_GUARD_" not in CONFIG

speed_ramp = re.search(
    r"static bool Task6_UpdateStartSpeedRamp\(uint32_t now_ms\)\s*"
    r"\{(?P<body>.*?)\n\}",
    TASK6,
    re.DOTALL,
)
assert speed_ramp, "time-only startup ramp function not found"
ramp_body = speed_ramp.group("body")
assert "TASK6_START_RAMP_MS" in ramp_body
assert "s_ball.error_mm" not in ramp_body
assert "s_ball.filtered_velocity_px_s" not in ramp_body

assert "CTRL=T6_SLOW_RAMP_V9" in TASK6
assert "COUPLED=0" in TASK6
assert "CHASE=1" in TASK6
assert "SLOW_RAMP" in ANALYZER

# Even the deliberately pessimistic assumption that every added ramp
# millisecond adds one full millisecond to the slowest measured V8 lap remains
# below the 30 s rule: 18.709 s + (10.000 - 2.200) s = 26.509 s.
slowest_v8_lap_ms = 18709
added_ramp_ms = macro("TASK6_START_RAMP_MS", CONFIG) - 2200
assert slowest_v8_lap_ms + added_ramp_ms == 26509
assert slowest_v8_lap_ms + added_ramp_ms < 30000

print("TASK6 SLOW RAMP V9 VALIDATION PASS")
print("time-only startup ramp: 10000 ms, forced exit: 12000 ms")
print("conservative lap budget: 26509 ms < 30000 ms")
