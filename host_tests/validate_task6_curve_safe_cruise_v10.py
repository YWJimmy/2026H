from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "User/action/task6_lap_target_config.h").read_text(
    encoding="utf-8"
)
TASK6 = (ROOT / "User/action/task6_lap_target.c").read_text(encoding="utf-8")


def macro(name: str, source: str) -> int:
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+\(\(int32_t\)(-?\d+)\)",
        source,
    )
    if not match:
        raise AssertionError(f"missing/unparseable macro: {name}")
    return int(match.group(1))


# V9 startup behavior stays unchanged.
assert "#define TASK6_START_RAMP_MS                   ((uint32_t)10000U)" in CONFIG
assert "#define TASK6_START_RAMP_MAX_MS               ((uint32_t)12000U)" in CONFIG
assert macro("TASK6_START_CENTER_SPEED_MM_S", CONFIG) == 60
assert macro("TASK6_START_MIN_SPEED_MM_S", CONFIG) == 50

# V10 uses a deterministic Task 6-only cruise cap; ball state must not alter it.
assert macro("TASK6_CRUISE_CENTER_SPEED_MM_S", CONFIG) == 260
assert macro("TASK6_CRUISE_MIN_SPEED_MM_S", CONFIG) == 200
assert "CTRL=T6_CURVE_SAFE_CRUISE_V10" in TASK6
assert "TASK6_CRUISE_CENTER_SPEED_MM_S" in TASK6
assert "TASK6_CRUISE_MIN_SPEED_MM_S" in TASK6
assert "Task6_UpdateStartSpeedGuard" not in TASK6

ramp_match = re.search(
    r"static bool Task6_UpdateStartSpeedRamp\(uint32_t now_ms\)"
    r"\s*\{(?P<body>.*?)\n\}",
    TASK6,
    re.DOTALL,
)
assert ramp_match, "time-only startup ramp function not found"
ramp_body = ramp_match.group("body")
assert "TASK6_CRUISE_CENTER_SPEED_MM_S" in ramp_body
assert "TASK6_CRUISE_MIN_SPEED_MM_S" in ramp_body
assert "s_ball.error_mm" not in ramp_body
assert "s_ball.filtered_velocity_px_s" not in ramp_body

# Parking must not command a post-line acceleration above the new cruise cap.
assert (
    "#define TASK6_STOP_APPROACH_CENTER_SPEED_MM_S "
    "TASK6_CRUISE_CENTER_SPEED_MM_S"
) in CONFIG
assert (
    "#define TASK6_STOP_APPROACH_MIN_SPEED_MM_S    "
    "TASK6_CRUISE_MIN_SPEED_MM_S"
) in CONFIG

# V9 measured 22.014 s at an effective post-ramp speed of about 338.5 mm/s.
# Replacing the remaining 3924 mm with a conservative 260 mm/s estimate gives
# about 25.5 s, leaving useful margin below the 30 s rule for the real vehicle.
v9_ramp_done_ms = 10422
v9_remaining_mm = 5880 - 1956
estimated_lap_ms = v9_ramp_done_ms + (
    v9_remaining_mm * 1000 // macro("TASK6_CRUISE_CENTER_SPEED_MM_S", CONFIG)
)
assert estimated_lap_ms == 25514
assert estimated_lap_ms < 30000

print("TASK6 CURVE-SAFE CRUISE V10 VALIDATION PASS")
print("Task 6 cruise: 260/200 mm/s, independent of ball feedback")
print(f"conservative V9-log lap estimate: {estimated_lap_ms / 1000:.3f} s")
