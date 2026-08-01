from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "User/action/task6_lap_target_config.h").read_text(
    encoding="utf-8"
)
TASK6 = (ROOT / "User/action/task6_lap_target.c").read_text(encoding="utf-8")


def macro(name: str) -> int:
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+\(\(int32_t\)(-?\d+)\)",
        CONFIG,
    )
    if not match:
        raise AssertionError(f"missing/unparseable macro: {name}")
    return int(match.group(1))


center = macro("TASK6_CRUISE_CENTER_SPEED_MM_S")
minimum = macro("TASK6_CRUISE_MIN_SPEED_MM_S")

# In normal line following the base-speed formula spans [minimum, center]
# according to line error. Equality is therefore the explicit guarantee that
# entering/exiting a bend cannot command longitudinal deceleration/acceleration.
assert center == 260
assert minimum == center, (
    f"curve-dependent base speed remains enabled: {center} -> {minimum} mm/s"
)
assert any(
    marker in TASK6
    for marker in (
        "CTRL=T6_CONSTANT_CRUISE_V11",
        "CTRL=T6_LOST_HOLD_V12",
    )
)

# Preserve the already validated gentle launch and ball/chassis decoupling.
assert "#define TASK6_START_RAMP_MS                   ((uint32_t)10000U)" in CONFIG
assert "#define TASK6_START_RAMP_MAX_MS               ((uint32_t)12000U)" in CONFIG
assert "Task6_UpdateStartSpeedGuard" not in TASK6

print("TASK6 CONSTANT CRUISE V11 VALIDATION PASS")
print("normal-run mean wheel speed is fixed at 260 mm/s")
