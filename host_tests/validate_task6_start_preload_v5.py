#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
main = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
cfg = (ROOT / 'User/action/task6_lap_target_config.h').read_text(encoding='utf-8')
t6 = (ROOT / 'User/action/task6_lap_target.c').read_text(encoding='utf-8')
t2 = (ROOT / 'User/action/task2_lap_stop_config.h').read_text(encoding='utf-8')


def macro(name: str, text: str) -> int:
    m = re.search(rf'^#define\s+{re.escape(name)}\s+\(\((?:u?int32_t)\)(-?\d+)U?\)\s*$', text, re.M)
    if not m:
        raise AssertionError(f'missing/unparseable macro: {name}')
    return int(m.group(1))

assert macro('TASK6_START_HOLD_MS', cfg) == 120
assert macro('TASK6_START_RAMP_MS', cfg) == 1400
assert macro('TASK6_START_CENTER_SPEED_MM_S', cfg) == 120
assert macro('TASK6_START_MIN_SPEED_MM_S', cfg) == 100

assert macro('T4_BALL_START_HOLD_PRELOAD_US', main) == 70
assert macro('T4_BALL_START_HOLD_FF_STEP_US', main) == 18
assert macro('T4_BALL_START_RAMP_FF_MIN_SCALE_MILLI', main) == 650
assert macro('T4_BALL_START_RAMP_FF_LIMIT_US', main) == 120
assert macro('T4_BALL_START_RAMP_FF_STEP_US', main) == 24
assert macro('T4_BALL_START_RAMP_PRELOAD_FULL_US', main) == 90
assert macro('T4_BALL_START_RAMP_PRELOAD_FULL_END_MILLI', main) == 250
assert macro('T4_BALL_START_RAMP_PRELOAD_RELEASE_MILLI', main) == 850
assert macro('T4_BALL_START_HOLD_SLEW_US', main) == 18
assert macro('T4_BALL_START_RAMP_SLEW_US', main) == 24
assert macro('T4_BALL_START_RAMP_RECOVER_SLEW_US', main) == 64

assert 'feedforward_target_us = -T4_BALL_START_HOLD_PRELOAD_US;' in main
assert 'feedforward_target_us > -transient_preload_us' in main
assert 'START_PRELOAD=%lu' in t6
assert 'PRELOAD=%lu,RAMP=%lu' in t6

# Parking transient remains V4.
assert macro('TASK6_STOP_APPROACH_CENTER_SPEED_MM_S', cfg) == 280
assert macro('TASK6_STOP_FINAL_CENTER_SPEED_MM_S', cfg) == 180
assert macro('TASK6_STOP_DECEL_MM_S2', cfg) == 180
assert macro('TASK6_STOP_JERK_MM_S3', cfg) == 800
assert macro('TASK6_STOP_SETTLE_MS', cfg) == 400

# Task 2 final approach remains 100 mm.
assert '((uint32_t)100U)' in t2

full = macro('T4_BALL_START_RAMP_PRELOAD_FULL_US', main)
full_end = macro('T4_BALL_START_RAMP_PRELOAD_FULL_END_MILLI', main)
release = macro('T4_BALL_START_RAMP_PRELOAD_RELEASE_MILLI', main)

def preload(p: int) -> int:
    if p <= full_end:
        return full
    if p < release:
        return round(full * (release - p) / (release - full_end))
    return 0

expected = {0: 90, 250: 90, 420: 64, 562: 43, 705: 22, 850: 0, 1000: 0}
actual = {p: preload(p) for p in expected}
for p, target in expected.items():
    assert abs(actual[p] - target) <= 1, (p, actual[p], target)

print('TASK6 START PRELOAD V5 VALIDATION PASS')
print('startup preload profile:', actual)
print('parking transient unchanged; Task2 final approach remains 100 mm')
