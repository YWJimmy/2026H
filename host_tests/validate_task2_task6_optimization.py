#!/usr/bin/env python3
"""Static and numeric checks for the Task 2 / Task 6 optimization patch."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
errors: list[str] = []


def read_utf8_no_bom(relative: str) -> str:
    path = ROOT / relative
    if not path.is_file():
        errors.append(f"missing file: {relative}")
        return ""
    data = path.read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        errors.append(f"unexpected UTF-8 BOM: {relative}")
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError as exc:
        errors.append(f"not UTF-8: {relative}: {exc}")
        return ""


def require(text: str, token: str, label: str) -> None:
    if token not in text:
        errors.append(f"{label}: missing {token}")


def macro_i32(text: str, name: str) -> int:
    pattern = rf"^#define\s+{re.escape(name)}\s+\(\(int32_t\)(-?\d+)\)\s*$"
    match = re.search(pattern, text, re.MULTILINE)
    if not match:
        errors.append(f"cannot parse int32 macro: {name}")
        return 0
    return int(match.group(1))


def c_round_mul_div(value: int, numerator: int, denominator: int) -> int:
    product = value * numerator
    if product >= 0:
        product += denominator // 2
    else:
        product -= denominator // 2
    # C integer division truncates toward zero.
    return abs(product) // denominator * (1 if product >= 0 else -1)


def schedule_for(target_mm: int, start_mm: int, full_mm: int) -> int:
    offset = abs(target_mm)
    if offset <= start_mm:
        return 0
    if offset >= full_mm:
        return 1000
    return c_round_mul_div(offset - start_mm, 1000, full_mm - start_mm)


def scheduled(center: int, offcenter: int, schedule: int) -> int:
    return center + c_round_mul_div(offcenter - center, schedule, 1000)


t2_cfg = read_utf8_no_bom("User/action/task2_lap_stop_config.h")
t2_src = read_utf8_no_bom("User/action/task2_lap_stop.c")
main_src = read_utf8_no_bom("Core/Src/main.c")
ball_hdr = read_utf8_no_bom("Core/Inc/task4_main_ball.h")
t6_src = read_utf8_no_bom("User/action/task6_lap_target.c")

for token, label in [
    ("TASK2_CRUISE_CENTER_SPEED_MM_S         ((int32_t)390)", "Task2 cruise center"),
    ("TASK2_CRUISE_MIN_SPEED_MM_S            ((int32_t)320)", "Task2 cruise minimum"),
    ("TASK2_PREDECEL_DISTANCE_MM             ((uint32_t)5600U)", "Task2 5.6 m gate"),
    ("TASK2_PREDECEL_CENTER_SPEED_MM_S       ((int32_t)260)", "Task2 search speed"),
    ("TASK2_RUN_TIMEOUT_MS                   ((uint32_t)20000U)", "Task2 timeout unchanged"),
]:
    require(t2_cfg, token, label)

require(t2_src, "TASK2_CRUISE_CENTER_SPEED_MM_S,", "Task2 uses task-specific cruise")
require(t2_src, "TASK2_CRUISE_MIN_SPEED_MM_S))", "Task2 uses task-specific minimum")
require(t2_src, "s_distance_snapshot.traveled_mm >=\n         TASK2_PREDECEL_DISTANCE_MM", "Task2 gate in state machine")
require(t2_src, "s_line_result_snapshot.black_count >= 3U", "Task2 stop-line rule retained")

for token, label in [
    ("T4_BALL_TARGET_SCHEDULE_START_MM", "schedule start"),
    ("T4_BALL_TARGET_SCHEDULE_FULL_MM", "schedule full"),
    ("T4_BALL_OFFCENTER_POSITION_SCALE_MILLI", "position scale"),
    ("T4_BALL_OFFCENTER_VELOCITY_SCALE_MILLI", "velocity scale"),
    ("T4_BALL_OFFCENTER_RAW_VEL_SCALE_MILLI", "raw velocity scale"),
    ("T4_BALL_OFFCENTER_PREDICT_SCALE_MILLI", "prediction scale"),
    ("T4_BALL_OFFCENTER_SLEW_SCALE_MILLI", "slew scale"),
    ("T4Ball_TargetScheduleMilli", "schedule helper"),
    ("T4Ball_ScheduledSlew", "scheduled slew helper"),
]:
    require(main_src, token, f"Task6 {label}")

for token in (
    "target_schedule_milli",
    "position_gain_milli",
    "velocity_gain_milli",
    "prediction_horizon_ms",
    "position_deadband_px",
):
    require(ball_hdr, token, f"Task6 diagnostic {token}")
require(t6_src, '"T6GAIN,SCH=%ld,PG=%ld,VG=%ld,PRED=%ld,DB=%ld', "Task6 serial diagnostics")

start_mm = macro_i32(main_src, "T4_BALL_TARGET_SCHEDULE_START_MM")
full_mm = macro_i32(main_src, "T4_BALL_TARGET_SCHEDULE_FULL_MM")
pos_scale = macro_i32(main_src, "T4_BALL_OFFCENTER_POSITION_SCALE_MILLI")
vel_scale = macro_i32(main_src, "T4_BALL_OFFCENTER_VELOCITY_SCALE_MILLI")
raw_scale = macro_i32(main_src, "T4_BALL_OFFCENTER_RAW_VEL_SCALE_MILLI")
predict_scale = macro_i32(main_src, "T4_BALL_OFFCENTER_PREDICT_SCALE_MILLI")
slew_scale = macro_i32(main_src, "T4_BALL_OFFCENTER_SLEW_SCALE_MILLI")
recover_slew_scale = macro_i32(main_src, "T4_BALL_OFFCENTER_RECOVER_SLEW_MILLI")
deadband_extra = macro_i32(main_src, "T4_BALL_OFFCENTER_DEADBAND_EXTRA_PX")
bias_extra = macro_i32(main_src, "T4_BALL_OFFCENTER_BIAS_DIVIDER_EXTRA")

center_schedule = schedule_for(0, start_mm, full_mm)
far_schedule = schedule_for(-60, start_mm, full_mm)
if center_schedule != 0:
    errors.append(f"0 mm schedule changed: {center_schedule}")
if far_schedule != 1000:
    errors.append(f"-60 mm does not select full profile: {far_schedule}")

# Verify the center profile is exactly the original controller and the -60 mm
# profile is deliberately more damped and rate-limited.
center_profile = {
    "position_scale": scheduled(1000, pos_scale, center_schedule),
    "velocity_normal": c_round_mul_div(400, scheduled(1000, vel_scale, center_schedule), 1000),
    "velocity_dynamic": c_round_mul_div(500, scheduled(1000, vel_scale, center_schedule), 1000),
    "raw_velocity": c_round_mul_div(100, scheduled(1000, raw_scale, center_schedule), 1000),
    "predict_normal": c_round_mul_div(90, scheduled(1000, predict_scale, center_schedule), 1000),
    "predict_dynamic": c_round_mul_div(130, scheduled(1000, predict_scale, center_schedule), 1000),
    "deadband": 3 + c_round_mul_div(deadband_extra, center_schedule, 1000),
    "bias_divider": 3 + c_round_mul_div(bias_extra, center_schedule, 1000),
    "slew_hold": c_round_mul_div(24, scheduled(1000, slew_scale, center_schedule), 1000),
    "slew_normal": c_round_mul_div(34, scheduled(1000, slew_scale, center_schedule), 1000),
    "slew_fast": c_round_mul_div(62, scheduled(1000, slew_scale, center_schedule), 1000),
    "slew_recover": c_round_mul_div(100, scheduled(1000, recover_slew_scale, center_schedule), 1000),
}
expected_center = {
    "position_scale": 1000,
    "velocity_normal": 400,
    "velocity_dynamic": 500,
    "raw_velocity": 100,
    "predict_normal": 90,
    "predict_dynamic": 130,
    "deadband": 3,
    "bias_divider": 3,
    "slew_hold": 24,
    "slew_normal": 34,
    "slew_fast": 62,
    "slew_recover": 100,
}
if center_profile != expected_center:
    errors.append(f"center profile is not preserved: {center_profile}")

far_profile = {
    "position_scale": scheduled(1000, pos_scale, far_schedule),
    "velocity_normal": c_round_mul_div(400, scheduled(1000, vel_scale, far_schedule), 1000),
    "velocity_dynamic": c_round_mul_div(500, scheduled(1000, vel_scale, far_schedule), 1000),
    "raw_velocity": c_round_mul_div(100, scheduled(1000, raw_scale, far_schedule), 1000),
    "predict_normal": c_round_mul_div(90, scheduled(1000, predict_scale, far_schedule), 1000),
    "predict_dynamic": c_round_mul_div(130, scheduled(1000, predict_scale, far_schedule), 1000),
    "deadband": 3 + c_round_mul_div(deadband_extra, far_schedule, 1000),
    "bias_divider": 3 + c_round_mul_div(bias_extra, far_schedule, 1000),
    "slew_hold": c_round_mul_div(24, scheduled(1000, slew_scale, far_schedule), 1000),
    "slew_normal": c_round_mul_div(34, scheduled(1000, slew_scale, far_schedule), 1000),
    "slew_fast": c_round_mul_div(62, scheduled(1000, slew_scale, far_schedule), 1000),
    "slew_recover": c_round_mul_div(100, scheduled(1000, recover_slew_scale, far_schedule), 1000),
}
expected_far = {
    "position_scale": 720,
    "velocity_normal": 600,
    "velocity_dynamic": 750,
    "raw_velocity": 60,
    "predict_normal": 72,
    "predict_dynamic": 104,
    "deadband": 5,
    "bias_divider": 6,
    "slew_hold": 17,
    "slew_normal": 24,
    "slew_fast": 43,
    "slew_recover": 85,
}
if far_profile != expected_far:
    errors.append(f"-60 mm profile mismatch: {far_profile}")

if errors:
    print("TASK2/TASK6 OPTIMIZATION VALIDATION FAIL")
    for error in errors:
        print("FAIL:", error)
    sys.exit(1)

print("TASK2/TASK6 OPTIMIZATION VALIDATION PASS")
print("OK: Task2 cruise 390/320 mm/s, search gate 5.6 m, search speed 260 mm/s")
print("OK: Task2 20 s timeout and >=3 black-channel stop-line rule retained")
print("OK: 0 mm target preserves the original Task4/5 controller profile")
print("OK: -60 mm target selects full Task6 damping profile")
print("PROFILE 0mm :", center_profile)
print("PROFILE -60mm:", far_profile)
