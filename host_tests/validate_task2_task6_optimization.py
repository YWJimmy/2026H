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


def macro_u8(text: str, name: str) -> int:
    pattern = rf"^#define\s+{re.escape(name)}\s+\(\(uint8_t\)(\d+)U\)\s*$"
    match = re.search(pattern, text, re.MULTILINE)
    if not match:
        errors.append(f"cannot parse uint8 macro: {name}")
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
    ("((uint32_t)80U)", "Task2 80 mm sensor offset"),
]:
    require(t2_cfg, token, label)

require(t2_src, "TASK2_CRUISE_CENTER_SPEED_MM_S,", "Task2 uses task-specific cruise")
require(t2_src, "TASK2_CRUISE_MIN_SPEED_MM_S))", "Task2 uses task-specific minimum")
require(t2_src, "s_distance_snapshot.traveled_mm >=", "Task2 distance gate")
require(t2_src, "TASK2_PREDECEL_DISTANCE_MM))", "Task2 predeceleration transition")
require(t2_src, "s_line_result_snapshot.black_count >= 3U", "Task2 stop-line rule retained")

for token, label in [
    ("T4_BALL_TARGET_SCHEDULE_START_MM", "schedule start"),
    ("T4_BALL_TARGET_SCHEDULE_FULL_MM", "schedule full"),
    ("T4_BALL_OFFCENTER_POSITION_SCALE_MILLI", "position scale"),
    ("T4_BALL_OFFCENTER_VELOCITY_SCALE_MILLI", "velocity scale"),
    ("T4_BALL_OFFCENTER_RAW_VEL_SCALE_MILLI", "raw velocity scale"),
    ("T4_BALL_OFFCENTER_PREDICT_SCALE_MILLI", "prediction scale"),
    ("T4_BALL_OFFCENTER_SLEW_SCALE_MILLI", "slew scale"),
    ("T4_BALL_EXTREME_SCHEDULE_START_MM", "extreme schedule start"),
    ("T4_BALL_EXTREME_SCHEDULE_FULL_MM", "extreme schedule full"),
    ("T4_BALL_CROSS_BRAKE_FRAMES", "crossing brake"),
    ("T4_BALL_CROSS_BRAKE_MAX_ERROR_MM", "physical crossing boundary"),
    ("T4_BALL_CROSS_MIN_VELOCITY_PX_S", "crossing velocity gate"),
    ("T4_BALL_CROSS_MAX_FRAME_DELTA_PX", "vision jump gate"),
    ("T4Ball_ExtremeScheduleMilli", "extreme schedule helper"),
    ("T4Ball_TargetScheduleMilli", "schedule helper"),
    ("T4Ball_ScheduledSlew", "scheduled slew helper"),
]:
    require(main_src, token, f"Task6 {label}")

for token in (
    "target_schedule_milli",
    "extreme_schedule_milli",
    "position_gain_milli",
    "velocity_gain_milli",
    "prediction_horizon_ms",
    "position_deadband_px",
    "fast_error_threshold_px",
    "recover_error_threshold_px",
    "cross_brake_frames",
):
    require(ball_hdr, token, f"Task6 diagnostic {token}")
require(t6_src, '"T6GAIN,SCH=%ld,XSCH=%ld,PG=%ld,VG=%ld,PRED=%ld,DB=%ld,', "Task6 serial diagnostics")
require(main_src, "(abs_error_mm > T4_BALL_ONE_CM_MM) ||", "physical out-of-band recovery")
if "T4_BALL_CROSS_BRAKE_MAX_ERROR_PX" in main_src:
    errors.append("crossing brake still uses a fixed pixel tolerance")

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
extreme_start_mm = macro_i32(main_src, "T4_BALL_EXTREME_SCHEDULE_START_MM")
extreme_full_mm = macro_i32(main_src, "T4_BALL_EXTREME_SCHEDULE_FULL_MM")
extreme_pos_scale = macro_i32(main_src, "T4_BALL_EXTREME_POSITION_SCALE_MILLI")
extreme_vel_scale = macro_i32(main_src, "T4_BALL_EXTREME_VELOCITY_SCALE_MILLI")
extreme_raw_scale = macro_i32(main_src, "T4_BALL_EXTREME_RAW_VEL_SCALE_MILLI")
extreme_predict_scale = macro_i32(main_src, "T4_BALL_EXTREME_PREDICT_SCALE_MILLI")
extreme_slew_scale = macro_i32(main_src, "T4_BALL_EXTREME_SLEW_SCALE_MILLI")
extreme_recover_slew_scale = macro_i32(main_src, "T4_BALL_EXTREME_RECOVER_SLEW_MILLI")
extreme_deadband_extra = macro_i32(main_src, "T4_BALL_EXTREME_DEADBAND_EXTRA_PX")
extreme_bias_extra = macro_i32(main_src, "T4_BALL_EXTREME_BIAS_DIVIDER_EXTRA")
cross_frames = macro_u8(main_src, "T4_BALL_CROSS_BRAKE_FRAMES")
cross_min_velocity = macro_i32(main_src, "T4_BALL_CROSS_MIN_VELOCITY_PX_S")
cross_max_delta = macro_i32(main_src, "T4_BALL_CROSS_MAX_FRAME_DELTA_PX")
one_cm_mm = macro_i32(main_src, "T4_BALL_ONE_CM_MM")
cross_brake_limit_mm = macro_i32(
    main_src, "T4_BALL_CROSS_BRAKE_MAX_ERROR_MM")
if cross_brake_limit_mm != 6:
    errors.append(
        f"crossing brake physical limit is not 6 mm: {cross_brake_limit_mm}")

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
    "position_scale": 650,
    "velocity_normal": 660,
    "velocity_dynamic": 825,
    "raw_velocity": 50,
    "predict_normal": 63,
    "predict_dynamic": 91,
    "deadband": 6,
    "bias_divider": 7,
    "slew_hold": 14,
    "slew_normal": 20,
    "slew_fast": 37,
    "slew_recover": 75,
}
if far_profile != expected_far:
    errors.append(f"-60 mm profile mismatch: {far_profile}")


def profile_for(target_mm: int) -> dict[str, int]:
    base_schedule = schedule_for(target_mm, start_mm, full_mm)
    extreme_schedule = schedule_for(
        target_mm, extreme_start_mm, extreme_full_mm)
    base_position = scheduled(1000, pos_scale, base_schedule)
    base_velocity = scheduled(1000, vel_scale, base_schedule)
    base_raw = scheduled(1000, raw_scale, base_schedule)
    base_predict = scheduled(1000, predict_scale, base_schedule)
    base_slew = scheduled(1000, slew_scale, base_schedule)
    base_recover_slew = scheduled(
        1000, recover_slew_scale, base_schedule)
    base_deadband_extra = c_round_mul_div(
        deadband_extra, base_schedule, 1000)
    base_bias_extra = c_round_mul_div(
        bias_extra, base_schedule, 1000)
    return {
        "position_scale": scheduled(
            base_position, extreme_pos_scale, extreme_schedule),
        "velocity_scale": scheduled(
            base_velocity, extreme_vel_scale, extreme_schedule),
        "raw_scale": scheduled(
            base_raw, extreme_raw_scale, extreme_schedule),
        "predict_scale": scheduled(
            base_predict, extreme_predict_scale, extreme_schedule),
        "slew_scale": scheduled(
            base_slew, extreme_slew_scale, extreme_schedule),
        "recover_slew_scale": scheduled(
            base_recover_slew,
            extreme_recover_slew_scale,
            extreme_schedule),
        "deadband": 3 + scheduled(
            base_deadband_extra,
            extreme_deadband_extra,
            extreme_schedule),
        "bias_divider": 3 + base_bias_extra + c_round_mul_div(
            extreme_bias_extra - bias_extra,
            extreme_schedule,
            1000),
    }


expected_extreme = {
    "position_scale": 500,
    "velocity_scale": 1900,
    "raw_scale": 350,
    "predict_scale": 550,
    "slew_scale": 420,
    "recover_slew_scale": 600,
    "deadband": 8,
    "bias_divider": 11,
}
if profile_for(110) != expected_extreme:
    errors.append(f"110 mm profile mismatch: {profile_for(110)}")
if profile_for(-110) != expected_extreme:
    errors.append(f"-110 mm profile is not symmetric: {profile_for(-110)}")

profiles = [profile_for(mm) for mm in (60, 85, 110)]
for left, right in zip(profiles, profiles[1:]):
    if not (left["position_scale"] > right["position_scale"]):
        errors.append("extreme position scheduling is not monotonic")
    if not (left["velocity_scale"] < right["velocity_scale"]):
        errors.append("extreme velocity scheduling is not monotonic")
    if not (left["slew_scale"] > right["slew_scale"]):
        errors.append("extreme slew scheduling is not monotonic")


def crossing_step(
    counter: int,
    last_sign: int,
    error_px: int,
    error_mm: int,
    raw_velocity: int,
    filtered_velocity: int,
    frame_delta: int,
    dt_ms: int,
    extreme_schedule: int = 1000,
) -> tuple[int, int, bool]:
    deadband = profile_for(110)["deadband"]
    current_sign = 1 if error_px > deadband else (
        -1 if error_px < -deadband else 0)
    motion_valid = (
        8 <= dt_ms <= 120
        and abs(frame_delta) <= cross_max_delta
        and abs(raw_velocity) >= cross_min_velocity
        and (
            current_sign > 0
            and raw_velocity > 0
            and filtered_velocity > 0
            or current_sign < 0
            and raw_velocity < 0
            and filtered_velocity < 0
        )
    )
    if extreme_schedule > 0 and current_sign != 0:
        if (
            last_sign != 0
            and current_sign != last_sign
            and abs(error_mm) <= cross_brake_limit_mm
            and motion_valid
        ):
            counter = cross_frames
        last_sign = current_sign
    elif extreme_schedule == 0:
        last_sign = current_sign
        counter = 0
    active = (
        counter > 0
        and abs(error_mm) <= cross_brake_limit_mm
        and extreme_schedule > 0
        and motion_valid
    )
    if not active:
        counter = 0
    elif counter > 0:
        counter -= 1
    return counter, last_sign, active


# A real crossing brakes for three accepted frames.
counter, sign, active = crossing_step(0, 1, -9, -2, -200, -120, -7, 33)
if not active or counter != 2:
    errors.append("valid crossing did not start the three-frame brake")
counter, sign, active = crossing_step(counter, sign, -12, -3, -120, -90, -4, 33)
counter, sign, active = crossing_step(counter, sign, -14, -4, -80, -60, -3, 33)
if not active or counter != 0:
    errors.append("valid crossing brake did not last exactly three frames")

# Safety and vision-quality gates must cancel or reject braking.
for label, args in [
    ("outside six millimeters", (2, -1, -35, -7, -120, -90, -4, 33)),
    ("outside one centimeter", (2, -1, -50, -11, -120, -90, -4, 33)),
    ("invalid timestamp", (0, 1, -9, -2, -200, -120, -7, 0)),
    ("low speed", (0, 1, -9, -2, -20, -15, -1, 33)),
    ("vision jump", (0, 1, -40, -9, -1000, -300, -40, 33)),
    ("motion reversed", (2, -1, -12, -3, 100, 60, 3, 33)),
]:
    next_counter, _, next_active = crossing_step(*args)
    if next_active or next_counter != 0:
        errors.append(f"crossing brake accepted {label}")

if errors:
    print("TASK2/TASK6 OPTIMIZATION VALIDATION FAIL")
    for error in errors:
        print("FAIL:", error)
    sys.exit(1)

print("TASK2/TASK6 OPTIMIZATION VALIDATION PASS")
print("OK: Task2 cruise 390/320 mm/s, search gate 5.6 m, search speed 260 mm/s")
print("OK: Task2 20 s timeout and >=3 black-channel stop-line rule retained")
print("OK: 0 mm target preserves the original Task4/5 controller profile")
print("OK: -60 mm target selects the refined off-center profile")
print("OK: 60~110 mm extreme profiles are continuous, symmetric and monotonic")
print("OK: crossing brake is bounded to +/-6 mm and rejects invalid motion")
print("PROFILE 0mm :", center_profile)
print("PROFILE -60mm:", far_profile)
print("PROFILE 110mm:", profile_for(110))
