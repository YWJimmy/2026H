#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def require(path: str, needle: str) -> None:
    text = (ROOT / path).read_text(encoding="utf-8")
    if needle not in text:
        raise AssertionError(f"missing in {path}: {needle}")


def require_no_bom(path: str) -> None:
    data = (ROOT / path).read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        raise AssertionError(f"unexpected UTF-8 BOM: {path}")
    if b"\x00" in data:
        raise AssertionError(f"unexpected NUL byte: {path}")


def sha256(path: str) -> str:
    return hashlib.sha256((ROOT / path).read_bytes()).hexdigest()


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def macro_int(text: str, name: str) -> int:
    pattern = rf"^#define\s+{re.escape(name)}\s+\(\((?:u?int\d+_t)\)(-?\d+)U?\)\s*$"
    match = re.search(pattern, text, re.MULTILINE)
    if not match:
        raise AssertionError(f"cannot parse macro: {name}")
    return int(match.group(1))


def require_order(text: str, *tokens: str) -> None:
    position = -1
    for token in tokens:
        next_position = text.find(token, position + 1)
        if next_position < 0:
            raise AssertionError(f"missing ordered token: {token}")
        position = next_position


def require_regex(text: str, pattern: str, label: str) -> None:
    if not re.search(pattern, text, re.DOTALL):
        raise AssertionError(f"missing state transition: {label}")


def main() -> int:
    # Task 2 currently uses the measured 80 mm sensor-to-axle offset.
    require("User/action/task2_lap_stop_config.h", "((uint32_t)80U)")

    cfg = "User/action/task6_lap_target_config.h"
    for token in (
        "TASK6_START_HOLD_MS                   ((uint32_t)300U)",
        "TASK6_START_RAMP_MS                   ((uint32_t)1400U)",
        "TASK6_START_CENTER_SPEED_MM_S         ((int32_t)120)",
        "TASK6_STOP_APPROACH_CENTER_SPEED_MM_S ((int32_t)280)",
        "TASK6_STOP_FINAL_CENTER_SPEED_MM_S    ((int32_t)180)",
        "TASK6_STOP_DECEL_MM_S2                ((int32_t)180)",
        "TASK6_STOP_JERK_MM_S3                 ((int32_t)800)",
        "TASK6_STOP_SETTLE_MS                  ((uint32_t)400U)",
    ):
        require(cfg, token)

    cfg_text = read(cfg)
    line_cfg_text = read("User/module/line_follow_control_config.h")
    start_hold_ms = macro_int(cfg_text, "TASK6_START_HOLD_MS")
    start_ramp_ms = macro_int(cfg_text, "TASK6_START_RAMP_MS")
    start_center = macro_int(cfg_text, "TASK6_START_CENTER_SPEED_MM_S")
    start_min = macro_int(cfg_text, "TASK6_START_MIN_SPEED_MM_S")
    approach_center = macro_int(
        cfg_text, "TASK6_STOP_APPROACH_CENTER_SPEED_MM_S")
    approach_min = macro_int(
        cfg_text, "TASK6_STOP_APPROACH_MIN_SPEED_MM_S")
    final_distance = macro_int(
        cfg_text, "TASK6_STOP_FINAL_STAGE_DISTANCE_MM")
    final_center = macro_int(
        cfg_text, "TASK6_STOP_FINAL_CENTER_SPEED_MM_S")
    final_min = macro_int(cfg_text, "TASK6_STOP_FINAL_MIN_SPEED_MM_S")
    stop_decel = macro_int(cfg_text, "TASK6_STOP_DECEL_MM_S2")
    stop_jerk = macro_int(cfg_text, "TASK6_STOP_JERK_MM_S3")
    stop_settle_ms = macro_int(cfg_text, "TASK6_STOP_SETTLE_MS")
    normal_center = macro_int(
        line_cfg_text, "LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S")
    normal_min = macro_int(
        line_cfg_text, "LINE_FOLLOW_CONTROL_MIN_BASE_SPEED_MM_S")
    start_accel = macro_int(
        line_cfg_text, "LINE_FOLLOW_CONTROL_BASE_ACCEL_SLEW_MM_S2")

    if not (0 < start_min <= start_center < normal_min <= normal_center):
        raise AssertionError("startup speed stages are not strictly ordered")
    if not (0 < final_min <= final_center < approach_min <= approach_center < normal_min):
        raise AssertionError("parking speed stages are not strictly ordered")
    if not (0 < final_distance < 300):
        raise AssertionError("final parking stage must be inside the 300 mm post-line run")
    if min(start_hold_ms, start_ramp_ms, stop_decel,
           stop_jerk, stop_settle_ms) <= 0:
        raise AssertionError("transient timing and stop limits must be positive")

    task6 = "User/action/task6_lap_target.c"
    for token in (
        "TASK6_STATE_START_HOLD",
        "TASK6_STATE_START_RAMP",
        "TASK6_STATE_STOP_SETTLE",
        "T6,START_RAMP=1",
        "T6,STOP_FINAL_STAGE=1",
        "T6,STOP_PROFILE=1",
        "T6,STOP_SETTLE=1",
        "TASK4_MAIN_BALL_TRANSIENT_STOPPING",
    ):
        require(task6, token)

    task6_text = read(task6)
    update_text = task6_text[task6_text.index(
        "Task6LapTargetResult_t Task6LapTarget_Update") :]
    for source_state, target_state in (
        ("CAPTURE_TARGET", "START_HOLD"),
        ("START_HOLD", "START_RAMP"),
        ("START_RAMP", "LAP_RUNNING"),
        ("FIND_A_LINE", "POST_LINE_RUN"),
        ("POST_LINE_RUN", "STOPPING"),
        ("STOPPING", "STOP_SETTLE"),
        ("STOP_SETTLE", "FINISHED"),
    ):
        require_regex(
            update_text,
            rf"if .*TASK6_STATE_{source_state}.*?"
            rf"s_state\s*=\s*TASK6_STATE_{target_state};",
            f"{source_state} -> {target_state}",
        )
    require_order(
        update_text,
        "TASK6_START_CENTER_SPEED_MM_S,",
        "!LineFollowControl_Start()",
        "LINE_FOLLOW_CONTROL_CENTER_SPEED_MM_S,",
    )
    require_order(
        update_text,
        "s_lap_elapsed_ms = now_ms - s_start_ms;",
        "TASK6_STOP_APPROACH_CENTER_SPEED_MM_S,",
        "TASK6_STOP_FINAL_CENTER_SPEED_MM_S,",
        "LineFollowControl_RequestStopWithDecel(",
    )

    main_c = "Core/Src/main.c"
    for token in (
        "Task4MainBall_SetTransient",
        "T4_BALL_START_RAMP_FF_MIN_SCALE_MILLI",
        "T4_BALL_STOPPING_DAMP_SCALE_MILLI",
        "T4Ball_TransientSlewLimit",
        "s_t4_ball_launch_active = false;",
        "Task4MainBall_TransientName",
        "T4_BALL_CROSS_BRAKE_MAX_ERROR_MM            ((int32_t)6)",
        "T4_BALL_CROSS_MIN_VELOCITY_PX_S",
        "T4_BALL_CROSS_MAX_FRAME_DELTA_PX",
        "(abs_error_mm > T4_BALL_ONE_CM_MM) ||",
    ):
        require(main_c, token)
    if "T4_BALL_CROSS_BRAKE_MAX_ERROR_PX" in read(main_c):
        raise AssertionError("crossing brake regressed to a pixel boundary")

    for path in (
        "Core/Src/main.c",
        "Core/Inc/task4_main_ball.h",
        "User/action/task6_lap_target.c",
        "User/action/task6_lap_target_config.h",
        "User/action/task2_lap_stop_config.h",
        "CHECK_TASK6_START_STOP_TRANSIENT_V4.py",
    ):
        require_no_bom(path)

    for project in ("MDK-ARM/2026H.uvprojx", "MDK-ARM/2026H.uvoptx"):
        ET.parse(ROOT / project)

    # Confirm the configured ramp can reach the actual cruise target.
    startup_reachable = start_center + (start_accel * start_ramp_ms) // 1000
    if startup_reachable < normal_center:
        raise AssertionError("startup ramp is too short to reach normal speed")

    print("TASK6_TRANSIENT_V4_AUDIT=PASS")
    print(f"startup_reachable_mm_s={startup_reachable}")
    print("cross_brake_physical_limit_mm=6")
    print("startup_and_parking_state_order=PASS")
    print(f"main_sha256={sha256(main_c)}")
    print(f"task6_sha256={sha256(task6)}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"TASK6_TRANSIENT_V4_AUDIT=FAIL: {exc}", file=sys.stderr)
        raise
