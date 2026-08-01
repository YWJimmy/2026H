#!/usr/bin/env python3
from __future__ import annotations

import hashlib
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


def main() -> int:
    require("User/action/task2_lap_stop_config.h", "((uint32_t)100U)")

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

    main_c = "Core/Src/main.c"
    for token in (
        "Task4MainBall_SetTransient",
        "T4_BALL_START_RAMP_FF_MIN_SCALE_MILLI",
        "T4_BALL_STOPPING_DAMP_SCALE_MILLI",
        "T4Ball_TransientSlewLimit",
        "s_t4_ball_launch_active = false;",
        "Task4MainBall_TransientName",
    ):
        require(main_c, token)

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

    # A coarse consistency check for the startup ramp duration:
    # 120 + 180 mm/s^2 * 1.4 s = 372 mm/s, enough to reach the 360 target.
    startup_reachable = 120 + (180 * 1400) // 1000
    if startup_reachable < 360:
        raise AssertionError("startup ramp is too short to reach normal speed")

    print("TASK6_TRANSIENT_V4_AUDIT=PASS")
    print(f"startup_reachable_mm_s={startup_reachable}")
    print(f"main_sha256={sha256(main_c)}")
    print(f"task6_sha256={sha256(task6)}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"TASK6_TRANSIENT_V4_AUDIT=FAIL: {exc}", file=sys.stderr)
        raise
