#!/usr/bin/env python3
"""Separate chassis steering jitter from tray-servo jitter in Task 6 curves."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from analyze_task6_log import integer, parse_fields, split_task6_runs


CURVE_STEER_MIN_MM_S = 80
PAIR_MAX_GAP_MS = 400
SERVO_LARGE_STEP_US = 120
SERVO_SINGLE_STEP_LIMIT_US = 180
STEER_LARGE_STEP_MM_S = 80


@dataclass(frozen=True)
class CurveSample:
    ms: int
    mask: str
    left_mm_s: int
    right_mm_s: int
    error_mm: int
    velocity_px_s: int
    pulse_us: int

    @property
    def steer_mm_s(self) -> int:
        return self.left_mm_s - self.right_mm_s


def parse_run(lines: list[str]) -> list[CurveSample]:
    pending: tuple[int, str, int, int] | None = None
    samples: list[CurveSample] = []
    for line in lines:
        fields = parse_fields(line)
        if line.startswith("T6,ST="):
            ms = integer(fields, "MS")
            command = fields.get("CMD")
            if ms is None or command is None:
                pending = None
                continue
            try:
                left, right = (int(part) for part in command.split("/", 1))
            except ValueError:
                pending = None
                continue
            pending = (ms, fields.get("MASK", "UNKNOWN"), left, right)
        elif line.startswith("T6BALL,") and pending is not None:
            error = integer(fields, "ERRMM")
            velocity = integer(fields, "VF")
            pulse = integer(fields, "PULSE")
            if error is not None and velocity is not None and pulse is not None:
                samples.append(
                    CurveSample(*pending, error, velocity, pulse)
                )
            pending = None
    return samples


def analyze_run(lines: list[str]) -> tuple[list[str], bool]:
    samples = parse_run(lines)
    curve = [
        sample
        for sample in samples
        if abs(sample.steer_mm_s) >= CURVE_STEER_MIN_MM_S
    ]
    report: list[str] = []
    if len(curve) < 2:
        return ["RESULT=FAIL", "FAIL: insufficient curve samples"], False

    pairs = [
        (previous, current)
        for previous, current in zip(curve, curve[1:])
        if 0 < current.ms - previous.ms <= PAIR_MAX_GAP_MS
    ]
    pulse_steps = [
        current.pulse_us - previous.pulse_us for previous, current in pairs
    ]
    steer_steps = [
        current.steer_mm_s - previous.steer_mm_s
        for previous, current in pairs
    ]
    steer_direction_reversals = sum(
        previous.steer_mm_s * current.steer_mm_s < 0
        for previous, current in pairs
    )
    significant_pulse_steps = [
        step for step in pulse_steps if abs(step) >= 80
    ]
    pulse_direction_reversals = sum(
        previous * current < 0
        for previous, current in zip(
            significant_pulse_steps, significant_pulse_steps[1:]
        )
    )
    large_pulse_steps = sum(
        abs(step) >= SERVO_LARGE_STEP_US for step in pulse_steps
    )
    max_pulse_step = max((abs(step) for step in pulse_steps), default=0)
    max_steer_step = max((abs(step) for step in steer_steps), default=0)
    curve_peak_error = max(abs(sample.error_mm) for sample in curve)

    chassis_jitter = (
        steer_direction_reversals >= 3
        and max_steer_step >= STEER_LARGE_STEP_MM_S
    )
    servo_jitter = (
        max_pulse_step > SERVO_SINGLE_STEP_LIMIT_US
        or (large_pulse_steps >= 3 and pulse_direction_reversals >= 3)
    )

    report.append(f"curve_samples={len(curve)}")
    report.append(f"curve_peak_abs_error_mm={curve_peak_error}")
    report.append(
        "chassis_steer="
        f"max_step_mm_s={max_steer_step},"
        f"direction_reversals={steer_direction_reversals}"
    )
    report.append(
        "curve_servo="
        f"range_us={min(sample.pulse_us for sample in curve)}.."
        f"{max(sample.pulse_us for sample in curve)},"
        f"max_step_us={max_pulse_step},"
        f"large_steps={large_pulse_steps},"
        f"direction_reversals={pulse_direction_reversals}"
    )
    if servo_jitter and not chassis_jitter:
        report.append("SOURCE=TRAY_SERVO_BALL_LOOP")
    elif chassis_jitter and not servo_jitter:
        report.append("SOURCE=CHASSIS_STEERING_LOOP")
    elif chassis_jitter and servo_jitter:
        report.append("SOURCE=BOTH_LOOPS")
    else:
        report.append("SOURCE=STABLE")

    passed = not chassis_jitter and not servo_jitter and curve_peak_error <= 10
    report.append(f"RESULT={'PASS' if passed else 'FAIL'}")
    if chassis_jitter:
        report.append("FAIL: chassis steering reverses repeatedly in curves")
    if servo_jitter:
        report.append("FAIL: tray servo command oscillates in curves")
    if curve_peak_error > 10:
        report.append("FAIL: curve ball error exceeds +/-10 mm")
    return report, passed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--require-stable-curve", action="store_true")
    args = parser.parse_args()

    lines = args.log.read_text(encoding="utf-8", errors="replace").splitlines()
    runs = split_task6_runs(lines)
    all_passed = True
    for index, run in enumerate(runs, start=1):
        if len(runs) > 1:
            print(f"[RUN {index}/{len(runs)}]")
        report, passed = analyze_run(run)
        print("\n".join(report))
        all_passed = all_passed and passed
    return 1 if args.require_stable_curve and not all_passed else 0


if __name__ == "__main__":
    raise SystemExit(main())
