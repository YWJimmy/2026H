#!/usr/bin/env python3
"""Analyze a Task 6 UART capture against the real competition limits.

The script intentionally separates three questions that were previously mixed:
1. Which startup controller actually ran?
2. Did the tray establish a physical servo preload before wheel motion?
3. Did the ball remain within +/-10 mm for the complete scored lap?
4. Did the servo command avoid rapid large-amplitude reversals?

Exit status is non-zero when ``--require-pass`` is used and any mandatory
criterion fails, which makes this suitable as the red/green hardware replay
loop for Task 6 tuning.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path


FIELD_RE = re.compile(r"(?:^|,)([A-Z0-9_]+)=([^,\r\n]+)")

SERVO_PAIR_MAX_GAP_MS = 400
SERVO_SIGNIFICANT_STEP_US = 80
SERVO_LARGE_STEP_US = 120
SERVO_SINGLE_STEP_LIMIT_US = 180


def parse_fields(line: str) -> dict[str, str]:
    return {match.group(1): match.group(2) for match in FIELD_RE.finditer(line)}


def integer(fields: dict[str, str], name: str) -> int | None:
    value = fields.get(name)
    if value is None:
        return None
    try:
        return int(value, 0)
    except ValueError:
        return None


def split_task6_runs(lines: list[str]) -> list[list[str]]:
    """Split a UART capture into independent Task 6 physical runs.

    Menu/boot chatter before the first START marker is intentionally ignored.
    A capture without a START marker remains one fragment for backward
    compatibility with clipped hardware logs.
    """

    starts = [
        index
        for index, line in enumerate(lines)
        if line.startswith("T6,START=1")
    ]
    if not starts:
        return [lines]
    return [
        lines[start : starts[index + 1] if index + 1 < len(starts) else len(lines)]
        for index, start in enumerate(starts)
    ]


@dataclass(frozen=True)
class BallSample:
    ms: int
    error_mm: int
    velocity_px_s: int
    pulse_us: int
    feedforward_us: int
    violation_latched: bool


def analyze(
    lines: list[str], *, require_stable_servo: bool = False
) -> tuple[list[str], bool]:
    start_fields: dict[str, str] = {}
    target_lock_ms: int | None = None
    first_motion_ms: int | None = None
    a_line_ms: int | None = None
    current_ms: int | None = None
    samples: list[BallSample] = []
    preload_ready: dict[str, str] | None = None

    for line in lines:
        fields = parse_fields(line)
        if line.startswith("T6,START=1"):
            start_fields = fields
        elif line.startswith("T6,TARGET_LOCK=1"):
            target_lock_ms = integer(fields, "MS")
        elif line.startswith("T6,PRELOAD_READY=1"):
            preload_ready = fields
        elif line.startswith("T6,A_LINE=1"):
            a_line_ms = integer(fields, "LAP")

        if line.startswith("T6,ST="):
            current_ms = integer(fields, "MS")
            command = fields.get("CMD", "0/0")
            try:
                left, right = (int(part) for part in command.split("/", 1))
            except (ValueError, TypeError):
                left = right = 0
            if first_motion_ms is None and (left != 0 or right != 0):
                first_motion_ms = current_ms

        if line.startswith("T6BALL,") and current_ms is not None:
            error_mm = integer(fields, "ERRMM")
            velocity_px_s = integer(fields, "VF")
            pulse_us = integer(fields, "PULSE")
            feedforward_us = integer(fields, "FF")
            violation = integer(fields, "VIOL")
            if None not in (
                error_mm,
                velocity_px_s,
                pulse_us,
                feedforward_us,
                violation,
            ):
                samples.append(
                    BallSample(
                        ms=current_ms,
                        error_mm=error_mm,
                        velocity_px_s=velocity_px_s,
                        pulse_us=pulse_us,
                        feedforward_us=feedforward_us,
                        violation_latched=bool(violation),
                    )
                )

    report: list[str] = []
    failures: list[str] = []

    controller = start_fields.get("CTRL", "UNKNOWN")
    preload_ms = integer(start_fields, "PRELOAD_MIN")
    if preload_ms is None:
        preload_ms = integer(start_fields, "PRELOAD")
    if preload_ms is None:
        preload_ms = integer(start_fields, "HOLD")
    report.append(f"controller={controller}")
    report.append(f"configured_preload_ms={preload_ms if preload_ms is not None else 'unknown'}")

    if not any(
        marker in controller
        for marker in (
            "ACTIVE_PRELOAD",
            "ADAPTIVE_LAUNCH_GUARD",
            "FRAME_SYNC_SERVO",
            "DECOUPLED_START",
            "SLOW_RAMP",
        )
    ):
        failures.append("firmware_identity: Task6 preload/launch-guard marker missing")

    if target_lock_ms is None:
        failures.append("target_lock: TARGET_LOCK marker missing")
    if first_motion_ms is None:
        failures.append("wheel_motion: no non-zero CMD sample")

    if preload_ready is None:
        failures.append("preload_output: PRELOAD_READY marker missing")
    else:
        preload_delta = integer(preload_ready, "DELTA")
        preload_error = integer(preload_ready, "ERRMM")
        report.append(
            "preload_ready="
            f"delta_us={preload_delta if preload_delta is not None else 'unknown'},"
            f"error_mm={preload_error if preload_error is not None else 'unknown'}"
        )
        if preload_delta is None or abs(preload_delta) < 40:
            failures.append("preload_output: physical servo delta below 40 us")
        if preload_error is not None and abs(preload_error) > 5:
            failures.append("preload_output: ball moved over 5 mm before launch")

    if target_lock_ms is not None and first_motion_ms is not None:
        report.append(f"lock_to_motion_ms={first_motion_ms - target_lock_ms}")

    if not samples:
        failures.append("ball_samples: no complete T6BALL samples")
    else:
        startup = [sample for sample in samples if sample.ms <= 3000]
        scored = [
            sample
            for sample in samples
            if a_line_ms is None or sample.ms <= a_line_ms
        ]
        if startup:
            startup_peak = max(startup, key=lambda sample: abs(sample.error_mm))
            report.append(
                f"startup_peak_abs_error_mm={abs(startup_peak.error_mm)}@{startup_peak.ms}ms"
            )
            if abs(startup_peak.error_mm) > 10:
                failures.append("startup_error: exceeds competition +/-10 mm")
        if scored:
            scored_peak = max(scored, key=lambda sample: abs(sample.error_mm))
            report.append(
                f"scored_peak_abs_error_mm={abs(scored_peak.error_mm)}@{scored_peak.ms}ms"
            )
            if abs(scored_peak.error_mm) > 10:
                failures.append("scored_error: exceeds competition +/-10 mm")
            if any(sample.violation_latched for sample in scored):
                failures.append("scored_latch: VIOL became 1 before A line")

            pulse_min = min(sample.pulse_us for sample in scored)
            pulse_max = max(sample.pulse_us for sample in scored)
            pulse_steps = [
                current.pulse_us - previous.pulse_us
                for previous, current in zip(scored, scored[1:])
                if 0 < current.ms - previous.ms <= SERVO_PAIR_MAX_GAP_MS
            ]
            significant_steps = [
                step
                for step in pulse_steps
                if abs(step) >= SERVO_SIGNIFICANT_STEP_US
            ]
            large_steps = sum(
                abs(step) >= SERVO_LARGE_STEP_US for step in pulse_steps
            )
            reversals = sum(
                previous * current < 0
                for previous, current in zip(
                    significant_steps, significant_steps[1:]
                )
            )
            max_step = max((abs(step) for step in pulse_steps), default=0)
            report.append(
                "servo_command="
                f"range_us={pulse_min}..{pulse_max},"
                f"span_us={pulse_max - pulse_min},"
                f"max_step_us={max_step},"
                f"large_steps={large_steps},"
                f"direction_reversals={reversals}"
            )
            violent_oscillation = (
                max_step > SERVO_SINGLE_STEP_LIMIT_US
                or (large_steps >= 3 and reversals >= 3)
            )
            if require_stable_servo and violent_oscillation:
                failures.append(
                    "servo_oscillation: rapid large command reversals detected"
                )

    if a_line_ms is not None:
        report.append(f"lap_time_ms={a_line_ms}")
        if a_line_ms > 30000:
            failures.append("lap_time: exceeds 30000 ms")
    else:
        failures.append("lap_time: A_LINE marker missing")

    if failures:
        report.append("RESULT=FAIL")
        report.extend(f"FAIL: {failure}" for failure in failures)
        return report, False

    report.append("RESULT=PASS")
    return report, True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--require-pass", action="store_true")
    parser.add_argument("--require-stable-servo", action="store_true")
    parser.add_argument(
        "--latest-run",
        action="store_true",
        help="analyze only the final run when a capture contains multiple runs",
    )
    args = parser.parse_args()

    lines = args.log.read_text(encoding="utf-8", errors="replace").splitlines()
    runs = split_task6_runs(lines)
    if args.latest_run:
        runs = runs[-1:]

    if len(runs) == 1:
        report, passed = analyze(
            runs[0], require_stable_servo=args.require_stable_servo
        )
        print("\n".join(report))
        return 1 if args.require_pass and not passed else 0

    passed_count = 0
    for index, run in enumerate(runs, start=1):
        report, passed = analyze(
            run, require_stable_servo=args.require_stable_servo
        )
        print(f"[RUN {index}/{len(runs)}]")
        print("\n".join(report))
        if passed:
            passed_count += 1
    all_passed = passed_count == len(runs)
    print(
        f"MULTI_RUN_RESULT={'PASS' if all_passed else 'FAIL'},"
        f"passed={passed_count},failed={len(runs) - passed_count},"
        f"total={len(runs)}"
    )
    return 1 if args.require_pass and not all_passed else 0


if __name__ == "__main__":
    raise SystemExit(main())
