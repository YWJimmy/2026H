#!/usr/bin/env python3
"""Compare two real Task 6 UART captures using identical acceptance metrics."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from analyze_task6_log import (
    SERVO_LARGE_STEP_US,
    SERVO_PAIR_MAX_GAP_MS,
    SERVO_SIGNIFICANT_STEP_US,
    integer,
    parse_fields,
)


@dataclass(frozen=True)
class Sample:
    ms: int
    error_mm: int
    pulse_us: int


@dataclass(frozen=True)
class Summary:
    controller: str
    startup_peak_mm: int
    scored_peak_mm: int
    pulse_span_us: int
    max_step_us: int
    large_steps: int
    reversals: int
    lap_ms: int


def summarize(path: Path) -> Summary:
    current_ms: int | None = None
    lap_ms: int | None = None
    controller = "UNKNOWN"
    samples: list[Sample] = []

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        fields = parse_fields(line)
        if line.startswith("T6,START=1"):
            controller = fields.get("CTRL", "UNKNOWN")
        elif line.startswith("T6,ST="):
            current_ms = integer(fields, "MS")
        elif line.startswith("T6,A_LINE=1"):
            lap_ms = integer(fields, "LAP")
        elif line.startswith("T6BALL,") and current_ms is not None:
            error_mm = integer(fields, "ERRMM")
            pulse_us = integer(fields, "PULSE")
            if error_mm is not None and pulse_us is not None:
                samples.append(Sample(current_ms, error_mm, pulse_us))

    if not samples or lap_ms is None:
        raise ValueError(f"incomplete Task6 log: {path}")

    startup = [sample for sample in samples if sample.ms <= 3000]
    scored = [sample for sample in samples if sample.ms <= lap_ms]
    if not startup or not scored:
        raise ValueError(f"missing startup/scored samples: {path}")

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
    return Summary(
        controller=controller,
        startup_peak_mm=max(abs(sample.error_mm) for sample in startup),
        scored_peak_mm=max(abs(sample.error_mm) for sample in scored),
        pulse_span_us=(
            max(sample.pulse_us for sample in scored)
            - min(sample.pulse_us for sample in scored)
        ),
        max_step_us=max((abs(step) for step in pulse_steps), default=0),
        large_steps=sum(
            abs(step) >= SERVO_LARGE_STEP_US for step in pulse_steps
        ),
        reversals=sum(
            previous * current < 0
            for previous, current in zip(
                significant_steps, significant_steps[1:]
            )
        ),
        lap_ms=lap_ms,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--require-not-worse", action="store_true")
    args = parser.parse_args()

    baseline = summarize(args.baseline)
    candidate = summarize(args.candidate)
    metrics = (
        ("startup_peak_mm", baseline.startup_peak_mm, candidate.startup_peak_mm),
        ("scored_peak_mm", baseline.scored_peak_mm, candidate.scored_peak_mm),
        ("pulse_span_us", baseline.pulse_span_us, candidate.pulse_span_us),
        ("max_step_us", baseline.max_step_us, candidate.max_step_us),
        ("large_steps", baseline.large_steps, candidate.large_steps),
        ("reversals", baseline.reversals, candidate.reversals),
        ("lap_ms", baseline.lap_ms, candidate.lap_ms),
    )

    print(f"baseline_controller={baseline.controller}")
    print(f"candidate_controller={candidate.controller}")
    regressions: list[str] = []
    for name, old, new in metrics:
        delta = new - old
        print(f"{name}: baseline={old},candidate={new},delta={delta:+d}")
        if new > old:
            regressions.append(name)

    if regressions:
        print("RESULT=WORSE")
        print("REGRESSED=" + ",".join(regressions))
        return 1 if args.require_not_worse else 0

    print("RESULT=NOT_WORSE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
