#!/usr/bin/env python3
import re
import sys
from collections import Counter
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("usage: python analyze_task4_v2_log.py task4_log.txt")
text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
lines = [line.strip() for line in text.splitlines() if line.startswith("T4BALL,")]
if not lines:
    raise SystemExit("no T4BALL lines found")

def grab(line, key):
    m = re.search(rf"(?:^|,){re.escape(key)}=(-?\d+)", line)
    return int(m.group(1)) if m else None

modes = Counter()
pulses, errors, velocities, ffs, planned, measured = [], [], [], [], [], []
viol = 0
for line in lines:
    m = re.search(r"(?:^|,)MODE=([^,]+)", line)
    if m:
        modes[m.group(1)] += 1
    for key, dest in [
        ("PULSE", pulses), ("ERR", errors), ("VF", velocities),
        ("FF", ffs), ("CA", planned), ("MA", measured),
    ]:
        value = grab(line, key)
        if value is not None:
            dest.append(value)
    v = grab(line, "VIOL")
    if v:
        viol = 1

print(f"samples={len(lines)}")
print(f"modes={dict(modes)}")
if errors:
    print(f"max_abs_error_px={max(abs(v) for v in errors)}")
if velocities:
    print(f"max_abs_velocity_px_s={max(abs(v) for v in velocities)}")
if pulses:
    print(f"pulse_range_us={min(pulses)}..{max(pulses)}")
if ffs:
    print(f"feedforward_range_us={min(ffs)}..{max(ffs)}")
if planned:
    print(f"planned_accel_range={min(planned)}..{max(planned)} mm/s^2")
if measured:
    print(f"measured_accel_range={min(measured)}..{max(measured)} mm/s^2")
print(f"one_cm_violation={viol}")

result = [line for line in text.splitlines() if line.startswith("T4,RESULT=")]
if result:
    print(result[-1])
