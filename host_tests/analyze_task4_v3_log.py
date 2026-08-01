#!/usr/bin/env python3
import re
import sys
from pathlib import Path

if len(sys.argv) != 2:
    raise SystemExit("usage: analyze_task4_v3_log.py task4_log.txt")
text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
lines = text.splitlines()
start = next((x for x in lines if "CTRL=MAIN_LAUNCH_FF_OLED_V3" in x), None)
locks = [x for x in lines if "T4,B_REACHED=1" in x]
results = [x for x in lines if "T4,RESULT=" in x]
balls = [x for x in lines if x.startswith("T4BALL,")]
print("V3_START:", "PASS" if start else "MISSING")
if locks:
    lock = locks[-1]
    trig = re.search(r"TRIG=([^,]+)", lock)
    ab = re.search(r"AB=(\d+)", lock)
    dist = re.search(r"DIST=(\d+)", lock)
    print("B_LOCK:", trig.group(1) if trig else "?",
          "AB_MS=", ab.group(1) if ab else "?",
          "DIST_MM=", dist.group(1) if dist else "?")
else:
    print("B_LOCK: MISSING")
if results:
    print("RESULT:", results[-1])
else:
    print("RESULT: MISSING")
if balls:
    launch = [x for x in balls if "LAUNCH=1" in x]
    ff = []
    maxmm = 0
    for x in balls:
        m = re.search(r"FF=(-?\d+)", x)
        if m:
            ff.append(int(m.group(1)))
        m = re.search(r"MAXMM=(\d+)", x)
        if m:
            maxmm = max(maxmm, int(m.group(1)))
    print("BALL_SAMPLES:", len(balls), "LAUNCH_SAMPLES:", len(launch),
          "FF_MIN/MAX:", (min(ff), max(ff)) if ff else "?",
          "MAX_ERR_MM:", maxmm)
else:
    print("BALL_SAMPLES: 0")
