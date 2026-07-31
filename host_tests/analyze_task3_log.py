#!/usr/bin/env python3
"""Extract task-3 result and acceptance metrics from a serial log."""
from __future__ import annotations
import re
import sys
from pathlib import Path

RESULT_RE = re.compile(
    r"BALL_BAL,RESULT=(?P<result>PASS|FAIL),REASON=(?P<reason>[^,]+),"
    r"TOTAL_MS=(?P<total>\d+),POS5_MS=(?P<pos5>\d+),"
    r"FINAL_CX=(?P<cx>-?\d+),FINAL_ERR_PX=(?P<err>-?\d+),"
    r"FINAL_X_MM=(?P<x_mm>-?\d+),FINAL_ERR_MM=(?P<err_mm>-?\d+),"
    r"FINAL_SPD=(?P<spd>-?\d+),POS5_OVER_PX=(?P<p5>-?\d+),"
    r"NEG5_OVER_PX=(?P<n5>-?\d+)"
)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: analyze_task3_log.py <serial_log.txt>")
        return 2
    text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
    matches = list(RESULT_RE.finditer(text))
    if not matches:
        print("No optimized RESULT line found.")
        return 1
    m = matches[-1]
    d = {k: int(v) if k not in {"result", "reason"} else v
         for k, v in m.groupdict().items()}
    print(d)
    ok = (
        d["result"] == "PASS"
        and d["total"] <= 5000
        and abs(d["err_mm"]) <= 10
        and d["p5"] <= 43
        and d["n5"] <= 43
    )
    print("acceptance:", "PASS" if ok else "CHECK")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
