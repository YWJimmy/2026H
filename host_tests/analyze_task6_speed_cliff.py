from __future__ import annotations

import argparse
from pathlib import Path
import re


ROW_RE = re.compile(
    r"T6,ST=(?P<state>\d+),MS=(?P<ms>\d+).*?"
    r"MASK=0x(?P<mask>[0-9A-Fa-f]+),CMD=(?P<left>-?\d+)/(?P<right>-?\d+)"
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Detect Task 6 curve-entry forward-speed cliffs"
    )
    parser.add_argument("log", type=Path)
    parser.add_argument("--after-ms", type=int, default=8000)
    args = parser.parse_args()

    rows: list[dict[str, int]] = []
    for line in args.log.read_text(encoding="utf-8", errors="replace").splitlines():
        match = ROW_RE.search(line)
        if not match:
            continue
        left = int(match.group("left"))
        right = int(match.group("right"))
        rows.append(
            {
                "ms": int(match.group("ms")),
                "mask": int(match.group("mask"), 16),
                "left": left,
                "right": right,
                "average": (left + right) // 2,
            }
        )

    cliffs: list[tuple[dict[str, int], dict[str, int]]] = []
    for previous, current in zip(rows, rows[1:]):
        if current["ms"] < args.after_ms:
            continue
        if (
            previous["average"] >= 250
            and current["average"] <= 100
            and previous["average"] - current["average"] >= 150
        ):
            cliffs.append((previous, current))

    print(f"samples={len(rows)}")
    print(f"speed_cliffs={len(cliffs)}")
    for previous, current in cliffs:
        print(
            "CLIFF,"
            f"MS={current['ms']},MASK=0x{current['mask']:02X},"
            f"AVG={previous['average']}->{current['average']},"
            f"CMD={current['left']}/{current['right']}"
        )

    if cliffs:
        print("SOURCE=TRANSIENT_LINE_LOSS_SEARCH")
        print("RESULT=FAIL")
        return 1

    print("RESULT=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
