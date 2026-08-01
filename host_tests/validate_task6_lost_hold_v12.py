from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "User/action/task6_lap_target_config.h").read_text(
    encoding="utf-8"
)
TASK6 = (ROOT / "User/action/task6_lap_target.c").read_text(encoding="utf-8")
LINE_C = (ROOT / "User/module/line_follow_control.c").read_text(encoding="utf-8")
LINE_H = (ROOT / "User/module/line_follow_control.h").read_text(encoding="utf-8")


def u32_macro(name: str) -> int:
    match = re.search(
        rf"#define\s+{re.escape(name)}\s+\(\(uint32_t\)(\d+)U\)",
        CONFIG,
    )
    if not match:
        raise AssertionError(f"missing/unparseable macro: {name}")
    return int(match.group(1))


assert u32_macro("TASK6_LOST_COMMAND_HOLD_MS") == 250
assert "LineFollowControl_SetLostCommandHoldMs" in LINE_H
assert "LineFollowControl_SetLostCommandHoldMs" in LINE_C
assert "LineFollowControl_SetLostCommandHoldMs(" in TASK6
assert "CTRL=T6_LOST_HOLD_V12" in TASK6
assert "LOST_HOLD=%lu" in TASK6

lost_handler = re.search(
    r"static bool LineFollowControl_HandleLost\(uint32_t now_ms\)"
    r"\s*\{(?P<body>.*?)\n\}",
    LINE_C,
    re.DOTALL,
)
assert lost_handler, "lost-line handler not found"
body = lost_handler.group("body")
assert "s_lost_command_hold_ms" in body
assert "s_lost_hold_base_mm_s" in body
assert "s_lost_hold_turn_mm_s" in body
assert body.index("s_lost_command_hold_ms") < body.index(
    "LINE_FOLLOW_CONTROL_LOST_SEARCH_SPEED_MM_S"
), "hold-last-command path must precede the low-speed search fallback"

# V11 guarantees that the held normal base is the required constant cruise.
assert "#define TASK6_CRUISE_CENTER_SPEED_MM_S        ((int32_t)260)" in CONFIG
assert "#define TASK6_CRUISE_MIN_SPEED_MM_S           ((int32_t)260)" in CONFIG

print("TASK6 LOST-COMMAND HOLD V12 VALIDATION PASS")
print("transient line loss holds entry base/turn for 250 ms")
print("persistent loss still falls back to the existing bounded search/timeout")
