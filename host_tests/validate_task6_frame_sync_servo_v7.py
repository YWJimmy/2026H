#!/usr/bin/env python3
"""Static contract for Task 6 frame-synchronous servo control V7.

The position and vision-velocity terms must advance only once per accepted
vision frame. Chassis-rate calls may still apply the incremental acceleration
feedforward needed by stationary preload and launch compensation.
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
TASK6 = (ROOT / "User/action/task6_lap_target.c").read_text(encoding="utf-8")


assert "s_t4_ball_applied_feedforward_us" in MAIN
assert "T4Ball_ApplyFeedforwardIncrement" in MAIN

same_frame_matches = list(re.finditer(
    r"if \(vision_status->sequence == s_t4_ball_last_vision_sequence\)\s*"
    r"\{(?P<body>.*?)\n\s*\}\s*"
    r"s_t4_ball_last_vision_sequence = vision_status->sequence;",
    MAIN,
    re.DOTALL,
))
assert len(same_frame_matches) >= 2, "normal same-frame control branch not found"
same_frame_body = same_frame_matches[-1].group("body")
assert "T4Ball_ApplyFeedforwardIncrement" in same_frame_body
assert "position_control_us +" not in same_frame_body
assert "damping_control_us" not in same_frame_body

assert "CTRL=T6_FRAME_SYNC_SERVO_V7" in TASK6
assert "FSYNC=1" in TASK6
assert "GUARD=%ld" in TASK6

print("TASK6 FRAME-SYNC SERVO V7 VALIDATION PASS")
print("same vision frame -> acceleration feedforward increment only")
print("new vision frame -> one position/velocity closed-loop step")
