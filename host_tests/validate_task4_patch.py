#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
required = [
    ROOT / "Core/Inc/task4_main_ball.h",
    ROOT / "Core/Src/main.c",
    ROOT / "User/action/task4_ab_hold.c",
    ROOT / "User/action/task4_ab_hold.h",
    ROOT / "User/action/task4_ab_hold_config.h",
]
for path in required:
    if not path.is_file():
        raise SystemExit(f"missing: {path.relative_to(ROOT)}")
    data = path.read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        raise SystemExit(f"unexpected UTF-8 BOM: {path.relative_to(ROOT)}")
    data.decode("utf-8")

main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
t4 = (ROOT / "User/action/task4_ab_hold.c").read_text(encoding="utf-8")
cfg = (ROOT / "User/action/task4_ab_hold_config.h").read_text(encoding="utf-8")

checks = {
    "formal MainApp entry": "PROJECT_ENTRY_MODE          PROJECT_ENTRY_MAIN_APP" in main,
    "controller implemented in main": "bool Task4MainBall_Update(" in main,
    "O center calibration": "T4_BALL_TARGET_CX" in main and "((int32_t)653)" in main,
    "+/-1 cm threshold": "T4_BALL_ONE_CM_PX" in main and "((int32_t)43)" in main,
    "1700 us level pulse": "T4_BALL_SERVO_CENTER_US" in main and "((int32_t)1700)" in main,
    "time-normalized velocity": "raw_velocity_px_s" in main and "vision_dt_ms" in main,
    "predicted position": "predicted_error_px" in main and "T4_BALL_PREDICT_DYNAMIC_MS" in main,
    "planned acceleration feedforward": "T4_BALL_PLANNED_ACCEL_FF_DIVISOR" in main,
    "measured acceleration feedforward": "T4_BALL_MEASURED_ACCEL_FF_DIVISOR" in main,
    "strong recovery range": "T4_BALL_SERVO_MIN_US" in main and "((int32_t)1320)" in main,
    "recovery state": "TASK4_MAIN_BALL_MODE_RECOVER" in main,
    "strict excursion latch": "one_cm_violation_latched = true" in main,
    "servo armed before chassis": t4.index("Task4MainBall_Init()") < t4.index("LineFollowControl_Start()"),
    "task4 calls main controller": "Task4MainBall_Update(&s_vision)" in t4,
    "v2 report marker": "CTRL=MAIN_PREDICTIVE_FF_V2" in t4,
    "AB time limit": "TASK4_AB_TIME_LIMIT_MS" in cfg and "8000U" in cfg,
    "B distance unchanged": "TASK4_B_TIME_DISTANCE_MM" in cfg and "1500U" in cfg,
    "stop distance unchanged": "TASK4_STOP_START_DISTANCE_MM" in cfg and "1800U" in cfg,
}
failed = [name for name, ok in checks.items() if not ok]
if failed:
    for name in failed:
        print(f"FAIL: {name}")
    raise SystemExit(1)
for name in checks:
    print(f"PASS: {name}")

for forbidden in [
    "User/module/line_follow.c",
    "User/module/line_follow_control.c",
    "User/module/line_sensor.c",
    "User/module/chassis.c",
    "User/module/wheel_speed_control.c",
]:
    if (ROOT / forbidden).exists():
        raise SystemExit(f"line-follow file unexpectedly included: {forbidden}")
print("PASS: no frame line-follow/chassis source is overwritten")
print("task4 predictive feedforward v2 validation: PASS")
