#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
required = [
    ROOT / "Core/Inc/task4_main_ball.h",
    ROOT / "Core/Src/main.c",
    ROOT / "User/action/task4_ab_hold.c",
    ROOT / "User/action/task4_ab_hold.h",
    ROOT / "User/action/task4_ab_hold_config.h",
    ROOT / "User/module/task_menu_ui.c",
    ROOT / "User/module/task_menu_ui.h",
    ROOT / "User/bsp/bsp_oled.c",
    ROOT / "User/bsp/bsp_oled.h",
]
def decode_source(path: Path) -> str:
    data = path.read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        raise SystemExit(f"unexpected UTF-8 BOM: {path.relative_to(ROOT)}")
    for encoding in ("utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    raise SystemExit(f"unsupported source encoding: {path.relative_to(ROOT)}")

for path in required:
    if not path.is_file():
        raise SystemExit(f"missing: {path.relative_to(ROOT)}")
    decode_source(path)

main = decode_source(ROOT / "Core/Src/main.c")
t4 = decode_source(ROOT / "User/action/task4_ab_hold.c")
cfg = decode_source(ROOT / "User/action/task4_ab_hold_config.h")
ui = decode_source(ROOT / "User/module/task_menu_ui.c")
oled_c = decode_source(ROOT / "User/bsp/bsp_oled.c")
oled_h = decode_source(ROOT / "User/bsp/bsp_oled.h")

checks = {
    "formal MainApp entry": "PROJECT_ENTRY_MODE          PROJECT_ENTRY_MAIN_APP" in main,
    "controller remains in main.c": "bool Task4MainBall_Update(" in main,
    "observed launch direction corrected": "T4_BALL_ACCEL_FF_SIGN                      ((int32_t)-1)" in main,
    "launch feedforward floor": "T4_BALL_LAUNCH_FF_STRONG_US" in main and "140" in main,
    "launch-only state": "s_t4_ball_launch_active" in main,
    "launch fast slew": "T4_BALL_LAUNCH_FF_STEP_STRONG_US" in main and "90" in main,
    "normal road controller preserved": "T4_BALL_PREDICT_DYNAMIC_MS" in main,
    "v3 marker": "CTRL=MAIN_LAUNCH_FF_OLED_V3" in t4,
    "distance timing lock": "TASK4_AB_LOCK_DISTANCE" in t4 and "TASK4_B_TIME_DISTANCE_MM" in t4,
    "turn timing lock": "TASK4_AB_LOCK_TURN" in t4 and "Task4_IsTurnAtBConfirmed" in t4,
    "turn distance gate": "TASK4_B_TURN_MIN_DISTANCE_MM" in cfg and "1100U" in cfg,
    "turn threshold": "TASK4_B_TURN_THRESHOLD_MM_S" in cfg and "80" in cfg,
    "turn confirmation": "TASK4_B_TURN_CONFIRM_MS" in cfg and "60U" in cfg,
    "OLED task4 live timer": "TASK4 A-B" in ui and "TASK_MENU_TASK_4_AB_HOLD" in ui,
    "OLED B lock display": "B LOCK K0" in ui,
    "OLED frozen result": "TaskMenuUi_SetFinishedResult" in t4,
    "large digit drawing": "TaskMenu_DrawLargeTime" in ui,
    "OLED pixel API implementation": "void BSP_Oled_FillRect(" in oled_c,
    "OLED pixel API declaration": "void BSP_Oled_FillRect(" in oled_h,
    "AB limit unchanged": "TASK4_AB_TIME_LIMIT_MS" in cfg and "8000U" in cfg,
    "distance target unchanged": "TASK4_B_TIME_DISTANCE_MM" in cfg and "1500U" in cfg,
    "stop distance unchanged": "TASK4_STOP_START_DISTANCE_MM" in cfg and "1800U" in cfg,
}
failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(("PASS" if ok else "FAIL") + ": " + name)
if failed:
    raise SystemExit(1)

for forbidden in [
    "User/module/line_follow.c",
    "User/module/line_follow_control.c",
    "User/module/line_sensor.c",
    "User/module/chassis.c",
    "User/module/wheel_speed_control.c",
]:
    if (ROOT / forbidden).exists():
        raise SystemExit(f"line-follow file unexpectedly included: {forbidden}")
print("PASS: frame line-follow/chassis source is not overwritten")
print("task4 launch feedforward + OLED A-B timer v3 validation: PASS")
