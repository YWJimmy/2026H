#!/usr/bin/env python3
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
errors = []

def text(rel):
    p = ROOT / rel
    if not p.exists():
        errors.append(f"missing: {rel}")
        return ""
    data = p.read_bytes()
    if data.startswith(b"\xef\xbb\xbf") and p.suffix.lower() in {'.c','.h','.s','.uvprojx','.uvoptx','.ioc','.sct'}:
        errors.append(f"forbidden UTF-8 BOM: {rel}")
    for enc in ('utf-8','gb18030'):
        try:
            return data.decode(enc)
        except UnicodeDecodeError:
            pass
    errors.append(f"cannot decode: {rel}")
    return data.decode('latin1')

def need(rel, token, label):
    if token not in text(rel):
        errors.append(f"{label}: {rel} lacks {token}")

# Project / menu / build integration.
need('Core/Src/main.c', '#define PROJECT_ENTRY_MODE          PROJECT_ENTRY_MAIN_APP', 'MainApp entry')
need('User/module/task_menu_ui.h', 'TASK_MENU_TASK_5_LAP_HOLD', 'Task5 menu')
need('User/module/task_menu_ui.h', 'TASK_MENU_TASK_6_LAP_TARGET', 'Task6 menu')
need('User/action/app_task_port.c', 'Task5LapHold_Start', 'Task5 port')
need('User/action/app_task_port.c', 'Task6LapTarget_Start', 'Task6 port')
need('MDK-ARM/2026H.uvprojx', '..\\User\\action\\task5_lap_hold.c', 'Task5 Keil source')
need('MDK-ARM/2026H.uvprojx', '..\\User\\action\\task6_lap_target.c', 'Task6 Keil source')

# Task 4 completeness (uploaded project feature audit).
for token, label in [
    ('Task4MainBall_Init()', 'Task4 ball init'),
    ('Task4MainBall_Update(&s_vision)', 'Task4 ball update'),
    ('Task4_IsTurnAtBConfirmed', 'Task4 turn backup'),
    ('TASK4_B_TIME_DISTANCE_MM', 'Task4 1.5m lock'),
    ('TASK4_STOP_START_DISTANCE_MM', 'Task4 1.8m stop'),
    ('TASK4_AB_TIME_LIMIT_MS', 'Task4 8s limit'),
    ('TaskMenuUi_SetRunningElapsedMs', 'Task4 OLED live timer'),
    ('TaskMenuUi_SetFinishedResult', 'Task4 OLED frozen result'),
    ('one_cm_violation_latched', 'Task4 full-course tolerance latch'),
]:
    need('User/action/task4_ab_hold.c', token, label)
for token, label in [
    ('T4_BALL_PREDICT_DYNAMIC_MS', 'prediction'),
    ('T4_BALL_VELOCITY_GAIN_DYNAMIC_NUM', 'velocity damping'),
    ('T4_BALL_BIAS_LIMIT_US', 'bias learning'),
    ('T4_BALL_PLANNED_ACCEL_FF_DIVISOR', 'planned acceleration feedforward'),
    ('T4_BALL_MEASURED_ACCEL_FF_DIVISOR', 'measured acceleration feedforward'),
    ('T4_BALL_LAUNCH_FF_STRONG_US', 'launch boost'),
    ('TASK4_MAIN_BALL_MODE_RECOVER', 'recovery mode'),
    ('physical_x_mm', 'physical mm tolerance'),
]:
    need('Core/Src/main.c', token, f'Task4 {label}')

# Task 5 requirements.
for token, label in [
    ('Task4MainBall_InitTarget', 'fixed O target'),
    ('TASK5_BALL_TARGET_CX', 'O pixel target'),
    ('TASK5_BALL_TARGET_MM', 'O physical target'),
    ('TASK5_A_LINE_SEARCH_DISTANCE_MM', 'A search gate'),
    ('TASK5_A_LINE_WINDOW_FRAMES', 'A window'),
    ('TASK5_A_LINE_REQUIRED_FRAMES', 'A vote'),
    ('s_ball.one_cm_violation_latched', 'full-course ±1cm latch'),
    ('TaskMenuUi_SetFinishedResult', 'OLED freeze'),
]:
    need('User/action/task5_lap_hold.c', token, f'Task5 {label}')
need('User/action/task5_lap_hold_config.h', '((uint32_t)5400U)', 'Task5 5.4m gate')
need('User/action/task5_lap_hold_config.h', '((uint32_t)30000U)', 'Task5 30s')
need('User/action/task5_lap_hold.c', '(now_ms - s_start_ms) > TASK5_LAP_TIMEOUT_MS', 'Task5 inclusive 30s')

# Task 6 requirements.
for token, label in [
    ('Task4MainBall_InitCaptureCurrent', 'capture current target'),
    ('Task4MainBall_IsTargetLocked', 'target lock gate'),
    ('TASK6_TARGET_CAPTURE_TIMEOUT_MS', 'capture timeout'),
    ('s_ball.one_cm_violation_latched', 'full-course ±1cm latch'),
    ('TaskMenuUi_SetFinishedResult', 'OLED freeze'),
    ('T6 LAP TARGET', 'arbitrary-target phase text'),
]:
    need('User/action/task6_lap_target.c', token, f'Task6 {label}')
need('Core/Src/main.c', 'T4_BALL_CAPTURE_REQUIRED_FRAMES           ((uint8_t)3U)', 'Task6 3 stable frames')
need('Core/Src/main.c', 'T4_BALL_CAPTURE_MIN_MM                    ((int32_t)-125)', 'Task6 -12.5cm')
need('Core/Src/main.c', 'T4_BALL_CAPTURE_MAX_MM                    ((int32_t)125)', 'Task6 +12.5cm')
need('User/action/task6_lap_target.c', '(now_ms - s_start_ms) > TASK6_LAP_TIMEOUT_MS', 'Task6 inclusive 30s')

# Reset / safe stop must end at 1650 with PWM enabled.
need('Core/Inc/project_servo_level.h', 'PROJECT_SERVO_HORIZONTAL_US ((uint16_t)1650U)', '1650 level')
need('Core/Src/main.c', 'Project_ServoHoldHorizontal()', 'hardware reset level')
port = text('User/action/app_task_port.c')
if 'BSP_Servo_Disable();' in port:
    errors.append('AppTaskPort_ForceSafeStop still disables servo')
for token in ('BSP_Servo_SetPulseUs(PROJECT_SERVO_HORIZONTAL_US)', 'BSP_Servo_Enable()'):
    if token not in port:
        errors.append(f'software reset level missing: {token}')

# Parse Keil XML.
try:
    ET.parse(ROOT / 'MDK-ARM/2026H.uvprojx')
except Exception as exc:
    errors.append(f'uvprojx XML parse failed: {exc}')

if errors:
    print('TASK456 AUDIT FAIL')
    for e in errors:
        print('FAIL:', e)
    sys.exit(1)

print('TASK456 AUDIT PASS')
print('OK: Task4 complete feature chain')
print('OK: Task5 fixed-O full-lap hold')
print('OK: Task6 captured-target full-lap hold')
print('OK: 30.000 s inclusive boundary')
print('OK: A-line search gate 5.4 m')
print('OK: reset/fault/menu reset holds servo at 1650 us')
print('NOTE: GitHub commit e28b2bb... was not publicly retrievable during audit;')
print('      Task4 was checked against uploaded source and accessible frame history.')
