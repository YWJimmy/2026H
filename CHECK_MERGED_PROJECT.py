from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent
project = root / "MDK-ARM" / "2026H.uvprojx"
assert project.exists(), "缺少 MDK-ARM/2026H.uvprojx"

tree = ET.parse(project)
missing = []
for node in tree.iter("FilePath"):
    raw = (node.text or "").strip()
    if raw:
        path = (project.parent / raw.replace("\\", "/")).resolve()
        if not path.exists():
            missing.append(raw)

required = [
    root / "User/test/test_ball_balance.c",
    root / "User/test/test_ball_balance.h",
    root / "User/test/test_oled_key.c",
    root / "User/module/task_menu_ui.c",
    root / "host_tests/test_task3_optimized.c",
]
missing.extend(str(path.relative_to(root)) for path in required if not path.exists())

if missing:
    print("PROJECT CHECK FAILED")
    for item in missing:
        print("MISSING:", item)
    raise SystemExit(1)

text = (root / "User/test/test_ball_balance.c").read_text(encoding="utf-8")
menu = (root / "User/test/test_oled_key.c").read_text(encoding="utf-8")
ui = (root / "User/module/task_menu_ui.c").read_text(encoding="utf-8")

for marker in [
    "BALL_TARGET_CX_O                 653",
    "BALL_TARGET_CX_P5                869",
    "BALL_TARGET_CX_N5                447",
    "BALL_RUN_LIMIT_MS               5000U",
    "BALL_STATE_TURN_BRAKE",
    "BALL_STATE_SETTLE_NEG5",
    "ONE_SHOT_PREDICT_CAPTURE",
    "Test_BallBalance_IsFinished",
]:
    assert marker in text, f"缺少任务3优化标记: {marker}"

for marker in [
    "Test_BallBalance_Init",
    "Test_BallBalance_Update",
    "Test_BallBalance_IsFinished",
    "TaskMenuUi_SetFinished",
]:
    assert marker in menu, f"缺少菜单任务3标记: {marker}"

assert "s_stop_request = true" in ui
print("PROJECT CHECK PASS")
