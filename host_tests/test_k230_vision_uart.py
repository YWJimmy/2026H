import ast
from pathlib import Path
import runpy


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "k230d-steel-ball"
    / "steel_ball_yolov8_v2_full_w8a8_stable.py"
)
TRACKER_SCRIPT = SCRIPT.with_name("steel_ball_temporal_tracker.py")


def assigned_integer(module, name):
    for node in module.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id == name:
            return ast.literal_eval(node.value)
    raise AssertionError("missing assignment: %s" % name)


def main():
    source = SCRIPT.read_text(encoding="utf-8")
    module = ast.parse(source, filename=str(SCRIPT))

    assert assigned_integer(module, "VISION_UART_TX_PIN") == 40
    assert assigned_integer(module, "VISION_UART_RX_PIN") == 41
    assert assigned_integer(module, "VISION_UART_BAUDRATE") == 115200

    required_fragments = (
        "FPIOA.UART1_TXD",
        "FPIOA.UART1_RXD",
        "UART.UART1",
        "UART.EIGHTBITS",
        "UART.PARITY_NONE",
        "UART.STOPBITS_ONE",
        "uart.write(line)",
        "\\r\\n",
        "def _draw_pipe_roi(self, pipeline):",
        "roi_color = (255, 255, 255, 0)",
        "self._draw_pipe_roi(pipeline)",
    )
    for fragment in required_fragments:
        assert fragment in source, "missing UART configuration: %s" % fragment

    roi_draw_index = source.index("self._draw_pipe_roi(pipeline)")
    no_detection_index = source.index("if detection is None:", roi_draw_index)
    assert roi_draw_index < no_detection_index
    assert source.count("color=roi_color") == 4

    tracker = runpy.run_path(str(TRACKER_SCRIPT))
    assert tracker["PIPE_ROI_X_MIN"] == 100
    assert tracker["PIPE_ROI_X_MAX"] == 1200
    assert tracker["PIPE_CENTER_SLOPE"] == -0.034
    assert tracker["PIPE_CENTER_INTERCEPT"] == 437.0
    assert tracker["PIPE_ROI_HALF_HEIGHT"] == 70.0

    valid_geometry = tracker["_valid_geometry"]

    def roi_detection(center_x, center_offset_y=0.0):
        center_y = (
            tracker["PIPE_CENTER_INTERCEPT"]
            + tracker["PIPE_CENTER_SLOPE"] * center_x
            + center_offset_y
        )
        return [
            center_x - 20,
            center_y - 20,
            center_x + 20,
            center_y + 20,
            0.5,
        ]

    assert valid_geometry(roi_detection(100))
    assert valid_geometry(roi_detection(1200))
    assert not valid_geometry(roi_detection(99))
    assert not valid_geometry(roi_detection(1201))
    assert valid_geometry(roi_detection(653, 70.0))
    assert not valid_geometry(roi_detection(653, 70.1))

    print("k230_vision_uart_and_roi: PASS")


if __name__ == "__main__":
    main()
