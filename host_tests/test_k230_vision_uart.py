import ast
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "k230d-steel-ball"
    / "steel_ball_yolov8_v2_full_w8a8_stable.py"
)


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
    )
    for fragment in required_fragments:
        assert fragment in source, "missing UART configuration: %s" % fragment

    print("k230_vision_uart: PASS")


if __name__ == "__main__":
    main()
