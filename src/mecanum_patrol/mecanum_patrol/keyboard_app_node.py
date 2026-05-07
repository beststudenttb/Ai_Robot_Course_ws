"""Keyboard command client: publish A/B/C target choices."""

from __future__ import annotations

import select
import termios
import tty
from typing import Optional, TextIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = """
Target command:
  A -> A_point
  B -> B_point
  C -> C_point
  Ctrl-C: quit
"""


class KeyboardAppNode(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_app")
        self.declare_parameter("target_command_topic", "/target_command")

        topic = self.get_parameter("target_command_topic").value
        self.publisher = self.create_publisher(String, topic, 10)

        self.get_logger().info(f"keyboard target client publishing to {topic}")
        print(HELP)

    def publish_key(self, key: str) -> None:
        key = key.upper()
        if key not in {"A", "B", "C"}:
            return
        msg = String()
        msg.data = f"{key}_point"
        self.publisher.publish(msg)
        self.get_logger().info(f"sent target {msg.data}")


def read_key(input_stream: TextIO, timeout: float = 0.1) -> str:
    ready, _, _ = select.select([input_stream], [], [], timeout)
    if not ready:
        return ""
    return input_stream.read(1)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = KeyboardAppNode()

    input_stream = open("/dev/tty", "r", encoding="utf-8")
    old_settings = termios.tcgetattr(input_stream)
    try:
        tty.setcbreak(input_stream.fileno())
        while rclpy.ok():
            key = read_key(input_stream)
            if key == "\x03":
                break
            node.publish_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(input_stream, termios.TCSADRAIN, old_settings)
        input_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
