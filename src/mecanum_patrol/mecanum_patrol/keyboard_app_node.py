"""Keyboard: A/B/C -> /target_command, r -> /reset, p -> /pause."""

from __future__ import annotations

import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HELP = """
  A/B/C -> named target
  r     -> reset to origin
  p     -> toggle pause
  Ctrl-C -> quit
"""


class KeyboardAppNode(Node):

    def __init__(self) -> None:
        super().__init__("keyboard_app")
        self.pub = self.create_publisher(String, "/target_command", 10)
        self.reset_pub = self.create_publisher(String, "/reset", 10)
        self.pause_pub = self.create_publisher(String, "/pause", 10)
        self.get_logger().info("keyboard ready")
        print(HELP)

    def publish(self, key: str) -> None:
        key = key.upper()
        msg = String()
        if key == "R":
            self.reset_pub.publish(msg)
            self.get_logger().info("reset")
        elif key == "P":
            self.pause_pub.publish(msg)
            self.get_logger().info("pause")
        elif key in ("A", "B", "C"):
            msg.data = f"{key}_point"
            self.pub.publish(msg)
            self.get_logger().info(f"target {msg.data}")


def main():
    rclpy.init()
    node = KeyboardAppNode()

    f = open("/dev/tty")
    old = termios.tcgetattr(f)
    tty.setcbreak(f.fileno())

    while rclpy.ok():
        ready, _, _ = select.select([f], [], [], 0.1)
        if ready:
            key = f.read(1)
            if key == "\x03":
                break
            node.publish(key)
        rclpy.spin_once(node, timeout_sec=0.0)

    termios.tcsetattr(f, termios.TCSADRAIN, old)
    f.close()
    node.destroy_node()
    rclpy.shutdown()
