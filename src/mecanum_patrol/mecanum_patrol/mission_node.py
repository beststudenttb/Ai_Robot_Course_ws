"""Mission layer: sequence patrol + keyboard override via action."""

from __future__ import annotations

import rclpy
from mecanum_patrol_interfaces.action import NavigateTo
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class MissionNode(Node):

    def __init__(self) -> None:
        super().__init__("mission")

        self.declare_parameter("target_sequence", ["A_point", "B_point", "C_point"])
        self.sequence = self.get_parameter("target_sequence").value

        self.client = ActionClient(self, NavigateTo, "/navigate_to")
        self.create_subscription(String, "/target_command", self.on_keyboard, 10)

        self.keyboard_name = None
        self.idx = 0
        self.get_logger().info(f"mission init sequence={self.sequence}")

    def on_keyboard(self, msg: String) -> None:
        self.keyboard_name = msg.data.strip()
        self.get_logger().info(f"keyboard: {self.keyboard_name}")

    def run(self) -> None:
        while rclpy.ok():
            if self.keyboard_name is not None:
                name = self.keyboard_name
                self.keyboard_name = None
                from_keyboard = True
            else:
                name = self.sequence[self.idx]
                from_keyboard = False

            self.client.wait_for_server(timeout_sec=1.0)
            goal = NavigateTo.Goal()
            goal.target_name = name
            self.get_logger().info(f"send action: {name}")

            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.get_logger().warn("action rejected")
                continue

            result_future = handle.get_result_async()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if result_future.done():
                    break
                if self.keyboard_name is not None:
                    break

            if self.keyboard_name is not None:
                continue

            result = result_future.result()
            if result is not None and result.result.success:
                self.get_logger().info("action arrived")
                if not from_keyboard:
                    self.idx = (self.idx + 1) % len(self.sequence)
            else:
                self.get_logger().warn("action failed")


def main():
    rclpy.init()
    node = MissionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
