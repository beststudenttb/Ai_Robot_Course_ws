"""Default mission layer: send A/B/C navigation goals through the action API."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from mecanum_patrol_interfaces.action import NavigateTo
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass
class Target:
    name: str
    x: float
    y: float


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission")
        self.declare_parameter("action_name", "/navigate_to")
        self.declare_parameter("target_sequence", ["A_point", "B_point", "C_point"])
        self.declare_parameter("A_point_x", 14.0)
        self.declare_parameter("A_point_y", 0.0)
        self.declare_parameter("B_point_x", 14.0)
        self.declare_parameter("B_point_y", 14.0)
        self.declare_parameter("C_point_x", 0.0)
        self.declare_parameter("C_point_y", 14.0)
        self.declare_parameter("retry_delay", 1.0)

        action_name = self.get_parameter("action_name").value
        self.retry_delay = float(self.get_parameter("retry_delay").value)
        self.targets = {
            "A_point": self.target_from_parameters("A_point"),
            "B_point": self.target_from_parameters("B_point"),
            "C_point": self.target_from_parameters("C_point"),
        }
        self.sequence = [
            self.targets[name]
            for name in self.get_parameter("target_sequence").value
            if name in self.targets
        ]
        self.action_client = ActionClient(self, NavigateTo, action_name)
        self.get_logger().info(
            "mission initialized sequence=%s"
            % ",".join(target.name for target in self.sequence)
        )

    def target_from_parameters(self, name: str) -> Target:
        return Target(
            name=name,
            x=float(self.get_parameter(f"{name}_x").value),
            y=float(self.get_parameter(f"{name}_y").value),
        )

    def run(self) -> None:
        if not self.sequence:
            self.get_logger().error("mission has no valid targets")
            return

        index = 0
        while rclpy.ok():
            target = self.sequence[index]
            if self.send_goal_and_wait(target):
                index = (index + 1) % len(self.sequence)
            else:
                time.sleep(self.retry_delay)

    def send_goal_and_wait(self, target: Target) -> bool:
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("waiting for decision action server")
            return False

        goal = NavigateTo.Goal()
        goal.target_name = target.name
        goal.target_x = float(target.x)
        goal.target_y = float(target.y)

        self.get_logger().info(
            "send action target=%s (%.3f, %.3f)" % (target.name, target.x, target.y)
        )
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("action target=%s rejected" % target.name)
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().warn("action target=%s returned no result" % target.name)
            return False
        if not result.result.success:
            self.get_logger().warn(
                "action target=%s failed: %s" % (target.name, result.result.message)
            )
            return False

        self.get_logger().info("action target=%s arrived" % target.name)
        return True


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
