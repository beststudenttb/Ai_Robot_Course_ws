"""Decision layer: choose targets and publish planar velocity commands."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mecanum_patrol_interfaces.action import NavigateTo
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Target:
    name: str
    x: float
    y: float


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("decision")
        self.declare_parameter("pose_topic", "/tracker_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("marker_state_topic", "/target_marker_state")
        self.declare_parameter("target_command_topic", "/target_command")
        self.declare_parameter("action_name", "/navigate_to")
        self.declare_parameter("target_name", "A_point")
        self.declare_parameter("A_point_x", 14.0)
        self.declare_parameter("A_point_y", 0.0)
        self.declare_parameter("B_point_x", 14.0)
        self.declare_parameter("B_point_y", 14.0)
        self.declare_parameter("C_point_x", 0.0)
        self.declare_parameter("C_point_y", 14.0)
        self.declare_parameter("slow_speed", 0.25)
        self.declare_parameter("slowdown_distance", 5.0)
        self.declare_parameter("yellow_distance", 5.0)
        self.declare_parameter("stop_distance", 0.05)

        target_name = self.get_parameter("target_name").value
        self.targets = {
            "A_point": self.target_from_parameters("A_point"),
            "B_point": self.target_from_parameters("B_point"),
            "C_point": self.target_from_parameters("C_point"),
        }
        self.target = self.targets[target_name]
        self.slow_speed = float(self.get_parameter("slow_speed").value)
        self.slowdown_distance = float(self.get_parameter("slowdown_distance").value)
        self.yellow_distance = float(self.get_parameter("yellow_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)

        pose_topic = self.get_parameter("pose_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        marker_state_topic = self.get_parameter("marker_state_topic").value
        target_command_topic = self.get_parameter("target_command_topic").value
        action_name = self.get_parameter("action_name").value
        self.callback_group = ReentrantCallbackGroup()

        self.cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_publisher = self.create_publisher(String, marker_state_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, pose_topic, self.on_pose, 10, callback_group=self.callback_group
        )
        self.target_subscription = self.create_subscription(
            String,
            target_command_topic,
            self.on_target_command,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            NavigateTo,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=self.callback_group,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.has_pose = False
        self.active_target = self.target
        self.action_active = False
        self.keyboard_override = False
        self.get_logger().info(
            "decision initialized default target=%s (%.3f, %.3f)"
            % (self.target.name, self.target.x, self.target.y)
        )

    def target_from_parameters(self, name: str) -> Target:
        return Target(
            name=name,
            x=float(self.get_parameter(f"{name}_x").value),
            y=float(self.get_parameter(f"{name}_y").value),
        )

    def on_target_command(self, msg: String) -> None:
        target_name = msg.data.strip()
        target = self.targets.get(target_name)
        if target is None:
            self.get_logger().warn(f"ignored unknown target '{target_name}'")
            return
        self.keyboard_override = True
        self.set_active_target(target)
        self.get_logger().info(
            "switched target=%s (%.3f, %.3f)" % (target.name, target.x, target.y)
        )

    def goal_callback(self, goal_request: NavigateTo.Goal) -> GoalResponse:
        if self.action_active:
            self.get_logger().warn("rejected action goal because another action is active")
            return GoalResponse.REJECT
        if self.keyboard_override and not self.is_at_active_target():
            self.get_logger().warn("rejected action goal because keyboard target is active")
            return GoalResponse.REJECT
        self.action_active = True
        self.keyboard_override = False
        self.get_logger().info(
            "accepted action target=%s (%.3f, %.3f)"
            % (goal_request.target_name, goal_request.target_x, goal_request.target_y)
        )
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target = Target(
            name=goal_handle.request.target_name,
            x=float(goal_handle.request.target_x),
            y=float(goal_handle.request.target_y),
        )
        if self.keyboard_override:
            goal_handle.abort()
            result = NavigateTo.Result()
            result.success = False
            result.message = "interrupted by keyboard"
            self.action_active = False
            return result

        self.set_active_target(target)
        feedback = NavigateTo.Feedback()
        rate = self.create_rate(10)

        try:
            while rclpy.ok():
                if self.keyboard_override:
                    goal_handle.abort()
                    result = NavigateTo.Result()
                    result.success = False
                    result.message = "interrupted by keyboard"
                    return result

                distance, cmd = self.compute_command(self.active_target)
                self.publish_marker_state(self.active_target, distance)
                feedback.distance_remaining = float(distance)
                feedback.cmd_vx = float(cmd.linear.x)
                feedback.cmd_vy = float(cmd.linear.y)
                goal_handle.publish_feedback(feedback)

                if distance <= self.stop_distance:
                    goal_handle.succeed()
                    result = NavigateTo.Result()
                    result.success = True
                    result.message = "arrived"
                    return result
                rate.sleep()

            result = NavigateTo.Result()
            result.success = False
            result.message = "interrupted"
            return result
        finally:
            self.action_active = False

    def on_pose(self, msg: PoseStamped) -> None:
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.has_pose = True

        distance, cmd = self.compute_command(self.active_target)
        self.cmd_publisher.publish(cmd)
        self.publish_marker_state(self.active_target, distance)
        if self.keyboard_override and distance <= self.stop_distance:
            self.keyboard_override = False
            self.get_logger().info("keyboard target arrived")
        self.get_logger().info(
            "target=%s euclidean=%.3f manhattan=%.3f cmd vx=%.3f vy=%.3f"
            % (
                self.active_target.name,
                distance,
                abs(self.active_target.x - self.current_x)
                + abs(self.active_target.y - self.current_y),
                cmd.linear.x,
                cmd.linear.y,
            )
        )

    def set_active_target(self, target: Target) -> None:
        self.active_target = target

    def is_at_active_target(self) -> bool:
        if not self.has_pose:
            return False
        distance, _ = self.compute_command(self.active_target)
        return distance <= self.stop_distance

    def compute_command(self, target: Target) -> tuple[float, Twist]:
        error_x = target.x - self.current_x
        error_y = target.y - self.current_y
        euclidean_distance = math.hypot(error_x, error_y)

        cmd = Twist()
        if not self.has_pose or euclidean_distance <= self.stop_distance:
            return euclidean_distance, cmd

        abs_x = abs(error_x)
        abs_y = abs(error_y)
        long_side = max(abs_x, abs_y)
        if long_side <= 0.0:
            return euclidean_distance, cmd

        speed = self.speed_for_distance(euclidean_distance)
        cmd.linear.x = math.copysign(speed * abs_x / long_side, error_x)
        cmd.linear.y = math.copysign(speed * abs_y / long_side, error_y)
        return euclidean_distance, cmd

    def speed_for_distance(self, distance: float) -> float:
        min_speed = self.slow_speed * 0.25
        if distance >= self.slowdown_distance:
            return self.slow_speed
        ratio = max(0.0, distance / self.slowdown_distance)
        return min_speed + (self.slow_speed - min_speed) * ratio

    def publish_marker_state(self, target: Target, distance: float) -> None:
        marker = String()
        if distance <= self.stop_distance:
            marker.data = f"{target.name}:red"
        elif distance < self.yellow_distance:
            marker.data = f"{target.name}:yellow"
        else:
            marker.data = f"{target.name}:green"
        self.marker_publisher.publish(marker)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
