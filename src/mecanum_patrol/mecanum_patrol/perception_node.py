"""Perception layer running as the Webots extern controller named `super`."""

from __future__ import annotations

import math
import os
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

webots_home = os.environ.get("WEBOTS_HOME", "/usr/local/webots")
webots_controller_python = os.path.join(webots_home, "lib", "controller", "python")
if webots_controller_python not in sys.path:
    sys.path.insert(0, webots_controller_python)

from controller import Supervisor


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def yaw_from_webots_orientation(orientation: list[float]) -> float:
    return math.atan2(orientation[3], orientation[0])


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception")
        self.declare_parameter("pose_topic", "/tracker_pose")
        self.declare_parameter("state_topic", "/tracker_state")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("tracker_frame", "tracker")
        self.declare_parameter("marker_state_topic", "/target_marker_state")
        self.declare_parameter("log_period_sec", 1.0)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.tracker_frame = self.get_parameter("tracker_frame").value
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.state_publisher = self.create_publisher(Twist, self.state_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        marker_state_topic = self.get_parameter("marker_state_topic").value
        self.marker_subscription = self.create_subscription(
            String, marker_state_topic, self.on_marker_state, 10
        )
        self.supervisor = None
        self.self_node = None
        self.timestep = 32
        self.last_log_time = 0.0
        self._connect_webots()
        self.get_logger().info("perception initialized")

    def _connect_webots(self) -> None:
        if Supervisor is None:
            self.get_logger().error("Webots Supervisor module not found")
            return
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        self.self_node = self.supervisor.getSelf()

    def on_marker_state(self, msg: String) -> None:
        if self.supervisor is None or ":" not in msg.data:
            return
        target_name, color_name = msg.data.split(":", 1)
        colors = {
            "white": [1.0, 1.0, 1.0],
            "green": [0.0, 1.0, 0.0],
            "yellow": [1.0, 1.0, 0.0],
            "red": [1.0, 0.0, 0.0],
        }
        color = colors.get(color_name)
        if color is None:
            return

        for point_name in ("A_point", "B_point", "C_point"):
            self.set_appearance_color(point_name, [1.0, 1.0, 1.0])
        self.set_appearance_color(target_name, color)

    def set_appearance_color(self, target_name: str, color: list[float]) -> None:
        appearance = self.supervisor.getFromDef(target_name)
        if appearance is None:
            self.get_logger().warn(f"target appearance '{target_name}' not found")
            return
        material = appearance.getField("material").getSFNode()
        material.getField("diffuseColor").setSFColor(color)

    def step(self) -> bool:
        if self.supervisor is None or self.self_node is None:
            return False
        return self.supervisor.step(self.timestep) != -1

    def publish_pose(self) -> None:
        position = self.self_node.getPosition()
        yaw = yaw_from_webots_orientation(self.self_node.getOrientation())
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.publisher.publish(pose)

        state = Twist()
        state.linear.x = position[0]
        state.linear.y = position[1]
        state.angular.z = yaw
        self.state_publisher.publish(state)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.tracker_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log_time >= self.log_period_sec:
            self.last_log_time = now
            self.get_logger().info(
                "tracker pose x=%.3f y=%.3f yaw=%.3f"
                % (position[0], position[1], yaw)
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        while rclpy.ok() and node.step():
            node.publish_pose()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
