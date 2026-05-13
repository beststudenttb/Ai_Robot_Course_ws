"""Perception layer: Webots Supervisor, publish pose + tf, marker color, reset."""

import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from mecanum_patrol_interfaces.srv import GetPose
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))
from controller import Supervisor  # noqa: E402

COLORS = {
    "white": [1.0, 1.0, 1.0],
    "green": [0.0, 1.0, 0.0],
    "yellow": [1.0, 1.0, 0.0],
    "red": [1.0, 0.0, 0.0],
}


class PerceptionNode(Node):

    def __init__(self) -> None:
        super().__init__("perception")

        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0

        self.pub = self.create_publisher(PoseStamped, "/tracker_pose", 10)
        self.state_pub = self.create_publisher(Twist, "/tracker_state", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, "/target_marker_state", self.on_marker, 10)
        self.create_subscription(String, "/reset", self.on_reset, 10)
        self.create_service(GetPose, "/get_pose", self.on_get_pose)

        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        self.tracker = self.supervisor.getFromDef("tracker")
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.create_timer(0.5, self.on_log)
        self.get_logger().info("perception ready")

    def on_marker(self, msg: String) -> None:
        if ":" not in msg.data:
            return
        name, c = msg.data.split(":", 1)
        color = COLORS.get(c)
        if color is None:
            return
        for p in ("A_point", "B_point", "C_point"):
            self.set_color(p, COLORS["white"])
        self.set_color(name, color)

    def on_reset(self, msg: String) -> None:
        self.tracker.getField("translation").setSFVec3f([self.init_x, self.init_y, 0.0])
        self.tracker.getField("rotation").setSFRotation([0.0, 0.0, 1.0, self.init_yaw])
        for p in ("A_point", "B_point", "C_point"):
            self.set_color(p, COLORS["white"])
        self.get_logger().info("reset")

    def set_color(self, name: str, color: list[float]) -> None:
        node = self.supervisor.getFromDef(name)
        if node is None:
            return
        mat = node.getField("material").getSFNode()
        mat.getField("diffuseColor").setSFColor(color)

    def step(self) -> bool:
        return self.supervisor.step(self.timestep) != -1

    def on_get_pose(self, request, response):
        response.x = float(self.x)
        response.y = float(self.y)
        response.yaw = float(self.yaw)
        return response

    def on_log(self) -> None:
        self.get_logger().info(
            f"pose x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}")

    def publish(self) -> None:
        pos = self.tracker.getPosition()
        ori = self.tracker.getOrientation()
        yaw = math.atan2(ori[3], ori[0])
        self.x = pos[0]
        self.y = pos[1]
        self.yaw = yaw
        h = yaw * 0.5
        qx, qy, qz, qw = 0.0, 0.0, math.sin(h), math.cos(h)
        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "world"
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub.publish(pose)

        state = Twist()
        state.linear.x = pos[0]
        state.linear.y = pos[1]
        state.angular.z = yaw
        self.state_pub.publish(state)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "tracker"
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = PerceptionNode()
    while rclpy.ok() and node.step():
        node.publish()
        rclpy.spin_once(node, timeout_sec=0.0)
    node.destroy_node()
    rclpy.shutdown()
