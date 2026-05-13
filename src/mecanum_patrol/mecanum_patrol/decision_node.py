"""Decision layer: resolve target_name, compute velocity, publish cmd_vel."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mecanum_patrol_interfaces.action import NavigateTo
from mecanum_patrol_interfaces.srv import GetTarget
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNode(Node):

    def __init__(self) -> None:
        super().__init__("decision")

        self.declare_parameter("default_target", "A_point")
        self.declare_parameter("slow_speed", 0.25)
        self.declare_parameter("slowdown_distance", 5.0)
        self.declare_parameter("yellow_distance", 7.0)
        self.declare_parameter("stop_distance", 0.05)
        for name in ("A_point", "B_point", "C_point"):
            self.declare_parameter(f"{name}_x", 14.0)
            self.declare_parameter(f"{name}_y", 0.0)

        self.named = {}
        for name in ("A_point", "B_point", "C_point"):
            self.named[name] = (
                float(self.get_parameter(f"{name}_x").value),
                float(self.get_parameter(f"{name}_y").value),
            )

        self.slow_speed = float(self.get_parameter("slow_speed").value)
        self.slowdown = float(self.get_parameter("slowdown_distance").value)
        self.yellow_dist = float(self.get_parameter("yellow_distance").value)
        self.stop_dist = float(self.get_parameter("stop_distance").value)

        default = self.get_parameter("default_target").value
        self.target_name = default
        self.target_x, self.target_y = self.named[default]

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(String, "/target_marker_state", 10)
        self.create_subscription(PoseStamped, "/tracker_pose", self.on_pose, 10, callback_group=cb)
        self.create_service(GetTarget, "/get_target", self.on_get_target)
        self.action_server = ActionServer(
            self, NavigateTo, "/navigate_to",
            execute_callback=self.on_execute,
            goal_callback=self.on_goal, callback_group=cb)

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.has_pose = False
        self.active = False
        self.current_handle = None
        self.last_cmd = Twist()
        self.create_timer(0.5, self.on_log)
        self.get_logger().info(f"decision init default={default} ({self.target_x:.2f}, {self.target_y:.2f})")

    def on_pose(self, msg: PoseStamped) -> None:
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        self.has_pose = True
        dist, cmd = self.compute()
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.marker(dist)

    def on_get_target(self, request, response):
        response.cmd_vx = float(self.last_cmd.linear.x)
        response.cmd_vy = float(self.last_cmd.linear.y)
        response.target_name = self.target_name
        return response

    def on_log(self) -> None:
        dist, cmd = self.compute()
        self.get_logger().info(
            f"target={self.target_name} dist={dist:.3f} vx={cmd.linear.x:.3f} vy={cmd.linear.y:.3f}")

    def on_goal(self, req: NavigateTo.Goal) -> GoalResponse:
        self.active = True
        return GoalResponse.ACCEPT

    def on_execute(self, handle):
        if self.current_handle is not None and self.current_handle.is_active:
            self.current_handle.abort()
        self.current_handle = handle

        name = handle.request.target_name.strip()
        self.target_name = name
        self.target_x, self.target_y = self.named[name]

        feedback = NavigateTo.Feedback()
        rate = self.create_rate(10)

        while rclpy.ok() and handle.is_active:
            dist, cmd = self.compute()
            self.last_cmd = cmd
            self.marker(dist)
            feedback.distance_remaining = float(dist)
            feedback.cmd_vx = float(cmd.linear.x)
            feedback.cmd_vy = float(cmd.linear.y)
            handle.publish_feedback(feedback)
            if dist <= self.stop_dist:
                handle.succeed()
                self.active = False
                r = NavigateTo.Result()
                r.success = True
                r.message = "arrived"
                return r
            rate.sleep()

        self.active = False
        r = NavigateTo.Result()
        r.success = False
        r.message = "interrupted"
        return r

    def compute(self) -> tuple[float, Twist]:
        ex = self.target_x - self.cur_x
        ey = self.target_y - self.cur_y
        dist = math.hypot(ex, ey)
        cmd = Twist()
        if not self.has_pose or dist <= self.stop_dist:
            return dist, cmd
        L = max(abs(ex), abs(ey))
        if dist >= self.slowdown:
            spd = self.slow_speed
        else:
            spd = self.slow_speed * 0.25 + self.slow_speed * 0.75 * (dist / self.slowdown)
        cmd.linear.x = math.copysign(spd * abs(ex) / L, ex)
        cmd.linear.y = math.copysign(spd * abs(ey) / L, ey)
        return dist, cmd

    def marker(self, dist: float) -> None:
        if not self.target_name:
            return
        if dist <= self.stop_dist:
            c = "red"
        elif dist < self.yellow_dist:
            c = "yellow"
        else:
            c = "green"
        m = String()
        m.data = f"{self.target_name}:{c}"
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = DecisionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
