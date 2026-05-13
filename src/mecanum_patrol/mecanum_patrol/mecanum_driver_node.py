"""Mecanum chassis driver: /cmd_vel -> inverse kinematics -> motor velocities."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))
from controller import Robot  # noqa: E402


@dataclass(frozen=True)
class WheelSpeeds:
    motor1: float
    motor2: float
    motor3: float
    motor4: float


class MecanumDriverNode(Node):

    def __init__(self) -> None:
        super().__init__("mecanum_driver")

        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("half_length", 0.15)
        self.declare_parameter("half_width", 0.12)
        self.declare_parameter("heading_gain", 0.8)

        self.r = self.get_parameter("wheel_radius").value
        self.hl = self.get_parameter("half_length").value
        self.hw = self.get_parameter("half_width").value
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.heading_error = 0.0
        self.paused = False
        self.last_speeds = WheelSpeeds(0.0, 0.0, 0.0, 0.0)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Twist, "/tracker_state", self.on_tracker_state, 10)
        self.create_subscription(String, "/pause", self.on_pause, 10)
        self.create_timer(0.3, self.on_timer)

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.m1 = self.robot.getDevice("motor1")
        self.m2 = self.robot.getDevice("motor2")
        self.m3 = self.robot.getDevice("motor3")
        self.m4 = self.robot.getDevice("motor4")
        for m in (self.m1, self.m2, self.m3, self.m4):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        self.get_logger().info("mecanum driver ready")

    def on_tracker_state(self, msg: Twist) -> None:
        self.heading_error = msg.angular.z

    def on_pause(self, msg: String) -> None:
        self.paused = not self.paused
        if self.paused:
            for m in (self.m1, self.m2, self.m3, self.m4):
                m.setVelocity(0.0)
        self.get_logger().info(f"pause = {self.paused}")

    def on_timer(self) -> None:
        if self.paused:
            self.get_logger().info("paused")
        else:
            s = self.last_speeds
            self.get_logger().info(
                f"m1={s.motor1:.3f} m2={s.motor2:.3f} m3={s.motor3:.3f} m4={s.motor4:.3f}")

    def on_cmd_vel(self, msg: Twist) -> None:
        if self.paused:
            return
        omega = msg.angular.z - self.heading_gain * self.heading_error
        speeds = self.inverse_kinematics(msg.linear.x, msg.linear.y, omega)
        self.last_speeds = speeds
        self.write_motors(speeds)

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> WheelSpeeds:
        L = float(self.hl) + float(self.hw)
        r = float(self.r)
        return WheelSpeeds(
            motor1=( vx + vy - L * omega) / r,
            motor2=(-vx + vy - L * omega) / r,
            motor3=(-vx - vy - L * omega) / r,
            motor4=( vx - vy - L * omega) / r,
        )

    def write_motors(self, s: WheelSpeeds) -> None:
        self.m1.setVelocity(s.motor1)
        self.m2.setVelocity(s.motor2)
        self.m3.setVelocity(s.motor3)
        self.m4.setVelocity(s.motor4)


def main():
    rclpy.init()
    node = MecanumDriverNode()
    while rclpy.ok() and node.robot.step(node.timestep) != -1:
        rclpy.spin_once(node, timeout_sec=0.0)
    node.destroy_node()
    rclpy.shutdown()
