"""ROS2 node that owns all low-level mecanum chassis commands."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

webots_home = os.environ.get("WEBOTS_HOME", "/usr/local/webots")
webots_controller_python = os.path.join(webots_home, "lib", "controller", "python")
if webots_controller_python not in sys.path:
    sys.path.insert(0, webots_controller_python)

try:
    from controller import Robot
except ImportError:  # Allows ROS-only testing without Webots' Python runtime.
    Robot = None


@dataclass(frozen=True)
class WheelSpeeds:
    motor1: float
    motor2: float
    motor3: float
    motor4: float


class MecanumDriverNode(Node):
    """Subscribe to /cmd_vel and convert chassis velocity to wheel speeds."""

    def __init__(self) -> None:
        super().__init__("mecanum_driver")

        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("half_length", 0.15)
        self.declare_parameter("half_width", 0.12)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tracker_state_topic", "/tracker_state")
        self.declare_parameter("heading_gain", 0.8)
        self.declare_parameter("motor1_name", "motor1")
        self.declare_parameter("motor2_name", "motor2")
        self.declare_parameter("motor3_name", "motor3")
        self.declare_parameter("motor4_name", "motor4")

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.half_length = self.get_parameter("half_length").value
        self.half_width = self.get_parameter("half_width").value
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.heading_error = 0.0

        topic = self.get_parameter("cmd_vel_topic").value
        self.subscription = self.create_subscription(Twist, topic, self.on_cmd_vel, 10)
        state_topic = self.get_parameter("tracker_state_topic").value
        self.state_subscription = self.create_subscription(
            Twist, state_topic, self.on_tracker_state, 10
        )
        self.robot: Optional[Robot] = None
        self.motors = {}
        self._connect_webots_motors()

        self.get_logger().info(f"mecanum driver listening on {topic}")

    def on_tracker_state(self, msg: Twist) -> None:
        self.heading_error = msg.angular.z

    def _connect_webots_motors(self) -> None:
        if Robot is None:
            self.get_logger().warn(
                "Webots controller module not found; running without motor output."
            )
            return

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        motor_params = {
            "motor1": "motor1_name",
            "motor2": "motor2_name",
            "motor3": "motor3_name",
            "motor4": "motor4_name",
        }

        for logical_name, parameter_name in motor_params.items():
            device_name = self.get_parameter(parameter_name).value
            motor = self.robot.getDevice(device_name)
            if motor is None:
                self.get_logger().error(f"Webots motor '{device_name}' was not found")
                continue
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.motors[logical_name] = motor

    def on_cmd_vel(self, msg: Twist) -> None:
        corrected_omega = msg.angular.z - self.heading_gain * self.heading_error
        speeds = self.inverse_kinematics(
            vx=msg.linear.x,
            vy=msg.linear.y,
            omega=corrected_omega,
        )
        self.set_wheel_speeds(speeds)

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> WheelSpeeds:
        radius = float(self.wheel_radius)
        chassis_radius = float(self.half_length) + float(self.half_width)
        return WheelSpeeds(
            motor1=(vx + vy - chassis_radius * omega) / radius,
            motor2=(-vx + vy - chassis_radius * omega) / radius,
            motor3=(-vx - vy - chassis_radius * omega) / radius,
            motor4=(vx - vy - chassis_radius * omega) / radius,
        )

    def set_wheel_speeds(self, speeds: WheelSpeeds) -> None:
        self.get_logger().info(
            "motor speeds m1=%.3f m2=%.3f m3=%.3f m4=%.3f"
            % (
                speeds.motor1,
                speeds.motor2,
                speeds.motor3,
                speeds.motor4,
            )
        )

        if not self.motors:
            return

        self.motors["motor1"].setVelocity(speeds.motor1)
        self.motors["motor2"].setVelocity(speeds.motor2)
        self.motors["motor3"].setVelocity(speeds.motor3)
        self.motors["motor4"].setVelocity(speeds.motor4)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MecanumDriverNode()
    try:
        if node.robot is None:
            rclpy.spin(node)
        else:
            while rclpy.ok() and node.robot.step(node.timestep) != -1:
                rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
