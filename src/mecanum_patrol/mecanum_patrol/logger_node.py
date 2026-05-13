"""Logger node: aggregates status from other nodes via service calls."""

import rclpy
from mecanum_patrol_interfaces.srv import GetStatus
from rclpy.node import Node


class LoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("logger")
        self.srv = self.create_service(GetStatus, "/get_status", self._on_request)
        self.get_logger().info("logger ready, service /get_status")

    def _on_request(self, request, response):
        from mecanum_patrol_interfaces.srv import GetPose, GetTarget

        pose_cli = self.create_client(GetPose, "/get_pose")
        target_cli = self.create_client(GetTarget, "/get_target")

        if pose_cli.wait_for_service(timeout_sec=1.0):
            future = pose_cli.call_async(GetPose.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if future.result() is not None:
                p = future.result()
                response.pose_x = p.x
                response.pose_y = p.y
        else:
            self.get_logger().warn("cannot reach /get_pose")

        if target_cli.wait_for_service(timeout_sec=1.0):
            future = target_cli.call_async(GetTarget.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if future.result() is not None:
                t = future.result()
                response.cmd_vx = t.cmd_vx
                response.cmd_vy = t.cmd_vy
                response.target_name = t.target_name
        else:
            self.get_logger().warn("cannot reach /get_target")

        self.get_logger().info(
            f"status: pos=({response.pose_x:.2f},{response.pose_y:.2f}) "
            f"cmd=({response.cmd_vx:.3f},{response.cmd_vy:.3f}) "
            f"target={response.target_name}")
        return response


def main():
    rclpy.init()
    node = LoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
