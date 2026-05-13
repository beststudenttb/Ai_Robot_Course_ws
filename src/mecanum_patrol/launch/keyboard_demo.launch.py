"""Launch with keyboard — manual control."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

CONFIG = "/home/tb/robot_ws/src/mecanum_patrol/config/mecanum_driver.yaml"


def generate_launch_description():
    os.environ["WEBOTS_HOME"] = "/usr/local/webots"
    webots_python = "/usr/local/webots/lib/controller/python"
    workspace_python = "/home/tb/robot_ws/install/mecanum_patrol/lib/python3.10/site-packages"
    ros_python = "/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

    common_setup = (
        "source /home/tb/anaconda3/etc/profile.d/conda.sh; "
        "conda activate rospy; "
        "source /opt/ros/humble/setup.bash; "
        "source /home/tb/robot_ws/install/setup.bash; "
        "export WEBOTS_HOME=/usr/local/webots; "
        f"export PYTHONPATH={webots_python}:{workspace_python}:{ros_python}:$PYTHONPATH; "
        "export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:/opt/ros/humble/lib:$LD_LIBRARY_PATH; "
    )

    def terminal(title, command):
        return ExecuteProcess(
            output="screen",
            cmd=[
                "gnome-terminal",
                f"--title={title}",
                "--",
                "bash", "-lc",
                common_setup + command + "; exec bash",
            ],
        )

    return LaunchDescription([
        terminal(
            "mecanum",
            "/opt/ros/humble/share/webots_ros2_driver/scripts/webots-controller "
            "--robot-name=tracker /opt/ros/humble/bin/ros2 run mecanum_patrol mecanum_driver "
            f"--ros-args --params-file {CONFIG}",
        ),
        terminal(
            "perception",
            "/opt/ros/humble/share/webots_ros2_driver/scripts/webots-controller "
            "--robot-name=super /opt/ros/humble/bin/ros2 run mecanum_patrol perception",
        ),
        terminal(
            "decision",
            f"ros2 run mecanum_patrol decision --ros-args --params-file {CONFIG}",
        ),
        terminal(
            "mission",
            f"ros2 run mecanum_patrol mission --ros-args --params-file {CONFIG}",
        ),
        terminal(
            "keyboard",
            f"ros2 run mecanum_patrol keyboard_app --ros-args --params-file {CONFIG}",
        ),
        ExecuteProcess(
            cmd=["bash", "-c",
                 common_setup + "ros2 run mecanum_patrol logger"]),
    ])
