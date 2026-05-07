"""Attach to an already-running Webots world and start keyboard control."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


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

    keyboard_terminal = ExecuteProcess(
        output="screen",
        cmd=[
            "gnome-terminal",
            "--title=keyboard",
            "--",
            "bash",
            "-lc",
            common_setup + "ros2 run mecanum_patrol keyboard_app; exec bash",
        ],
    )

    mecanum_terminal = ExecuteProcess(
        output="screen",
        cmd=[
            "gnome-terminal",
            "--title=mecanum",
            "--",
            "bash",
            "-lc",
            common_setup
            + "/opt/ros/humble/share/webots_ros2_driver/scripts/webots-controller "
            + "--robot-name=tracker /opt/ros/humble/bin/ros2 run mecanum_patrol mecanum_driver "
            + "--ros-args --params-file /home/tb/robot_ws/src/mecanum_patrol/config/mecanum_driver.yaml; exec bash",
        ],
    )

    perception_terminal = ExecuteProcess(
        output="screen",
        cmd=[
            "gnome-terminal",
            "--title=perception",
            "--",
            "bash",
            "-lc",
            common_setup
            + "/opt/ros/humble/share/webots_ros2_driver/scripts/webots-controller "
            + "--robot-name=super /opt/ros/humble/bin/ros2 run mecanum_patrol perception; exec bash",
        ],
    )

    decision_terminal = ExecuteProcess(
        output="screen",
        cmd=[
            "gnome-terminal",
            "--title=decision",
            "--",
            "bash",
            "-lc",
            common_setup
            + "ros2 run mecanum_patrol decision --ros-args --params-file "
            + "/home/tb/robot_ws/src/mecanum_patrol/config/mecanum_driver.yaml; exec bash",
        ],
    )

    mission_terminal = ExecuteProcess(
        output="screen",
        cmd=[
            "gnome-terminal",
            "--title=mission",
            "--",
            "bash",
            "-lc",
            common_setup
            + "ros2 run mecanum_patrol mission --ros-args --params-file "
            + "/home/tb/robot_ws/src/mecanum_patrol/config/mecanum_driver.yaml; exec bash",
        ],
    )

    return LaunchDescription(
        [
            keyboard_terminal,
            mecanum_terminal,
            perception_terminal,
            decision_terminal,
            mission_terminal,
        ]
    )
