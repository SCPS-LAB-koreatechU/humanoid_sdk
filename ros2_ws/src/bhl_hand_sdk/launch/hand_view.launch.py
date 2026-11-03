"""RViz with the full DexHand V2 hand (every finger and thumb joint), drawing
/hand_<hand>/joint_states_estimate -- the commanded pose, not a measurement.

    ros2 launch bhl_hand_sdk hand_view.launch.py                 # right hand
    ros2 launch bhl_hand_sdk hand_view.launch.py hand_urdf:=/path/dexhandv2_right.urdf

The URDF and meshes come from the Berkeley-Humanoid-Lite description workspace
(vendor/dexhandv2_description, CC BY-NC-SA 4.0), found through BHL_DESCRIPTION_WS
(default ~/Berkeley-Humanoid-Lite/ros2_ws); nothing is copied here. Upstream only
has a right hand, so the left hand is shown by the robot model (bhl_bringup), which
moves the fingers' Pitch and Yaw.

Frames are prefixed hand_view/ so they never clash with the robot's /tf.
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *_args, **_kwargs):
    from bhl_hand_sdk import v2_urdf

    hand = LaunchConfiguration("hand").perform(context)
    if hand != "right":
        return [LogInfo(msg="[bhl_hand_sdk] hand_view: only the right V2 hand URDF exists upstream; "
                            "use bhl_bringup's robot model for the left hand.")]
    try:
        get_package_share_directory("dexhandv2_description")
        package_found = True
    except PackageNotFoundError:
        package_found = False
    try:
        description = v2_urdf.load(LaunchConfiguration("hand_urdf").perform(context) or None, package_found)
    except FileNotFoundError as error:
        return [LogInfo(msg=f"[bhl_hand_sdk] hand_view skipped: {error}")]
    share = Path(get_package_share_directory("bhl_hand_sdk"))
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             namespace="hand_view", output="screen",
             parameters=[{"robot_description": description, "frame_prefix": "hand_view/"}],
             remappings=[("joint_states", f"/hand_{hand}/joint_states_estimate")]),
        Node(package="rviz2", executable="rviz2", name="hand_view_rviz", output="screen",
             arguments=["-d", str(share / "rviz" / "hand_view.rviz")]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("hand", default_value="right", description="right (the only V2 hand URDF)"),
        DeclareLaunchArgument("hand_urdf", default_value="", description="default from BHL_DESCRIPTION_WS"),
        OpaqueFunction(function=launch_setup),
    ])
