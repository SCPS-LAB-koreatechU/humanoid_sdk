"""hand_controller for one or both DexHands.

    ros2 launch bhl_hand_sdk hand_sdk.launch.py                      # right hand, dry run
    ros2 launch bhl_hand_sdk hand_sdk.launch.py hands:=both dry_run:=false
    ros2 launch bhl_hand_sdk hand_sdk.launch.py hands:=left config:=/path/to/dexhand.yaml

Each hand is its own node (/hand_left, /hand_right) and process with its own
CAN socket on the arms' can0.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def hand_nodes(hands: str, dry_run: bool, config: str) -> list:
    names = {"both": ["left", "right"], "none": [], "": []}.get(hands, [h.strip() for h in hands.split(",") if h.strip()])
    params = {"dry_run": dry_run}
    if config:
        params["config"] = config
    return [Node(package="bhl_hand_sdk", executable="hand_controller", name=f"hand_{name}", output="screen",
                 parameters=[{**params, "hand": name}], sigterm_timeout="5")
            for name in names]


def launch_setup(context, *_args, **_kwargs):
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() in ("true", "1", "yes", "on")
    return hand_nodes(LaunchConfiguration("hands").perform(context), dry_run,
                      LaunchConfiguration("config").perform(context))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("hands", default_value="right", description="left | right | both | none"),
        DeclareLaunchArgument("dry_run", default_value="true", description="true: no CAN is opened"),
        DeclareLaunchArgument("config", default_value="", description="default: share/bhl_hand_sdk/config/dexhand.yaml"),
        OpaqueFunction(function=launch_setup),
    ])
