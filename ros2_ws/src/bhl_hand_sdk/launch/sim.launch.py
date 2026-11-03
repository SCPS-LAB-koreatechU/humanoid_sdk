"""Try the hands with no hardware: simulated boards + hand nodes + RViz estimate.

    ros2 launch bhl_hand_sdk sim.launch.py                      # both hands, channel sim, V2 hand view
    ros2 launch bhl_hand_sdk sim.launch.py hands:=right view:=false
    ros2 launch bhl_hand_sdk sim.launch.py channel:=vcan0       # with `hand_sim --channel vcan0` running

channel sim:   each hand node starts its own simulated board (the boards' real
               firmware logic, bhl_hand_sdk.sim); faults on /hand_<side>/sim_command.
channel vcan0: the nodes talk SocketCAN to a hand_sim you started yourself, whose
               terminal takes fault commands (power, drop, lose, fail, flash, status).

For the whole robot in RViz instead, see bhl_bringup:
    ros2 launch bhl_bringup demo.launch.py   +  robot.launch.py hands:=both dry_run:=false hand_channel:=sim
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def launch_setup(context, *_args, **_kwargs):
    text = LaunchConfiguration("hands").perform(context).strip().lower()
    hands = ["left", "right"] if text == "both" else [h.strip() for h in text.split(",") if h.strip()]
    channel = LaunchConfiguration("channel").perform(context)
    config = LaunchConfiguration("config").perform(context)
    extra = {"config": config} if config else {}
    actions = []
    for hand in hands:
        actions.append(Node(package="bhl_hand_sdk", executable="hand_controller", name=f"hand_{hand}",
                            output="screen", sigterm_timeout="5",
                            parameters=[{"hand": hand, "dry_run": False, "channel": channel,
                                         "follow_arm_estop": False, **extra}]))
        actions.append(Node(package="bhl_hand_sdk", executable="hand_joint_estimate", name=f"hand_{hand}_estimate",
                            output="screen", parameters=[{"hand": hand, **extra}]))
    if LaunchConfiguration("view").perform(context).lower() in ("true", "1", "yes", "on") and "right" in hands:
        share = Path(get_package_share_directory("bhl_hand_sdk"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "hand_view.launch.py")),
            launch_arguments=[("hand", "right"), ("hand_urdf", LaunchConfiguration("hand_urdf").perform(context))]))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("hands", default_value="both", description="left | right | both"),
        DeclareLaunchArgument("channel", default_value="sim", description="sim (no CAN) | vcan0 | can0"),
        DeclareLaunchArgument("view", default_value="true", description="RViz with the full V2 right hand"),
        DeclareLaunchArgument("hand_urdf", default_value="", description="DexHand V2 URDF; default from BHL_DESCRIPTION_WS"),
        DeclareLaunchArgument("config", default_value="", description="default: share/bhl_hand_sdk/config/dexhand.yaml"),
        OpaqueFunction(function=launch_setup),
    ])
