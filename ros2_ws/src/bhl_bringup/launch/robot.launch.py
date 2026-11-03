"""On-board bring-up of the upper body. The robot's measured state is the bus.

    ros2 launch bhl_bringup robot.launch.py                       # dry run: no CAN, angles follow the command
    ros2 launch bhl_bringup robot.launch.py dry_run:=false        # real arms (CAN interfaces up first)
    ros2 launch bhl_bringup robot.launch.py model:=stock          # stock | dexhand | v1arm (default) | tuning
    ros2 launch bhl_bringup robot.launch.py urdf:=/path/to/robot.urdf
    ros2 launch bhl_bringup robot.launch.py arm:=false joint_state_sources:=/bhl/joint_states
                                        # joint states come from somewhere else (bhl_onboard on the RDK X5)
    ros2 launch bhl_bringup robot.launch.py joint_state_sources:=/arm_controller/joint_states,/hand/joint_states
    ros2 launch bhl_bringup robot.launch.py hands:=both            # + DexHand nodes (bhl_hand_sdk), same dry_run
    ros2 launch bhl_bringup robot.launch.py hands:=both hand_channel:=sim
                                        # hands on simulated boards (hand_sim, no CAN); the arms stay
                                        # in dry run unless dry_run:=false

Runs on the computer that is wired to the motors (Jetson / RDK X5 / bench PC):

    arm_controller          bhl_arm_sdk   CAN <-> /arm_controller/joint_states [rad]
                                          in : /arm_controller/target (JointState, rad), target.<joint> params
                                                ~/enable ~/estop ~/reset_estop ~/home services
                                          out: /arm_controller/status (JSON, 10 Hz), /diagnostics (1 Hz)
    joint_state_publisher                 merges every topic in `joint_state_sources` into /joint_states:
                                          every movable joint of the URDF, 0 rad for the ones no source
                                          has reported yet (wrists, fingers, until their driver exists)
    robot_state_publisher                 URDF + /joint_states -> /tf, /tf_static, /robot_description
    hand_left, hand_right   bhl_hand_sdk  (hands:=left|right|both) one process and CAN socket per
                                          DexHand on the same can0; ~/enable ~/hold ~/resume ~/off,
                                          ~/delivery (APPLIED / NO_REPLY / REJECT_ per set),
                                          follows /arm_controller/status E-STOP. No /joint_states:
                                          the finger servos measure nothing.
    hand_joint_estimate     bhl_hand_sdk  (hand_estimate:=true, default) commanded pulses -> estimated
                                          finger joints, merged into /joint_states so RViz draws the
                                          command (an estimate, not a measurement)

Nothing graphical runs here; that is remote.launch.py on the PC. Both only need
to share ROS_DOMAIN_ID (env.sh). Without a URDF -- description package not
installed and no urdf:= -- the controller still runs and publishes /joint_states
by itself; there is just no /tf and no /robot_description.
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DESCRIPTION_PACKAGE = "berkeley_humanoid_lite_description"
ARM_PACKAGE = "bhl_arm_sdk"
HAND_PACKAGE = "bhl_hand_sdk"
ARM_STATES = "/arm_controller/joint_states"
URDF_FILES = {
    "stock": "berkeley_humanoid_lite.urdf",
    "dexhand": "berkeley_humanoid_lite_dexhand.urdf",
    "v1arm": "berkeley_humanoid_lite_v1arm.urdf",
    "tuning": "berkeley_humanoid_lite_tuning.urdf",
}


def truthy(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("true", "1", "yes", "on")


def parse_sources(text: str) -> list[str]:
    """'/a,/b' or '[/a, /b]' -> ['/a', '/b']."""
    return [s.strip() for s in text.strip().strip("[]").split(",") if s.strip()]


def find_urdf(explicit: str, model: str) -> tuple[str, str]:
    """(path, problem). The explicit file wins; otherwise the model's URDF from the
    description package. `problem` says why there is no path."""
    if explicit:
        return (explicit, "") if Path(explicit).is_file() else ("", f"urdf:={explicit} does not exist")
    try:
        share = Path(get_package_share_directory(DESCRIPTION_PACKAGE))
    except PackageNotFoundError:
        return "", (f"package {DESCRIPTION_PACKAGE} not found (source the Berkeley-Humanoid-Lite "
                    "ros2_ws, import it with humanoid.repos, or pass urdf:=<file>)")
    path = share / "urdf" / URDF_FILES.get(model, f"berkeley_humanoid_lite_{model}.urdf")
    return (str(path), "") if path.is_file() else ("", f"no URDF for model:={model} at {path}")


def default_arm_config() -> str:
    return str(Path(get_package_share_directory(ARM_PACKAGE)) / "config" / "bhl_arms.yaml")


def parse_hands(text: str) -> list[str]:
    """'both' -> ['left', 'right']; 'none' / '' -> []; 'right' or 'left,right' as given."""
    text = text.strip().lower()
    if text in ("", "none", "false"):
        return []
    if text == "both":
        return ["left", "right"]
    return [h.strip() for h in text.split(",") if h.strip()]


def hand_available() -> bool:
    try:
        get_package_share_directory(HAND_PACKAGE)
        return True
    except PackageNotFoundError:
        return False


def hand_nodes(context, hands: list[str], estimate: bool) -> list:
    if not hand_available():
        return [LogInfo(msg=f"[bhl_bringup] hands:={','.join(hands)} but {HAND_PACKAGE} is not built; "
                            "no hand nodes started.")]
    channel = LaunchConfiguration("hand_channel").perform(context)
    # A simulated board is not hardware: the hand streams to it even while the
    # arms stay in dry run (dry_run:=false would put the arms live on can0).
    params = {"dry_run": truthy(context, "dry_run") and channel != "sim"}
    hand_config = LaunchConfiguration("hand_config").perform(context)
    if hand_config:
        params["config"] = hand_config
    if channel:
        params["channel"] = channel
    nodes = [Node(package=HAND_PACKAGE, executable="hand_controller", name=f"hand_{hand}", output="screen",
                  parameters=[{**params, "hand": hand}],
                  sigterm_timeout="5")     # time to send HOLD on shutdown
             for hand in hands]
    if estimate:
        nodes += [Node(package=HAND_PACKAGE, executable="hand_joint_estimate", name=f"hand_{hand}_estimate",
                       output="screen",
                       parameters=[{"hand": hand, **({"config": hand_config} if hand_config else {})}])
                  for hand in hands]
    return nodes


def launch_setup(context, *_args, **_kwargs):
    actions = []
    arm = truthy(context, "arm")
    publish_tf = truthy(context, "publish_tf")
    rate = int(float(LaunchConfiguration("rate").perform(context)))
    sources = parse_sources(LaunchConfiguration("joint_state_sources").perform(context))
    hands = parse_hands(LaunchConfiguration("hands").perform(context))
    estimate = bool(hands) and truthy(context, "hand_estimate") and hand_available()
    if estimate:
        sources += [f"/hand_{h}/joint_states_estimate" for h in hands
                    if f"/hand_{h}/joint_states_estimate" not in sources]
    urdf, problem = find_urdf(LaunchConfiguration("urdf").perform(context),
                              LaunchConfiguration("model").perform(context))
    if publish_tf and not urdf:
        actions.append(LogInfo(msg=f"[bhl_bringup] no /tf, no /robot_description: {problem}. "
                                   "arm_controller publishes /joint_states directly."))
        publish_tf = False
    if publish_tf and not arm and ARM_STATES in sources:
        actions.append(LogInfo(msg=f"[bhl_bringup] arm:=false but joint_state_sources still lists {ARM_STATES}; "
                                   "/joint_states will stay at 0 until something publishes it."))

    if arm:
        config = LaunchConfiguration("config").perform(context) or default_arm_config()
        actions.append(Node(
            package=ARM_PACKAGE, executable="arm_controller", name="arm_controller", output="screen",
            parameters=[{"dry_run": truthy(context, "dry_run"), "config": config}],
            # With the state publishers up, the arm's own ten joints go to a side topic
            # and joint_state_publisher owns /joint_states for the whole URDF.
            remappings=[("/joint_states", ARM_STATES)] if publish_tf else [],
            sigterm_timeout="5"))     # time to put every motor in IDLE on shutdown

    if publish_tf:
        actions += [
            Node(package="joint_state_publisher", executable="joint_state_publisher",
                 name="joint_state_publisher", output="screen",
                 arguments=[urdf],          # joint list from the file; no wait for the topic
                 parameters=[{"source_list": sources, "rate": rate}]),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="robot_state_publisher", output="screen",
                 parameters=[{"robot_description": Path(urdf).read_text(),
                              "publish_frequency": float(rate)}]),
        ]
    if hands:
        actions += hand_nodes(context, hands, estimate)
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true: no CAN is opened, angles follow the command"),
        DeclareLaunchArgument("config", default_value="",
                              description="bhl_arm_sdk joint map; default: its config/bhl_arms.yaml"),
        DeclareLaunchArgument("arm", default_value="true",
                              description="start arm_controller (false: joint states come from elsewhere)"),
        DeclareLaunchArgument("model", default_value="v1arm",
                              description="URDF from the description package: stock | dexhand | v1arm | tuning"),
        DeclareLaunchArgument("urdf", default_value="", description="explicit URDF file, overrides model"),
        DeclareLaunchArgument("joint_state_sources", default_value=ARM_STATES,
                              description="comma-separated JointState topics merged into /joint_states"),
        DeclareLaunchArgument("rate", default_value="50",
                              description="/joint_states and /tf publish rate [Hz]"),
        DeclareLaunchArgument("publish_tf", default_value="true",
                              description="run joint_state_publisher + robot_state_publisher here "
                                          "(false if another machine already does)"),
        DeclareLaunchArgument("hands", default_value="none",
                              description="DexHand nodes from bhl_hand_sdk: none | left | right | both"),
        DeclareLaunchArgument("hand_config", default_value="",
                              description="bhl_hand_sdk profiles; default: its config/dexhand.yaml"),
        DeclareLaunchArgument("hand_channel", default_value="",
                              description="hands' CAN channel: '' = the profile's (can0), vcan0, or sim "
                                          "(simulated boards, no CAN)"),
        DeclareLaunchArgument("hand_estimate", default_value="true",
                              description="with hands: draw the commanded finger pose in /joint_states (estimate)"),
        OpaqueFunction(function=launch_setup),
    ])
