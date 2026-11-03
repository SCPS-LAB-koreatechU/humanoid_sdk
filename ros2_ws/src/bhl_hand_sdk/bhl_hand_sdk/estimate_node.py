"""ROS 2 node: hand_controller's ~/command_us -> estimated finger joints for RViz.

    ros2 run bhl_hand_sdk hand_joint_estimate --ros-args -p hand:=right
      in : /hand_<hand>/command_us              std_msgs/UInt16MultiArray (what was commanded)
      out: /hand_<hand>/joint_states_estimate   sensor_msgs/JointState, 20 Hz, rad

The output is a drawing of the command, not a measurement (see estimate.py).
bhl_bringup's robot.launch.py merges it into /joint_states when hands:= is set,
so the robot model in RViz moves its fingers; hand_view.launch.py shows it on
the full V2 hand.

Parameters: config (dexhand.yaml), hand, command_topic, rate (Hz).
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

from .config import load_config
from .estimate import estimate, load_maps


class HandJointEstimate(Node):
    def __init__(self) -> None:
        super().__init__("hand_joint_estimate")
        default_config = Path(get_package_share_directory("bhl_hand_sdk")) / "config" / "dexhand.yaml"
        self.declare_parameter("config", str(default_config))
        self.declare_parameter("hand", "right")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("rate", 20.0)
        path = self.get_parameter("config").value
        side = str(self.get_parameter("hand").value)
        self.spec = load_config(path).hand(side)
        self.side = side
        self.maps = load_maps(yaml.safe_load(Path(path).read_text()).get("visualization"))
        self.fractions = [0.0] * 15                     # drawn open until a command arrives
        topic = self.get_parameter("command_topic").value or f"/hand_{side}/command_us"
        self.create_subscription(UInt16MultiArray, topic, self._on_command, 10)
        self.publisher = self.create_publisher(JointState, f"/hand_{side}/joint_states_estimate", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._publish)
        self.get_logger().info(f"{side}: {topic} -> /hand_{side}/joint_states_estimate "
                               f"({len(self.maps)} joints, estimate of the command, not measured)")

    def _on_command(self, message: UInt16MultiArray) -> None:
        try:
            self.fractions = self.spec.to_fractions(message.data)
        except ValueError as error:
            self.get_logger().warn(f"command_us ignored: {error}", throttle_duration_sec=5.0)

    def _publish(self) -> None:
        joints = estimate(self.fractions, self.maps, self.side)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(joints)
        message.position = list(joints.values())
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = HandJointEstimate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
