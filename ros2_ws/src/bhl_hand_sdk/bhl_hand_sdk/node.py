"""ROS 2 node: one DexHand on the arms' CAN bus, with a delivery result per command.

Run one per hand (hand:=left | right), each its own process and its own CAN
socket, so an arm cycle waiting for motor replies never delays the hand.

    ros2 run bhl_hand_sdk hand_controller --ros-args -r __node:=hand_right -p hand:=right
    ros2 service call /hand_right/enable std_srvs/srv/SetBool "{data: true}"
    ros2 param set /hand_right grip power
    ros2 param set /hand_right grip_level 0.6          # 0 open .. 1 the named grip
    ros2 topic echo /hand_right/delivery               # APPLIED / NO_REPLY / REJECT_... per set

  ~/enable        std_srvs/SetBool   true: ping, START (or RESUME if the hand is already
                                     driving), first set = current goal; false: HOLD
  ~/hold ~/resume ~/off ~/estop ~/reset_estop ~/ping ~/open
                  std_srvs/Trigger   success = the hand confirmed it (v2); message says what
                                     came back (CONTROL_ACK, APPLIED, NO_REPLY, SENT for v1)
  ~/target        std_msgs/Float64MultiArray (in)  15 pull fractions 0..1
  ~/target_us     std_msgs/UInt16MultiArray  (in)  15 pulse widths [us], calibrated range only
  ~/delivery      std_msgs/String (out, every set) JSON: sequence, result, latency_ms
  ~/command_us    std_msgs/UInt16MultiArray (out)  what was commanded -- not a measurement
  ~/status        std_msgs/String (out, 10 Hz)     JSON, HandController.status()
  /diagnostics    diagnostic_msgs/DiagnosticArray (out, 1 Hz)

Parameters: config (YAML), hand, dry_run (default true), grip, grip_level,
follow_arm_estop (default true: /arm_controller/status estop -> this hand's estop).

There is no hand /joint_states on purpose: the ES3352 servos measure nothing.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String, UInt16MultiArray
from std_srvs.srv import SetBool, Trigger

from .config import load_config
from .controller import Delivery, HandController
from .diagnostics import diagnostics

LEVELS = (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE)


class HandControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_controller")
        default_config = Path(get_package_share_directory("bhl_hand_sdk")) / "config" / "dexhand.yaml"
        self.declare_parameter("config", str(default_config))
        self.declare_parameter("hand", "right")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("follow_arm_estop", True)
        config = load_config(self.get_parameter("config").value)
        self.hand = HandController(config, str(self.get_parameter("hand").value),
                                   dry_run=bool(self.get_parameter("dry_run").value))
        self.dt = 1.0 / config.control_rate_hz
        self.declare_parameter("grip", "open", ParameterDescriptor(
            description=f"named grip: {sorted(config.grips)}"))
        self.declare_parameter("grip_level", 0.0, ParameterDescriptor(
            description="0 = open .. 1 = the named grip",
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)]))
        self.add_on_set_parameters_callback(self._on_parameters)

        self.delivery_pub = self.create_publisher(String, "~/delivery", 10)
        self.command_pub = self.create_publisher(UInt16MultiArray, "~/command_us", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Float64MultiArray, "~/target", self._on_target, 10)
        self.create_subscription(UInt16MultiArray, "~/target_us", self._on_target_us, 10)
        if self.get_parameter("follow_arm_estop").value:
            self.create_subscription(String, "/arm_controller/status", self._on_arm_status, 10)
        self.create_service(SetBool, "~/enable", self._on_enable)
        for name, call in (("hold", self.hand.hold), ("resume", self.hand.resume), ("off", self.hand.off),
                           ("estop", lambda: self.hand.estop("E-STOP (service)")),
                           ("reset_estop", self._reset_estop), ("ping", self._ping), ("open", self._open)):
            self.create_service(Trigger, f"~/{name}", self._trigger(name, call))
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.1, self._publish_status)
        self.create_timer(1.0, self._publish_diagnostics)

        spec = self.hand.spec
        mode = "DRY RUN (no CAN opened)" if self.hand.dry_run else f"LIVE on {spec.channel}"
        confirm = "STATUS replies (v2)" if spec.protocol >= 2 else "no replies (v1 firmware)"
        self.get_logger().info(f"{mode}: hand {spec.name}, node {spec.node}, {confirm}, "
                               f"{config.control_rate_hz:.0f} Hz. Output is off -- call ~/enable.")

    # ------------------------------------------------------------- parameters
    def _on_parameters(self, params: list[Parameter]) -> SetParametersResult:
        grip = self.get_parameter("grip").value
        level = self.get_parameter("grip_level").value
        for param in params:
            if param.name == "grip":
                grip = str(param.value)
                if grip not in self.hand.config.grips:
                    return SetParametersResult(successful=False,
                                               reason=f"unknown grip; known {sorted(self.hand.config.grips)}")
            elif param.name == "grip_level":
                level = float(param.value)
            elif param.name in ("config", "hand", "dry_run", "follow_arm_estop"):
                return SetParametersResult(successful=False, reason=f"{param.name} is fixed at start-up")
            else:
                continue
            try:
                self.hand.set_grip(grip, level)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ inputs
    def _on_target(self, message: Float64MultiArray) -> None:
        try:
            self.hand.set_goal(message.data)
        except ValueError as error:
            self.get_logger().warn(f"~/target refused: {error}")

    def _on_target_us(self, message: UInt16MultiArray) -> None:
        try:
            self.hand.set_goal_us(message.data)
        except ValueError as error:
            self.get_logger().warn(f"~/target_us refused: {error}")

    def _on_arm_status(self, message: String) -> None:
        try:
            arm_estop = bool(json.loads(message.data).get("estop"))
        except (ValueError, AttributeError):
            return
        if arm_estop and not self.hand.estopped:
            delivery = self.hand.estop("arm_controller E-STOP")
            self.get_logger().error(f"arm E-STOP -> hand {self.hand.config.estop_action}: {delivery.result}")

    # ---------------------------------------------------------------- services
    @staticmethod
    def _describe(delivery: Delivery) -> str:
        text = delivery.result
        if delivery.latency_ms >= 0:
            text += f" in {delivery.latency_ms:.1f} ms"
        return text + (f" ({delivery.message})" if delivery.message else "")

    def _trigger(self, name, call):
        def handler(_request, response: Trigger.Response) -> Trigger.Response:
            delivery = call()
            response.success = bool(delivery.ok)
            response.message = f"{name}: {self._describe(delivery)}"
            (self.get_logger().info if response.success else self.get_logger().warn)(response.message)
            return response
        return handler

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        delivery = self.hand.enable() if request.data else self.hand.hold()
        response.success = delivery.ok
        response.message = ("enable: " if request.data else "disable (HOLD): ") + self._describe(delivery)
        (self.get_logger().info if delivery.ok else self.get_logger().warn)(response.message)
        return response

    def _reset_estop(self) -> Delivery:
        self.hand.reset_estop()
        return Delivery(True, "CLEARED", message=self.hand.message)

    def _ping(self) -> Delivery:
        pong = self.hand.ping()
        if pong is None:
            return Delivery(False, "NO_REPLY", message="no PONG" if self.hand.confirmed else "v1 firmware cannot answer")
        return Delivery(True, "PONG", message=f"protocol {pong.version}, profile {pong.profile_tag}, "
                                              f"flags 0x{pong.flags:02X}")

    def _open(self) -> Delivery:
        self.hand.set_grip("open")
        self.set_parameters([Parameter("grip_level", Parameter.Type.DOUBLE, 0.0)])
        return Delivery(True, "GOAL_SET", message="open; the hand slews there while live")

    # -------------------------------------------------------------------- loop
    def _tick(self) -> None:
        delivery = self.hand.step(self.dt)
        if delivery is None:
            return
        if not delivery.ok and not self.hand.live:
            self.get_logger().error(f"hand stopped: {self.hand.fault}: {self.hand.message}")
        self.delivery_pub.publish(String(data=json.dumps(delivery.as_dict())))
        self.command_pub.publish(UInt16MultiArray(data=self.hand.command_us))

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self.hand.status())))

    def _publish_diagnostics(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for entry in diagnostics(self.hand.status()):
            status = DiagnosticStatus(level=LEVELS[entry["level"]], name=entry["name"],
                                      message=entry["message"], hardware_id=entry["hardware_id"])
            status.values = [KeyValue(key=k, value=v) for k, v in entry["values"].items()]
            message.status.append(status)
        self.diagnostics_pub.publish(message)

    def destroy_node(self) -> None:
        self.hand.close()          # HOLD if live: leave the hand gripping, not limp
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HandControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
