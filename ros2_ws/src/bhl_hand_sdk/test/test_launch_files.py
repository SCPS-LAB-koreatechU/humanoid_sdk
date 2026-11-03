"""sim.launch.py, hand_view.launch.py and hand_sdk.launch.py with stand-ins for
launch / launch_ros (no ROS needed): which nodes start with which parameters."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Declare(Action):
    def __init__(self, name, default_value="", description=""):
        super().__init__()
        self.name, self.default_value = name, default_value


class Opaque(Action):
    def __init__(self, function):
        super().__init__()
        self.function = function


class Node(Action):
    def __init__(self, package, executable, name=None, namespace=None, output=None, parameters=None,
                 remappings=None, arguments=None, sigterm_timeout=None):
        super().__init__()
        self.package, self.executable, self.name, self.namespace = package, executable, name, namespace
        self.parameters, self.remappings, self.arguments = parameters or [], remappings or [], arguments or []


class Include(Action):
    def __init__(self, source, launch_arguments=()):
        super().__init__()
        self.source, self.launch_arguments = source, list(launch_arguments)


class Log(Action):
    def __init__(self, msg):
        super().__init__()
        self.msg = msg


class Config:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class NotFound(KeyError):
    pass


@pytest.fixture
def launch(monkeypatch, tmp_path):
    shares = {"bhl_hand_sdk": PACKAGE}

    def share(name):
        if name not in shares:
            raise NotFound(name)
        return str(shares[name])

    names = ["launch", "launch.actions", "launch.substitutions", "launch.launch_description_sources",
             "launch_ros", "launch_ros.actions", "ament_index_python", "ament_index_python.packages"]
    mods = {n: types.ModuleType(n) for n in names}
    mods["launch"].LaunchDescription = list
    mods["launch.actions"].DeclareLaunchArgument = Declare
    mods["launch.actions"].OpaqueFunction = Opaque
    mods["launch.actions"].IncludeLaunchDescription = Include
    mods["launch.actions"].LogInfo = Log
    mods["launch.substitutions"].LaunchConfiguration = Config
    mods["launch.launch_description_sources"].PythonLaunchDescriptionSource = lambda path: path
    mods["launch_ros.actions"].Node = Node
    mods["ament_index_python.packages"].get_package_share_directory = share
    mods["ament_index_python.packages"].PackageNotFoundError = NotFound
    for n, m in mods.items():
        monkeypatch.setitem(sys.modules, n, m)

    def run(stem, **overrides):
        spec = importlib.util.spec_from_file_location(f"hand_launch_{stem}", PACKAGE / "launch" / f"{stem}.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = {d.name: d.default_value for d in description if isinstance(d, Declare)}
        assert not set(overrides) - set(context), "not a launch argument"
        context.update(overrides)
        return [a for d in description if isinstance(d, Opaque) for a in d.function(context)]

    return types.SimpleNamespace(run=run, tmp_path=tmp_path)


def test_sim_launch_starts_simulated_hands_estimates_and_the_view(launch):
    actions = launch.run("sim")
    hands = [a for a in actions if isinstance(a, Node) and a.executable == "hand_controller"]
    assert [h.name for h in hands] == ["hand_left", "hand_right"]
    assert hands[1].parameters[0] == {"hand": "right", "dry_run": False, "channel": "sim", "follow_arm_estop": False}
    assert [a.name for a in actions if isinstance(a, Node) and a.executable == "hand_joint_estimate"] == [
        "hand_left_estimate", "hand_right_estimate"]
    include = [a for a in actions if isinstance(a, Include)]
    assert len(include) == 1 and include[0].source.endswith("hand_view.launch.py")


def test_sim_launch_on_vcan_without_view(launch):
    actions = launch.run("sim", hands="right", channel="vcan0", view="false")
    hand = next(a for a in actions if isinstance(a, Node) and a.executable == "hand_controller")
    assert hand.parameters[0]["channel"] == "vcan0"
    assert not [a for a in actions if isinstance(a, Include)]


def test_hand_view_loads_the_v2_urdf_with_fixes(launch):
    urdf = launch.tmp_path / "dexhandv2_description" / "urdf" / "dexhandv2_right.urdf"
    urdf.parent.mkdir(parents=True)
    urdf.write_text('<robot name="h"><joint name="R_Index_Flexor" type="revolute"/>'
                    '<joint name="R_Index_DIP" type="revolute"><mimic joint="Index_Flexor"/></joint></robot>')
    actions = launch.run("hand_view", hand_urdf=str(urdf))
    rsp = next(a for a in actions if isinstance(a, Node) and a.executable == "robot_state_publisher")
    assert rsp.namespace == "hand_view" and rsp.parameters[0]["frame_prefix"] == "hand_view/"
    assert '<mimic joint="R_Index_Flexor"' in rsp.parameters[0]["robot_description"]
    assert rsp.remappings == [("joint_states", "/hand_right/joint_states_estimate")]
    rviz = next(a for a in actions if isinstance(a, Node) and a.executable == "rviz2")
    assert Path(rviz.arguments[1]).is_file()


def test_hand_view_explains_what_is_missing(launch):
    actions = launch.run("hand_view", hand_urdf=str(launch.tmp_path / "nope.urdf"))
    assert [a for a in actions if isinstance(a, Log)] and not [a for a in actions if isinstance(a, Node)]
    assert "only the right" in launch.run("hand_view", hand="left")[0].msg


def test_hand_sdk_launch_both(launch):
    actions = launch.run("hand_sdk", hands="both", dry_run="false")
    assert [a.parameters[0] for a in actions] == [{"dry_run": False, "hand": "left"}, {"dry_run": False, "hand": "right"}]
