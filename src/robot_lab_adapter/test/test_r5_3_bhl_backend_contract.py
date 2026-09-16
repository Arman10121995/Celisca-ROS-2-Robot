"""R5.3 tests: BHL policy bringup contract (backend + registry wiring).

Static, graph-free verification that the policy controller is actually
launchable and consumable through the platform's normal bringup path:

- Backend: ``bhl_controllers.yaml`` commands the same 22 joints in the same
  canonical order as ``BHL_JOINT_NAMES`` (the order both adapter nodes publish
  ``Float64MultiArray`` data in), through a position
  ``forward_command_controller``.
- Backend velocity reporting: ``bhl_ros2_control.xacro`` declares position +
  velocity state interfaces on every commanded joint, so the
  ``joint_state_broadcaster`` can report the joint velocities the policy
  observation consumes (the previously declared "zeros until JointState
  carries them" limitation is a property of missing publishers, not of the
  backend description).
- Description frames: the BHL URDFs must not reuse one name for both a link
  and a joint.  URDF->SDF conversion promotes each to a frame, so a shared
  name makes the model unspawnable in Gazebo (the upstream export named the
  dummy IMU link *and* its mount joint "imu").
- Registry/dispatch: ``humanoid_policy_controller`` is cataloged in
  ``algorithms.yaml`` (control category), dispatched in
  ``algorithm_dispatch.yaml`` to a real console-script entry point of
  ``robot_lab_adapter``, and its declared capability/class requirements are
  satisfied by the berkeley_humanoid_lite robot entity (same check the
  registry's typed compatibility validation performs).

These are launch-contract tests: they parse the on-disk YAML/xacro/setup.py,
not a live gz-sim graph.
"""

from __future__ import annotations

import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

_HERE = Path(__file__).resolve().parent
_ADAPTER = _HERE.parent
_SRC = _ADAPTER.parent
if str(_ADAPTER) not in sys.path:
    sys.path.insert(0, str(_ADAPTER))

from robot_lab_adapter.bhl_balance import BHL_JOINT_NAMES  # noqa: E402
from robot_lab_adapter.bhl_policy import load_policy_config  # noqa: E402

_ROBOTS_CONFIG = _SRC / "robot_lab_robots" / "config" / "robots.yaml"
_CONTROLLERS_YAML = (_SRC / "robot_lab_robots" / "berkeley_humanoid_lite"
                     / "config" / "bhl_controllers.yaml")
_ROS2_CONTROL_XACRO = (_SRC / "robot_lab_robots" / "berkeley_humanoid_lite"
                       / "xacro" / "bhl_ros2_control.xacro")
_REGISTRY_ROBOTS = (_SRC / "robot_lab" / "robot_lab_registry" / "config"
                    / "robots.yaml")
_ALGORITHMS = (_SRC / "robot_lab" / "robot_lab_registry" / "config"
               / "algorithms.yaml")
_DISPATCH = (_SRC / "robot_lab_bringup" / "config" / "algorithm_dispatch.yaml")
_SETUP_PY = _ADAPTER / "setup.py"

CONTROLLER_NAME = "bhl_standing_controller"
ALGORITHM_ID = "humanoid_policy_controller"


def _load_yaml(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


# ----------------------------------------------------------------------------
# Backend: bhl_controllers.yaml
# ----------------------------------------------------------------------------

def test_backend_is_position_forward_command_controller():
    config = _load_yaml(_CONTROLLERS_YAML)
    manager = config["controller_manager"]["ros__parameters"]
    assert manager[CONTROLLER_NAME]["type"] == (
        "forward_command_controller/ForwardCommandController")


def test_backend_commands_the_canonical_22_joints_in_order():
    config = _load_yaml(_CONTROLLERS_YAML)
    params = config[CONTROLLER_NAME]["ros__parameters"]
    assert params["interface_name"] == "position"
    assert tuple(params["joints"]) == tuple(BHL_JOINT_NAMES)


# ----------------------------------------------------------------------------
# Backend: bhl_ros2_control.xacro
# ----------------------------------------------------------------------------

def _ros2_control_joint_declarations():
    """Parse the ``bhl_joint`` macro invocations with their interfaces."""
    text = _ROS2_CONTROL_XACRO.read_text()
    declarations = {}
    for match in re.finditer(
            r'<xacro:bhl_joint name="([^"]+)"\s*/>', text):
        declarations[match.group(1)] = {
            "command": "position",  # fixed inside the macro body
            "state": re.findall(r'<state_interface name="(\w+)"/>', text),
        }
    return declarations


def test_every_backend_joint_declares_velocity_state_interface():
    declarations = _ros2_control_joint_declarations()
    assert set(declarations) == set(BHL_JOINT_NAMES)
    for joint, declared in declarations.items():
        assert "position" in declared["state"], joint
        assert "velocity" in declared["state"], joint


def test_joint_macro_declares_position_command_and_full_state():
    """The ``bhl_joint`` macro body, expanded for all 22 joints, is what the
    backend hardware interface actually implements."""
    text = _ROS2_CONTROL_XACRO.read_text()
    match = re.search(
        r'<xacro:macro name="bhl_joint".*?</xacro:macro>', text, re.DOTALL)
    assert match, "bhl_joint macro not found in bhl_ros2_control.xacro"
    body = match.group(0)
    assert '<command_interface name="position">' in body
    for state in ("position", "velocity", "effort"):
        assert f'<state_interface name="{state}"/>' in body


# ----------------------------------------------------------------------------
# Description: Gazebo/SDF frame-name uniqueness
# ----------------------------------------------------------------------------

_BHL_URDFS = (
    _SRC / "robot_lab_robots" / "berkeley_humanoid_lite" / "urdf"
    / "berkeley_humanoid_lite.urdf",
    _SRC / "robot_lab_robots" / "berkeley_humanoid_lite" / "urdf"
    / "berkeley_humanoid_lite_biped.urdf",
)


@pytest.mark.parametrize("urdf", _BHL_URDFS, ids=lambda path: path.name)
def test_bhl_urdf_has_unique_link_and_joint_names(urdf):
    """A link and a joint must not share one name.

    URDF->SDF conversion promotes every link *and* every joint to a frame, so
    the upstream export's duplicate "imu" (dummy IMU link + its mount joint)
    made the model unspawnable: ign gazebo logged "Non-unique name[imu]
    detected 2 times", "Error Code 2: frame with name[imu] already exists" and
    FrameAttachedToGraph/PoseRelativeToGraph cycles.  ``ros_gz_sim create``
    still printed "OK creation of entity", so the defect was invisible to a
    spawn check graded on that marker.  The joint is now imu_mount_joint.
    """
    root = ET.parse(urdf).getroot()
    links = [element.get("name") for element in root.findall("link")]
    joints = [element.get("name") for element in root.findall("joint")]
    assert len(links) == len(set(links)), "duplicate link names in %s" % urdf
    assert len(joints) == len(set(joints)), "duplicate joint names in %s" % urdf
    shared = sorted(set(links) & set(joints))
    assert not shared, (
        "names used by both a link and a joint in %s: %s (Gazebo frames must "
        "be unique)" % (urdf, shared))


def test_bhl_imu_frame_link_keeps_its_registry_name():
    """The fix renames the mount joint, never the IMU frame link.

    The registry IMU contract publishes in frame ``imu`` (the dummy link), and
    the xacro layer attaches the simulated sensor to ``imu_2``; both names must
    survive, only the colliding joint name changes.
    """
    root = ET.parse(_BHL_URDFS[0]).getroot()
    links = {element.get("name") for element in root.findall("link")}
    joints = {element.get("name") for element in root.findall("joint")}
    assert "imu" in links
    assert "imu_2" in links
    assert "imu_frame" in joints
    assert "imu_mount_joint" in joints
    assert "imu" not in joints


# ----------------------------------------------------------------------------
# Registry + dispatch wiring
# ----------------------------------------------------------------------------

def _algorithm_entry(algorithm_id):
    algorithms = _load_yaml(_ALGORITHMS)
    entries = algorithms if isinstance(algorithms, list) else algorithms.get("algorithms", [])
    for entry in entries:
        if entry.get("id") == algorithm_id:
            return entry
    return None


def _dispatch_entry(category, algorithm_id):
    dispatch = _load_yaml(_DISPATCH)["algorithms"]
    return (dispatch.get(category) or {}).get(algorithm_id)


def test_policy_controller_is_cataloged_in_the_registry():
    entry = _algorithm_entry(ALGORITHM_ID)
    assert entry is not None, f"{ALGORITHM_ID} missing from algorithms.yaml"
    assert entry["category"] == "control"
    assert entry["status"] == "integrated"
    assert entry["implementation"]["package"] == "robot_lab_adapter"
    assert entry["input_contract"]["required_topics"] == [
        "/joint_states", "/bhl/imu", "/cmd_vel"]
    # Command rate honest to the upstream policy_dt = 0.04 s.
    assert entry["parameters"]["command_rate_hz"] == 25.0


def test_policy_controller_is_dispatched_to_a_real_entry_point():
    entry = _algorithm_entry(ALGORITHM_ID)
    dispatch = _dispatch_entry(entry["category"], ALGORITHM_ID)
    assert dispatch, f"{ALGORITHM_ID} missing from algorithm_dispatch.yaml"
    node = dispatch["node"]
    assert node["package"] == entry["implementation"]["package"]

    # The dispatch executable is a registered console script of that package.
    setup_text = _SETUP_PY.read_text()
    match = re.search(
        rf"'{re.escape(node['executable'])} = {node['package']}\.(\w+):main'",
        setup_text)
    assert match, (
        f"{node['package']} setup.py does not register "
        f"'{node['executable']} = {node['package']}.<module>:main'")

    # And that module really defines the cataloged plugin class.
    import importlib
    module = importlib.import_module(f"{node['package']}.{match.group(1)}")
    assert hasattr(module, entry["implementation"]["plugin"])


def test_registry_compatibility_with_the_bhl_robot_entity():
    """The same typed check validation.perform_compatibility_check applies."""
    robots = _load_yaml(_REGISTRY_ROBOTS)
    entries = robots if isinstance(robots, list) else robots.get("robots", [])
    robot = next(r for r in entries
                 if r["id"] == "berkeley_humanoid_lite")
    entry = _algorithm_entry(ALGORITHM_ID)

    assert entry["supported_robot_classes"] == ["humanoid"]
    assert robot["robot_class"] == "humanoid"
    missing = set(entry["required_capabilities"]) - set(robot["capabilities"])
    assert not missing, f"robot lacks capabilities required by {ALGORITHM_ID}: {missing}"


def test_policy_joint_ordering_is_consistent_with_the_backend():
    """The adapter maps measurements by name into the policy order; the
    canonical publish order (BHL_JOINT_NAMES) and the policy config joint
    set must agree so the Float64MultiArray indexing stays meaningful."""
    config = load_policy_config(name="policy_humanoid")
    assert set(config.joints) == set(BHL_JOINT_NAMES)
    assert len(config.joints) == len(BHL_JOINT_NAMES) == 22

