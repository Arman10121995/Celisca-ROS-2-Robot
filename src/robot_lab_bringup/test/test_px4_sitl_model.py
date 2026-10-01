"""The PX4 SITL SDF flattener must produce a vehicle with a body (R5.4).

`gz sdf -p` cannot resolve `<include><uri>model://NAME</uri></include>` without
a find-callback, so the "resolved" model it prints keeps the plugins but has
**no links** - PX4 then sees a world with no vehicle, its estimator never
converges ("Preflight Fail: height estimate not stable") and it refuses to arm.
`scripts/px4_sitl_model.py` resolves the include graph itself; these tests pin
that, including the rotor-topic rewrite the FCU needs.
"""

import importlib.util
import os
import sys
import tempfile

# src/robot_lab_bringup/test/<this file> -> workspace root -> scripts/
_WORKSPACE = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
_SCRIPTS = os.path.join(_WORKSPACE, "scripts")


def _load_module():
    path = os.path.join(_SCRIPTS, "px4_sitl_model.py")
    spec = importlib.util.spec_from_file_location("px4_sitl_model", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_model(root, name, body):
    directory = os.path.join(root, name)
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, "model.sdf"), "w") as handle:
        handle.write(body)
    return directory


BASE = """<?xml version="1.0"?>
<sdf version='1.9'>
  <model name='%s'>
    <link name='%s_link'><inertial><mass>1.0</mass></inertial></link>
    <sensor name='%s_imu' type='imu'/>
    %s
  </model>
</sdf>
"""

ROTOR_WRAPPER = """<?xml version="1.0"?>
<sdf version='1.9'>
  <model name='%s'>
    <include merge='true'><uri>model://%s</uri></include>
    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <commandSubTopic>command/motor_speed</commandSubTopic>
    </plugin>
  </model>
</sdf>
"""


def test_include_is_flattened_into_links():
    module = _load_module()
    with tempfile.TemporaryDirectory() as root:
        _write_model(root, "chassis", BASE % ("chassis", "chassis", "chassis", ""))
        _write_model(root, "frame", BASE % ("frame", "frame", "frame",
                                            "<include merge='true'>"
                                            "<uri>model://chassis</uri></include>"))
        _write_model(root, "rotors", ROTOR_WRAPPER % ("rotors", "frame"))
        text = module.entity_sdf(root, "rotors")
    assert "<include" not in text, "an include survived the flattening"
    # rotors -> frame -> chassis: both included links must survive.
    assert text.count("<link") == 2, "an included link was dropped"
    assert "frame_link" in text and "chassis_link" in text
    assert "MulticopterMotorModel" in text, "the wrapper's plugin was dropped"
    assert "imu" in text, "the included sensor was dropped"


def test_motor_topic_is_rewritten_to_the_fcu_topic():
    """PX4 publishes /<model>/command/motor_speed; the entity must listen there."""
    module = _load_module()
    with tempfile.TemporaryDirectory() as root:
        _write_model(root, "chassis", BASE % ("chassis", "chassis", "chassis", ""))
        _write_model(root, "rotors", ROTOR_WRAPPER % ("rotors", "chassis"))
        text = module.entity_sdf(root, "rotors",
                                 motor_topic="/rotors_0/command/motor_speed")
    assert "commandSubTopic>/rotors_0/command/motor_speed<" in text, text
    assert "commandSubTopic>command/motor_speed<" not in text


def test_entity_name_and_sdf_version():
    module = _load_module()
    with tempfile.TemporaryDirectory() as root:
        _write_model(root, "chassis", BASE % ("chassis", "chassis", "chassis", ""))
        text = module.entity_sdf(root, "chassis", entity_name="x500_0")
    assert 'version="1.9"' in text
    assert 'name="x500_0"' in text


def test_missing_model_and_include_cycles_are_refused():
    module = _load_module()
    with tempfile.TemporaryDirectory() as root:
        _write_model(root, "loop", BASE % (
            "loop", "loop", "loop",
            "<include merge='true'><uri>model://loop</uri></include>"))
        try:
            module.flatten_model(root, "does_not_exist")
        except FileNotFoundError:
            pass
        else:
            raise AssertionError("a missing model must be refused")
        try:
            module.flatten_model(root, "loop")
        except ValueError:
            pass
        else:
            raise AssertionError("an include cycle must be refused")
