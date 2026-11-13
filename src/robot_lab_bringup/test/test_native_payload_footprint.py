"""Native geometry bounds include actual carried geometry and follow source FK."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

mj = pytest.importorskip('mujoco')
from robot_lab_utils.native_footprint import collision_footprint


@pytest.fixture
def plant():
    model = mj.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom type="plane" size="10 10 .1"/>
      <body name="base" pos="0 0 .2"><freejoint/>
        <geom type="box" size=".2 .15 .1" mass="2"/>
        <body name="arm" pos=".3 0 .2"><joint type="slide" axis="1 0 0"/>
          <geom type="box" size=".15 .04 .04" mass=".1"/></body></body>
      <body name="payload" pos=".7 0 .4"><freejoint/>
        <geom type="box" size=".12 .08 .06" mass=".2"/>
        <body pos=".2 0 0"><geom type="sphere" size=".05" mass=".01"/></body></body>
      <body name="unrelated" pos="20 0 .4"><freejoint/>
        <geom type="sphere" size=".1" mass=".1"/></body>
      </worldbody></mujoco>''')
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    return model, data


def test_payload_union_encloses_whole_compound_object_and_excludes_world(plant):
    model, data = plant
    unloaded = collision_footprint(model, data, 1, 3, padding=0.)
    loaded = collision_footprint(model, data, 1, 3, payload=3, padding=0.)
    assert max(x for x, y in unloaded)==pytest.approx(.45)
    assert max(x for x, y in loaded)==pytest.approx(.95)
    assert max(x for x, y in loaded)<1.   # unrelated object and infinite plane excluded
    assert min(x for x, y in loaded)==pytest.approx(-.2)
    # Merely computing/publishing a footprint does not move or weld any state.
    np.testing.assert_array_equal(data.qpos, model.qpos0)


def test_footprint_changes_with_actual_arm_and_payload_pose(plant):
    model, data = plant
    before = collision_footprint(model, data, 1, 3, payload=3, padding=0.)
    data.qpos[7] = 1.1  # arm slide
    mj.mj_forward(model, data)
    after = collision_footprint(model, data, 1, 3, payload=3, padding=0.)
    assert max(x for x, y in after)>max(x for x, y in before)+.5


def test_translation_and_yaw_do_not_change_base_local_bounds(plant):
    model, data = plant
    original = np.array(collision_footprint(model, data, 1, 3, payload=3))
    # Apply the same rigid world transform to all free bodies.
    for joint in range(model.njnt):
        if model.jnt_type[joint]!=mj.mjtJoint.mjJNT_FREE:
            continue
        index = model.jnt_qposadr[joint]
        x, y = data.qpos[index:index+2]
        data.qpos[index:index+2] = [3-y, 4+x]
        data.qpos[index+3:index+7] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
    mj.mj_forward(model, data)
    rotated = np.array(collision_footprint(model, data, 1, 3, payload=3))
    from shapely.geometry import Polygon
    assert Polygon(original).hausdorff_distance(Polygon(rotated)) < 1e-12


def test_missing_collision_or_invalid_payload_is_refused(plant):
    model, data = plant
    with pytest.raises(ValueError):
        collision_footprint(model, data, 1, 3, payload=2)
    with pytest.raises(ValueError,match='independent free'):
        collision_footprint(model, data, 1, 3, payload=4)
    model.geom_contype[2:] = 0
    model.geom_conaffinity[2:] = 0
    with pytest.raises(ValueError,match='no physical collision'):
        collision_footprint(model, data, 1, 3, payload=3)


def test_payload_interlock_requires_matching_fresh_generation():
    pytest.importorskip('control_msgs.action')
    from robot_lab_mujoco.native_mobile_control import NativeStretchControl
    state = SimpleNamespace(node=SimpleNamespace(mobile_tasks=SimpleNamespace(payload_body=3)),
        data=SimpleNamespace(time=10.), footprint_payload=3, footprint_time=9.5, footprint_error='')
    assert NativeStretchControl.payload_footprint_ready(state)
    state.footprint_payload = None
    assert not NativeStretchControl.payload_footprint_ready(state)
    state.footprint_payload = 3
    state.footprint_time = 9.
    assert not NativeStretchControl.payload_footprint_ready(state)
    state.footprint_time = 11.
    assert not NativeStretchControl.payload_footprint_ready(state)  # reset/backward clock
    state.footprint_time = 10.
    state.footprint_error = 'bad bounds'
    assert not NativeStretchControl.payload_footprint_ready(state)
    state.node.mobile_tasks.payload_body = None
    assert NativeStretchControl.payload_footprint_ready(state)  # prior unloaded control unchanged


def test_real_sensor_publisher_updates_both_costmaps_and_invalid_bounds_keep_gate_closed(plant):
    pytest.importorskip('geometry_msgs.msg')
    from robot_lab_mujoco.native_mobile_sensors import NativeMobileSensors
    model, data = plant
    emitted, errors = [[], []], []
    state = SimpleNamespace(footprint_time=-float('inf'), footprint_payload=None,
                            footprint_points=[], footprint_error='')
    sensors = NativeMobileSensors.__new__(NativeMobileSensors)
    sensors.model, sensors.data, sensors.base = model, data, 1
    sensors.node = SimpleNamespace(robot_bodies=3, mobile=state,
        mobile_tasks=SimpleNamespace(payload_body=3),
        get_logger=lambda: SimpleNamespace(error=lambda message, **kw: errors.append(message)))
    sensors.footprints = [SimpleNamespace(publish=messages.append) for messages in emitted]
    sensors.footprint()
    assert all(len(messages)==1 for messages in emitted)
    assert state.footprint_payload==3 and state.footprint_time==data.time
    assert max(p.x for p in emitted[0][0].points)>.95
    assert not state.footprint_error
    model.geom_contype[2:] = 0
    model.geom_conaffinity[2:] = 0
    sensors.footprint()
    assert state.footprint_error and errors
    assert all(len(messages)==1 for messages in emitted)  # no bogus/shrunken envelope published
