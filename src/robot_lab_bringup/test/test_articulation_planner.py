"""Private source joint plans and guards; fixtures are not TIAGo missions."""
import math
import time
from types import SimpleNamespace

import mujoco as mj
import numpy as np
import pytest

from robot_lab_mujoco.articulation_planner import JointScene, plan_joints, ArticulationPlanner
from robot_lab_utils.urdf_articulation import URDFArticulation, ArticulationROS, add_mujoco_articulation_actuators

XML = '''<mujoco><option gravity="0 0 0" timestep=".002" integrator="implicitfast"/>
<worldbody><geom name="obstacle" type="box" pos="0 0 .1" size=".07 .07 .04"/>
<body name="robot"><freejoint/><inertial pos="0 0 0" mass="1000" diaginertia="10 10 10"/>
<body><joint name="x" type="slide" axis="1 0 0" range="-.4 .4"/>
<inertial pos="0 0 0" mass=".1" diaginertia=".01 .01 .01"/>
<body><joint name="y" type="slide" axis="0 1 0" range="-.4 .4"/>
<geom name="tip" type="sphere" size=".015" pos="0 0 .1" mass=".1"/>
</body></body></body></worldbody></mujoco>'''
URDF = '''<robot name="fixture"><link name="base"/><link name="a"/><link name="b"/>
<joint name="x" type="prismatic"><parent link="base"/><child link="a"/><axis xyz="1 0 0"/>
<limit lower="-.4" upper=".4" effort="20" velocity=".4"/></joint>
<joint name="y" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="0 1 0"/>
<limit lower="-.4" upper=".4" effort="20" velocity=".4"/></joint></robot>'''


def make_fixture(tmp_path, obstacle=(0., 0., .1), start=(-.3, 0.)):
    config = dict(schema='robot_lab.urdf_articulation.v1', panel='arm', joints={
        n: dict(mode='position', lower=-.4, upper=.4, effort=20., max_velocity=.4, kp=1000., kd=40.)
        for n in ('x', 'y')})
    path = tmp_path/'source.urdf'
    path.write_text(URDF)
    c = URDFArticulation(config, URDF, path)
    model = mj.MjModel.from_xml_string(add_mujoco_articulation_actuators(XML, c))
    model.geom_pos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, 'obstacle')] = obstacle
    data = mj.MjData(model)
    addresses = [model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)] for n in ('x', 'y')]
    data.qpos[addresses] = start
    mj.mj_forward(model, data)
    c.step(dict(zip(('x', 'y'), start)), dict(x=0., y=0.), .002)
    c.heartbeat()
    root = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'robot')
    node = SimpleNamespace(_model=model, _data=data, _body_id=root)
    return node, c, addresses


def identity(contract, **args):
    return dict(plant_id=contract.plant_id, model_sha256=contract.model_sha256,
                urdf_sha256=contract.urdf_sha256, **args)


def tick(planner, node, c, addresses):
    positions = dict(zip(('x', 'y'), map(float, node._data.qpos[addresses])))
    velocities = {n: float(node._data.qvel[node._model.jnt_dofadr[
        mj.mj_name2id(node._model, mj.mjtObj.mjOBJ_JOINT, n)]]) for n in ('x', 'y')}
    planner.tick(positions, velocities, .002)
    return c.step(positions, velocities, .002,
                  position_rates=planner.rates if planner.phase == 'executing' else None)


def ready_plan(tmp_path, targets=None):
    node, c, addresses = make_fixture(tmp_path, obstacle=(.3, .3, .1), start=(0., 0.))
    planner = ArticulationPlanner(node, c)
    planner.request(identity(c, operation='plan', request_id='operator-1', targets=targets or dict(x=.1, y=.05)))
    tick(planner, node, c, addresses)
    planner.worker.join(timeout=3.)
    tick(planner, node, c, addresses)
    assert planner.phase == 'ready', planner.state()
    return node, c, planner, addresses


def test_detour_uses_private_model_and_independent_dense_contact_audit(tmp_path):
    node, c, addresses = make_fixture(tmp_path)
    original = node._data.qpos.copy()
    scene = JointScene(node._model, node._data, node._body_id, c.joints, ('x', 'y'))
    result = plan_joints(scene, dict(x=.3, y=0.))
    assert result['algorithm'] == 'rrt_connect'
    assert 1 < len(result['waypoints']) <= 64
    assert node._data.qpos == pytest.approx(original)
    assert not np.shares_memory(scene.model.geom_pos, node._model.geom_pos)
    probe = mj.MjData(node._model)
    probe.qpos[:] = original
    origin = np.array([-.3, 0.])
    for point in result['waypoints']:
        goal = np.array([point[n] for n in ('x', 'y')])
        duration = max(abs(goal-origin)/.4)
        for t in np.linspace(0., duration, 301):
            probe.qpos[addresses] = origin+np.clip(goal-origin, -.4*t, .4*t)
            mj.mj_forward(node._model, probe)
            assert not any(contact.dist < -.0005 for contact in probe.contact[:probe.ncon])
        origin = goal


def test_blocked_goal_and_source_limit_violation_are_refused(tmp_path):
    node, c, _ = make_fixture(tmp_path)
    for goal in (dict(x=0., y=0.), dict(x=.5, y=0.)):
        scene = JointScene(node._model, node._data, node._body_id, c.joints, ('x', 'y'))
        with pytest.raises(ValueError):
            plan_joints(scene, goal)


def test_planning_does_not_move_and_execute_uses_actual_servo_physics(tmp_path):
    node, c, planner, addresses = ready_plan(tmp_path)
    assert node._data.qpos[addresses] == pytest.approx([0., 0.])
    assert c.holding and not planner.busy
    planner.request(identity(c, operation='execute_plan', plan_id=planner.plan_id))
    for _ in range(2000):
        c.heartbeat()
        targets = tick(planner, node, c, addresses)
        for name, value in targets['position'].items():
            actuator = mj.mj_name2id(node._model, mj.mjtObj.mjOBJ_ACTUATOR, name+'_articulation')
            node._data.ctrl[actuator] = value
        mj.mj_step(node._model, node._data)
        if planner.phase in ('failed', 'succeeded'):
            break
    assert planner.phase == 'succeeded', planner.state()
    assert node._data.qpos[addresses] == pytest.approx([.1, .05], abs=.0005)
    assert c.holding


@pytest.mark.parametrize('change', ['geometry', 'plant', 'lease', 'expired', 'wrong_plan'])
def test_ready_plan_cannot_execute_after_binding_changes(tmp_path, change):
    node, c, planner, addresses = ready_plan(tmp_path)
    plan = planner.plan_id
    if change == 'geometry': node._model.geom_pos[0, 0] += .1
    elif change == 'plant': c.reset()
    elif change == 'lease': c.heartbeat_at = time.monotonic()-1.
    elif change == 'expired': planner.ready_at = time.monotonic()-11.
    elif change == 'wrong_plan': plan = 'different'
    planner.request(identity(c, operation='execute_plan', plan_id=plan))
    tick(planner, node, c, addresses)
    assert planner.phase in ('failed', 'stopped')
    assert c.holding and node._data.qpos[addresses] == pytest.approx([0., 0.])


def test_base_motion_and_stop_revoke_execution(tmp_path):
    node, c, planner, addresses = ready_plan(tmp_path)
    planner.request(identity(c, operation='execute_plan', plan_id=planner.plan_id))
    tick(planner, node, c, addresses)
    assert planner.phase == 'executing'
    planner.tick(dict(x=0., y=0.), dict(x=0., y=0.), .002, base_moving=True)
    assert planner.phase == 'failed' and c.holding
    planner.cancel('operator Stop')
    assert not planner.plan_id and not planner.busy


def test_planning_hold_keeps_lease_but_stop_revokes_old_commands(tmp_path):
    node, c, _ = make_fixture(tmp_path)
    plant, heartbeat = c.plant_id, c.heartbeat_at
    c.hold_for_planning()
    assert c.plant_id == plant and c.heartbeat_at == heartbeat and c.holding
    c.stop()
    assert c.plant_id != plant and c.heartbeat_at == -math.inf
    c.heartbeat()
    assert not c.command(dict(plant_id=plant, targets=dict(x=.1)))


def test_manual_takeover_keeps_current_authorized_command(tmp_path):
    node, c, planner, _ = ready_plan(tmp_path)
    bridge = object.__new__(ArticulationROS)
    bridge.contract, bridge.motion_planner, bridge.base_moving = c, planner, False
    import json
    plant = c.plant_id
    bridge.command(SimpleNamespace(data=json.dumps(dict(plant_id=plant, targets=dict(x=.03)))))
    assert not planner.plan_id and c.plant_id == plant
    assert not c.holding and c.targets['x'] == .03


def test_rate_override_is_reduction_only(tmp_path):
    node, c, _ = make_fixture(tmp_path)
    c.command(dict(plant_id=c.plant_id, targets=dict(x=-.1)))
    prior = c.references['x']
    result = c.step(dict(x=-.3, y=0.), dict(x=0., y=0.), .01, position_rates={'x': .02})
    assert result['position']['x']-prior == pytest.approx(.0002)
    for rates in ({'x': .5}, {'y': 0.}, {'unknown': .1}, {'x': float('nan')}):
        with pytest.raises(ValueError, match='position rates'):
            c.step(dict(x=-.3, y=0.), dict(x=0., y=0.), .01, position_rates=rates)
