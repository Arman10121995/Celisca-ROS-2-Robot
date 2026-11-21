"""Source-contract checks only; these do not qualify a simulator mission."""
from copy import deepcopy
import json
import math
import time
import xml.etree.ElementTree as ET

import pytest

from robot_lab_utils.urdf_articulation import (
    ArticulationROS, URDFArticulation, add_mujoco_articulation_actuators, effort_budget,
    gravity_feedforward, gravity_loads, measured_motion_rate,
)


URDF = '''<robot name="fixture"><link name="base"/>
<joint name="conveyor_lift_joint" type="prismatic"><limit lower="0" upper="0.68" velocity="0.035" effort="2000"/></joint>
<joint name="telescope_stage1_joint" type="prismatic"><limit lower="0" upper="0.14" velocity="0.1" effort="250"/></joint>
<joint name="telescope_stage2_joint" type="prismatic"><limit lower="0" upper="0.14" velocity="0.1" effort="250"/><mimic joint="telescope_stage1_joint"/></joint>
<joint name="transfer_roller_joint" type="continuous"><limit velocity="30" effort="2"/></joint>
<joint name="caster_spin_joint" type="continuous"><limit velocity="30" effort="2"/></joint></robot>'''

CONFIG = dict(schema='robot_lab.urdf_articulation.v1', joints={
    'conveyor_lift_joint': dict(mode='position', lower=0., upper=.68, max_velocity=.035, effort=2000., kp=20000., kd=200.),
    'telescope_stage1_joint': dict(mode='position', lower=0., upper=.14, max_velocity=.1, effort=250., kp=4000., kd=80.),
    'transfer_roller_joint': dict(mode='velocity', max_velocity=8., effort=2., kp=0., kd=.2)},
    couplings={'telescope_stage2_joint': dict(joint='telescope_stage1_joint', multiplier=1., offset=0.)})


@pytest.fixture
def controller(tmp_path):
    model = tmp_path/'robot.urdf'
    model.write_text(URDF)
    return URDFArticulation(deepcopy(CONFIG), URDF, model)


def measurement(controller, lift=.2, slide=0.):
    values = {name: 0. for name in controller.joints}
    values['conveyor_lift_joint'] = lift
    values['telescope_stage1_joint'] = values['telescope_stage2_joint'] = slide
    return values, {name: 0. for name in values}


def test_initial_target_is_actual_and_commands_require_heartbeat_and_identity(controller):
    position, velocity = measurement(controller)
    result = controller.step(position, velocity, .01, now=1.)
    assert result['position']['conveyor_lift_joint'] == .2
    assert not controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .4}}, now=1.)
    controller.heartbeat(1.)
    assert not controller.command({'plant_id': 'old', 'targets': {'conveyor_lift_joint': .4}}, now=1.)
    controller.heartbeat(1.)  # rejection Stop revokes the old heartbeat
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .4}}, now=1.)
    result = controller.step(position, velocity, .01, now=1.)
    assert result['position']['conveyor_lift_joint'] == pytest.approx(.20035)
    assert controller.base_inhibited


def test_coupled_stage_is_bounded_and_not_independent(controller):
    position, velocity = measurement(controller)
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    assert not controller.command({'plant_id': controller.plant_id, 'targets': {'telescope_stage2_joint': .1}}, now=1.)
    controller.heartbeat(1.)  # rejection Stop starts a new control generation
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'telescope_stage1_joint': .14}}, now=1.)
    result = controller.step(position, velocity, .02, now=1.)
    assert result['position']['telescope_stage1_joint'] == result['position']['telescope_stage2_joint'] == .002
    assert all('caster' not in name for name in controller.joints)


def test_watchdog_and_stop_capture_measurement_once_and_zero_roller(controller):
    position, velocity = measurement(controller)
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .6, 'transfer_roller_joint': 6.}}, now=1.)
    result = controller.step(position, velocity, .01, now=1.6)
    assert result['position']['conveyor_lift_joint'] == .2
    assert result['velocity']['transfer_roller_joint'] == 0.
    position['conveyor_lift_joint'] = .19
    result = controller.step(position, velocity, .01, now=1.7)
    assert result['position']['conveyor_lift_joint'] == .2  # no chasing gravity sag
    controller.stop()
    assert controller.references['conveyor_lift_joint'] == .19
    old = controller.plant_id
    controller.reset()
    assert old != controller.plant_id and not controller.initialized


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, .69])
def test_invalid_targets_fail_closed(controller, value):
    position, velocity = measurement(controller)
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    assert not controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': value}}, now=1.)
    assert controller.references['conveyor_lift_joint'] == .2


def test_base_motion_rejects_articulation_and_extension_inhibits_drive(controller):
    position, velocity = measurement(controller, slide=.01)
    controller.step(position, velocity, .01, now=1.)
    assert controller.base_inhibited
    controller.heartbeat(1.)
    assert not controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .4}}, now=1., base_moving=True)
    controller.step(position, velocity, .01, now=1.)
    position['conveyor_lift_joint'] = float('nan')
    controller.step(position, velocity, .01, now=1.)
    assert controller.fault and controller.base_inhibited
    json.dumps(controller.state(), allow_nan=False)


def test_source_bounds_and_actuator_limits_are_preserved(controller, tmp_path):
    root = ET.fromstring(add_mujoco_articulation_actuators('<mujoco><worldbody/></mujoco>', controller))
    assert root.find('option').get('integrator') == 'implicitfast'
    actuators = {entry.get('joint'): entry for entry in root.find('actuator')}
    assert set(actuators) == set(controller.joints)
    assert actuators['conveyor_lift_joint'].get('forcerange') == '-2000 2000'
    assert actuators['transfer_roller_joint'].get('ctrlrange') == '-8 8'
    config = deepcopy(CONFIG)
    config['joints']['conveyor_lift_joint']['effort'] = 2001
    with pytest.raises(ValueError, match='exceeds source'):
        URDFArticulation(config, URDF, controller.model_path)


def test_mujoco_position_damping_keeps_the_motor_force_limit(controller):
    name = 'conveyor_lift_joint'
    fixture = f'<mujoco><worldbody><body><joint name="{name}"/></body></worldbody></mujoco>'
    config = controller.config
    config['mujoco_joint_servo_damping'] = {name: 12.0}
    root = ET.fromstring(add_mujoco_articulation_actuators(fixture, controller))
    actuator = root.find(f"actuator/general[@joint='{name}']")
    assert actuator.get('biasprm').split()[-1] == '-12'
    assert actuator.get('forcerange') == '-2000 2000'
    assert controller.joints[name]['kd'] == CONFIG['joints'][name]['kd']
    config['mujoco_joint_servo_damping'] = {'transfer_roller_joint': 12.0}
    with pytest.raises(ValueError, match='independent position joint'):
        add_mujoco_articulation_actuators(fixture, controller)
    config['mujoco_joint_servo_damping'] = {name: float('nan')}
    with pytest.raises(ValueError, match='Invalid MuJoCo servo damping'):
        add_mujoco_articulation_actuators(fixture, controller)


def test_optional_pad_reader_distinguishes_unavailable_from_a_valid_empty_report(controller):
    from types import SimpleNamespace

    bridge = object.__new__(ArticulationROS)
    bridge.contract = controller
    bridge.motion_planner = None
    bridge._string = SimpleNamespace
    reports = []
    bridge.publisher = SimpleNamespace(publish=lambda message: reports.append(json.loads(message.data)))
    position, velocity = measurement(controller)
    for sample in (None, {'sim_time_s': .1, 'normal_force_n': {}}):
        bridge.grip_contacts = lambda: sample
        bridge.last_publish = -math.inf
        bridge.step(position, velocity, .01)
    assert 'grip_contacts' not in reports[0]
    assert reports[1]['grip_contacts'] == {'sim_time_s': .1, 'normal_force_n': {}}


def test_start_and_stop_clamp_only_targets_not_measured_limit_epsilon(controller):
    position, velocity = measurement(controller, lift=-1e-6, slide=-1e-6)
    result = controller.step(position, velocity, .01, now=1.)
    assert controller.measured['conveyor_lift_joint'] == -1e-6
    assert all(value == 0. for value in result['position'].values())
    position['conveyor_lift_joint'] = .680001
    position['telescope_stage1_joint'] = position['telescope_stage2_joint'] = .140001
    controller.step(position, velocity, .01, now=1.1)
    controller.stop()
    result = controller.step(position, velocity, .01, now=1.2)
    assert result['position']['conveyor_lift_joint'] == .68
    assert result['position']['telescope_stage1_joint'] == .14
    for channel in controller.state()['channels']:
        assert channel['limits'][0] <= channel['target'] <= channel['limits'][1]
    assert controller.measured['conveyor_lift_joint'] == .680001


def test_stop_and_watchdog_hold_each_measured_stage_without_recoupling(controller):
    position, velocity = measurement(controller, slide=.05)
    position['telescope_stage2_joint'] = .02
    result = controller.step(position, velocity, .01, now=1.)
    assert result['position']['telescope_stage1_joint'] == .05
    assert result['position']['telescope_stage2_joint'] == .02
    assert controller.base_inhibited and not controller.fault
    assert controller.state()['coupling_inhibited']
    controller.heartbeat(1.)
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'telescope_stage1_joint': .1}}, now=1.)
    result = controller.step(position, velocity, .01, now=1.)
    assert result['position']['telescope_stage2_joint'] == pytest.approx(.021)
    result = controller.step(position, velocity, .01, now=1.6)
    assert result['position']['telescope_stage2_joint'] == .02
    result = controller.step(position, velocity, .01, now=1.7)
    assert result['position']['telescope_stage2_joint'] == .02
    position['telescope_stage2_joint'] = .045
    controller.step(position, velocity, .01, now=1.8)
    controller.stop()
    result = controller.step(position, velocity, .01, now=1.9)
    assert result['position']['telescope_stage2_joint'] == .045
    assert not controller.fault
    assert not controller.state()['coupling_inhibited']


def test_coupled_reference_slews_both_forward_and_backward(controller):
    position, velocity = measurement(controller, slide=.05)
    position['telescope_stage2_joint'] = .1
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'telescope_stage1_joint': .1}}, now=1.)
    result = controller.step(position, velocity, .02, now=1.)
    assert result['position']['telescope_stage1_joint'] == pytest.approx(.052)
    assert result['position']['telescope_stage2_joint'] == pytest.approx(.098)
    assert controller.targets['telescope_stage2_joint'] == .1
    assert controller.measured['telescope_stage2_joint'] == .1


def test_stop_revokes_heartbeat_and_rejects_delayed_prior_generation(controller):
    position, velocity = measurement(controller)
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    command = {'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .5}}
    assert controller.command(command, now=1.)
    previous_id = controller.plant_id
    controller.stop()
    assert controller.plant_id != previous_id
    assert controller.heartbeat_at == -math.inf
    # A fresh heartbeat alone cannot make a queued old command current again.
    controller.heartbeat(1.1)
    assert not controller.command(command, now=1.1)
    assert controller.targets['conveyor_lift_joint'] == .2
    controller.heartbeat(1.2)
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .5}}, now=1.2)


def test_watchdog_stop_changes_generation_once_and_state_has_capture_time(controller):
    position, velocity = measurement(controller)
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .5}}, now=1.)
    previous_id = controller.plant_id
    controller.step(position, velocity, .01, now=1.6)
    stopped_id = controller.plant_id
    assert stopped_id != previous_id and controller.holding
    for now in (1.7, 1.8, 2., 3.):
        controller.step(position, velocity, .01, now=now)
        assert controller.plant_id == stopped_id
    before = time.monotonic()
    state = controller.state()
    assert before <= state['state_monotonic'] <= time.monotonic()


def test_either_held_stage_extension_blocks_drive_even_with_small_coupling_error(controller):
    position, velocity = measurement(controller, slide=.003)
    position['telescope_stage2_joint'] = .007
    controller.step(position, velocity, .01, now=1.)
    assert controller.holding and not controller.active
    assert not controller.state()['coupling_inhibited']
    assert controller.base_inhibited
    position['telescope_stage2_joint'] = .005
    controller.step(position, velocity, .01, now=1.1)
    assert not controller.base_inhibited


CHAIN = '''<robot name="chain">
<link name="base"><inertial><mass value="100"/></inertial></link>
<link name="carriage"><inertial><mass value="10"/></inertial></link>
<link name="stage1"><inertial><mass value="2"/></inertial></link>
<link name="stage2"><inertial><mass value="2"/></inertial></link>
<link name="roller"><inertial><mass value=".2"/></inertial></link>
<joint name="conveyor_lift_joint" type="prismatic"><parent link="base"/><child link="carriage"/>
<axis xyz="0 0 1"/><limit lower="0" upper="0.68" velocity="0.035" effort="2000"/></joint>
<joint name="telescope_stage1_joint" type="prismatic"><parent link="carriage"/><child link="stage1"/>
<axis xyz="1 0 0"/><limit lower="0" upper="0.14" velocity="0.1" effort="250"/></joint>
<joint name="telescope_stage2_joint" type="prismatic"><parent link="stage1"/><child link="stage2"/>
<axis xyz="1 0 0"/><limit lower="0" upper="0.14" velocity="0.1" effort="250"/><mimic joint="telescope_stage1_joint"/></joint>
<joint name="transfer_roller_joint" type="continuous"><parent link="carriage"/><child link="roller"/>
<axis xyz="0 1 0"/><limit velocity="30" effort="2"/></joint></robot>'''


def test_gravity_feedforward_holds_each_slide_subtree(tmp_path):
    model = tmp_path/'chain.urdf'
    model.write_text(CHAIN)
    contract = URDFArticulation(deepcopy(CONFIG), CHAIN, model)
    loads = gravity_loads(CHAIN, contract)
    assert loads['conveyor_lift_joint'][0] == pytest.approx(14.2)
    assert loads['conveyor_lift_joint'][1] == pytest.approx((0., 0., 1.))
    assert loads['telescope_stage1_joint'][0] == pytest.approx(4.)
    assert loads['telescope_stage2_joint'][0] == pytest.approx(2.)
    assert 'transfer_roller_joint' not in loads
    efforts = {name: spec['effort'] for name, spec in contract.joints.items()}
    level = gravity_feedforward(loads, (0., 0., 0., 1.), efforts)
    assert level['conveyor_lift_joint'] == pytest.approx(14.2*9.81)
    assert level['telescope_stage1_joint'] == pytest.approx(0.)
    # Pitched 0.2 rad about y: the slides point down, so they are held back.
    pitched = gravity_feedforward(loads, (0., math.sin(.1), 0., math.cos(.1)), efforts)
    assert pitched['telescope_stage1_joint'] == pytest.approx(-4.*9.81*math.sin(.2))
    assert pitched['conveyor_lift_joint'] == pytest.approx(14.2*9.81*math.cos(.2))
    assert gravity_feedforward(loads, (0., 0., 0., 1.), dict(efforts, conveyor_lift_joint=50.))[
        'conveyor_lift_joint'] == 50.


def test_activity_follows_measured_motion_not_a_biased_velocity_report(controller):
    position, velocity = measurement(controller)
    velocity['conveyor_lift_joint'] = -.044   # Isaac's report for the resting lift
    velocity['transfer_roller_joint'] = .1
    for _ in range(40):
        controller.step(position, velocity, .01, now=1.)
    assert not controller.active and not controller.base_inhibited
    position = dict(position)
    for _ in range(40):
        position['conveyor_lift_joint'] += .0003   # an actual 0.03 m/s
        controller.step(position, velocity, .01, now=1.)
    assert controller.active and controller.base_inhibited


@pytest.mark.parametrize('name,amplitude', (
    ('conveyor_lift_joint', .003), ('transfer_roller_joint', .03)))
def test_returning_oscillation_still_inhibits_base(controller, name, amplitude):
    position, velocity = measurement(controller)
    origin = position[name]
    for index in range(64):
        phase = 2*math.pi*((index+1)/32)/.25
        position[name] = origin+amplitude*math.sin(phase)
        velocity[name] = amplitude*2*math.pi/.25*math.cos(phase)
        controller.step(position, velocity, 1/32, now=1.)
    # The endpoints of the 0.25 s window coincide despite real motion inside.
    samples = controller.history[name]
    assert samples[-1][1] == pytest.approx(samples[0][1])
    assert controller.active and controller.base_inhibited


def test_excursion_noise_allowance_keeps_biased_rest_available(controller):
    position, velocity = measurement(controller)
    velocity['conveyor_lift_joint'] = -.044
    velocity['transfer_roller_joint'] = .1
    for index in range(64):
        position['conveyor_lift_joint'] = .2+.00075*math.sin(index*1.7)
        controller.step(position, velocity, 1/32, now=1.)
    assert not controller.active and not controller.base_inhibited


def test_motion_rate_retains_slow_net_motion_and_startup_fallback():
    assert measured_motion_rate([(0., 0.)], fallback=.044) == .044
    assert measured_motion_rate([(0., 0.), (.25, -.003)]) == pytest.approx(-.012)


@pytest.mark.parametrize('wanted,limit', ((139.302, 2000.), (-139.302, 2000.),
                                         (3000., 2000.), (-3000., 2000.), (0., 250.)))
def test_feedforward_and_servo_share_one_source_effort_limit(wanted, limit):
    feedforward, servo_limit = effort_budget(wanted, limit)
    assert abs(feedforward) <= limit
    assert servo_limit >= 0.
    for servo in (-servo_limit, 0., servo_limit):
        assert abs(feedforward+servo) <= limit+1e-9


@pytest.mark.parametrize('wanted,limit', ((math.nan, 2000.), (math.inf, 2000.),
                                         (0., math.inf), (0., 0.), (0., -1.)))
def test_invalid_effort_budget_is_rejected(wanted, limit):
    with pytest.raises(ValueError, match='force budget'):
        effort_budget(wanted, limit)


GRIP_URDF = '''<robot name="grip"><link name="base"/>
<joint name="finger_joint" type="prismatic"><limit lower="0" upper="0.045" velocity="0.05" effort="16"/></joint></robot>'''


def grip_controller(tmp_path, **extra):
    model = tmp_path/'grip.urdf'
    model.write_text(GRIP_URDF)
    config = dict(schema='robot_lab.urdf_articulation.v1', grip_joints=['finger_joint'], joints={
        'finger_joint': dict(mode='position', lower=0., upper=.045, max_velocity=.025, effort=16., kp=800., kd=20.)},
        **extra)
    return URDFArticulation(config, GRIP_URDF, model)


def squeeze_on_object(controller, at=.02, steps=40):
    """Close on an object that stops the finger at ``at``; returns the last time."""
    controller.step({'finger_joint': at}, {'finger_joint': 0.}, .01, now=1.)
    controller.heartbeat(1.)
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'finger_joint': 0.}}, now=1.)
    now = 1.
    for _ in range(steps):
        now += .05
        controller.heartbeat(now)
        controller.step({'finger_joint': at}, {'finger_joint': 0.}, .05, now=now)
    return now


def test_still_grip_on_an_object_does_not_inhibit_the_base(tmp_path):
    controller = grip_controller(tmp_path)
    squeeze_on_object(controller)
    assert controller.targets['finger_joint'] == 0.
    assert not controller.active and not controller.base_inhibited


def test_stop_and_base_motion_keep_a_bounded_squeeze(tmp_path):
    controller = grip_controller(tmp_path)
    now = squeeze_on_object(controller)
    controller.step({'finger_joint': .02}, {'finger_joint': 0.}, .05, now=now, base_moving=True)
    assert controller.holding
    assert controller.targets['finger_joint'] == controller.references['finger_joint'] == pytest.approx(.015)
    result = controller.step({'finger_joint': .02}, {'finger_joint': 0.}, .05, now=now+.05, base_moving=True)
    assert result['position']['finger_joint'] == pytest.approx(.015)
    controller.stop()
    assert controller.targets['finger_joint'] == pytest.approx(.015)    # stays bounded, never tightens


def test_moving_grip_stops_at_its_measured_position(tmp_path):
    controller = grip_controller(tmp_path)
    controller.step({'finger_joint': .04}, {'finger_joint': 0.}, .01, now=1.)
    controller.heartbeat(1.)
    controller.command({'plant_id': controller.plant_id, 'targets': {'finger_joint': 0.}}, now=1.)
    now, position = 1., .04
    for _ in range(10):
        now, position = now+.05, position-.001
        controller.heartbeat(now)
        controller.step({'finger_joint': position}, {'finger_joint': -.02}, .05, now=now)
    assert controller.active
    controller.stop()
    assert controller.targets['finger_joint'] == pytest.approx(position)


def test_grip_configuration_is_checked(tmp_path):
    with pytest.raises(ValueError):
        grip_controller(tmp_path, grip_hold_squeeze=.05)
    model = tmp_path/'grip.urdf'
    model.write_text(GRIP_URDF)
    with pytest.raises(ValueError):
        URDFArticulation(dict(schema='robot_lab.urdf_articulation.v1', grip_joints=['other'], joints={
            'finger_joint': dict(mode='position', lower=0., upper=.045, max_velocity=.025, effort=16., kp=800., kd=20.)}),
            GRIP_URDF, model)


def test_configured_squeeze_keeps_the_full_grasp_force(tmp_path):
    controller = grip_controller(tmp_path, grip_hold_squeeze=.02)
    squeeze_on_object(controller, at=.0195)
    controller.stop()
    assert controller.targets['finger_joint'] == pytest.approx(0.)    # 19.5 mm < 20 mm: the full target is kept
