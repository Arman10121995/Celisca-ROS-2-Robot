"""Source-contract checks only; these do not qualify a simulator mission."""
from copy import deepcopy
import json
import xml.etree.ElementTree as ET

import pytest

from robot_lab_utils.urdf_articulation import URDFArticulation, add_mujoco_articulation_actuators


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
    assert controller.command({'plant_id': controller.plant_id, 'targets': {'conveyor_lift_joint': .4}}, now=1.)
    result = controller.step(position, velocity, .01, now=1.)
    assert result['position']['conveyor_lift_joint'] == pytest.approx(.20035)
    assert controller.base_inhibited


def test_coupled_stage_is_bounded_and_not_independent(controller):
    position, velocity = measurement(controller)
    controller.step(position, velocity, .01, now=1.)
    controller.heartbeat(1.)
    assert not controller.command({'plant_id': controller.plant_id, 'targets': {'telescope_stage2_joint': .1}}, now=1.)
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
