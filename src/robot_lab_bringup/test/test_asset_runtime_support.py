"""A PASS label or changed source must not enable imported robot modes."""
import copy
import hashlib
import json

import pytest

from robot_lab_utils.asset_support import apply_recorded_modes


def support(tmp_path, measured):
    profile = dict(name='test_robot', supported_simulators=['mujoco', 'pybullet'],
                   supported_modes=['display'], drive={'type':'diff'}, sensor_config={'scan_rate':5})
    record = dict(robot='test_robot', backend='pybullet', mode='nav', passed=True,
                  launch_returncode=0, workflow=measured)
    proofs = []
    for world in ('nav_empty', 'nav_obstacle'):
        path = tmp_path/(world+'.json')
        path.write_text(json.dumps(dict(record, map=world)))
        proofs.append(dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    data = dict(source_revision='fixed', drive=profile['drive'], sensor_config=profile['sensor_config'],
                screens=[dict(backend='pybullet', mode='nav', reports=proofs)])
    return profile, data


def measurement():
    return dict(outcome='succeeded', final_error_truth_m=.05, final_yaw_error_truth_deg=1.,
                truth_age_wall_s=.02, settled_sim_second=True, truth_samples=100,
                route_acceptance=dict(passed=True, min_swept_clearance_m=.1,
                    direct_route_clearance_m=-.185, robot_radius_m=.185))


def test_only_the_backend_with_recorded_body_goals_is_enabled(tmp_path):
    profile, data = support(tmp_path, measurement())
    selected = apply_recorded_modes(profile, 'fixed', data, tmp_path)
    assert selected['supported_modes_by_simulator'] == {'mujoco':['display'], 'pybullet':['display','nav']}
    assert profile['supported_modes'] == ['display']


@pytest.mark.parametrize('key,value', [('final_error_truth_m', .2),
    ('final_yaw_error_truth_deg', -18.), ('truth_age_wall_s', 2.),
    ('final_error_truth_m', float('nan')), ('outcome', 'aborted'), ('settled_sim_second', False)])
def test_pass_flag_does_not_override_failed_measurements(tmp_path, key, value):
    measured = measurement(); measured[key] = value
    profile, data = support(tmp_path, measured)
    with pytest.raises(ValueError, match='measurements'):
        apply_recorded_modes(profile, 'fixed', data, tmp_path)


def test_changed_model_control_or_evidence_is_rejected(tmp_path):
    profile, data = support(tmp_path, measurement())
    changed = copy.deepcopy(profile); changed['drive']['type'] = 'ackermann'
    with pytest.raises(ValueError, match='configuration'):
        apply_recorded_modes(changed, 'fixed', data, tmp_path)
    with pytest.raises(ValueError, match='configuration'):
        apply_recorded_modes(profile, 'different', data, tmp_path)
    (tmp_path/'nav_empty.json').write_text('{}')
    with pytest.raises(ValueError, match='checksum'):
        apply_recorded_modes(profile, 'fixed', data, tmp_path)


def test_clear_map_alone_cannot_enable_obstacle_navigation(tmp_path):
    profile, data = support(tmp_path, measurement())
    data['screens'][0]['reports'] = data['screens'][0]['reports'][:1]
    with pytest.raises(ValueError, match='both clear and obstacle'):
        apply_recorded_modes(profile, 'fixed', data, tmp_path)


@pytest.mark.parametrize('key,value', [('min_swept_clearance_m', -.01),
    ('robot_radius_m', .05), ('direct_route_clearance_m', 1.),
    ('min_swept_clearance_m', float('nan'))])
def test_route_pass_label_cannot_override_collision_or_a_missing_detour(tmp_path, key, value):
    measured = measurement(); measured['route_acceptance'][key] = value
    profile, data = support(tmp_path, measured)
    with pytest.raises(ValueError, match='measurements'):
        apply_recorded_modes(profile, 'fixed', data, tmp_path)


def test_changed_navigation_configuration_requires_new_evidence(tmp_path):
    profile, data = support(tmp_path, measurement())
    config = tmp_path/'nav.yaml'
    config.write_text('xy_goal_tolerance: 0.07\n')
    data['configuration_files'] = [dict(path=config.name,
        sha256=hashlib.sha256(config.read_bytes()).hexdigest())]
    apply_recorded_modes(profile, 'fixed', data, tmp_path)
    config.write_text('xy_goal_tolerance: 0.2\n')
    with pytest.raises(ValueError, match='configuration differs'):
        apply_recorded_modes(profile, 'fixed', data, tmp_path)


@pytest.mark.parametrize('failure', ['no_lift', 'unbounded_force', 'no_contact', 'late_watchdog'])
def test_panda_pass_label_cannot_override_physical_failures(tmp_path, failure):
    from pathlib import Path
    from robot_lab_utils.asset_support import apply_recorded_panda_hand
    root = Path(__file__).resolve().parents[3]
    report = json.loads((root/'docs/status/evidence/panda-gripper-2026-10-06/report.json').read_text())
    if failure == 'no_lift':
        report['lift_and_hold']['objects'][-1]['position'][2] = report['initial_object'][2]
    elif failure == 'unbounded_force':
        report['close']['status']['actuator_effort_per_finger_n'] = 21.
    elif failure == 'no_contact':
        report['close']['status']['contacts']['finger_joint1'] = []
    else:
        report['heartbeat_loss_wall_s'] = 2.
    xml = tmp_path/'panda.xml'; xml.write_text('Pinned XML checksum contract fixture')
    proof = tmp_path/'report.json'; proof.write_text(json.dumps(report))
    profile = dict(arm_control='panda', supported_simulators=['mujoco'], native_mjcf=str(xml))
    certificate = dict(source_revision='fixed', native_xml_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
        configuration_files=[], report=dict(path=proof.name, sha256=hashlib.sha256(proof.read_bytes()).hexdigest()))
    with pytest.raises(ValueError, match='measurements'):
        apply_recorded_panda_hand(profile, 'fixed', certificate, tmp_path)
