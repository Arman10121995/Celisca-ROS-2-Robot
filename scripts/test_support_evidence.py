"""Guard against unmeasured support claims and fabricated release readiness."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml

path = Path(__file__).with_name('r9_3_support_matrix.py')
spec = importlib.util.spec_from_file_location('support_matrix',path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def framework(tmp_path,report):
    output = tmp_path/'nav.json'
    output.write_text(json.dumps(report))
    (tmp_path/'hashes.json').write_text(json.dumps({'source':'a'*64}))
    index = {'records':[{'robot_id':'four_wheel_steer_car','environment_id':'nav_empty',
        'simulator':'isaac','task_type':'navigation','steering_mode':'crab','report':'nav.json',
        'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'source_hashes':'hashes.json',
        'revision':'unit-fixture','date':'unit-fixture','report_kind':'navigation_screen',
        'scope':'Unit fixture for a single measured goal, not runtime evidence'}]}
    (tmp_path/'index.yaml').write_text(yaml.safe_dump(index))
    return module.SupportMatrixFramework(tmp_path/'index.yaml',root=tmp_path)


def measured_nav():
    return {'outcome':'succeeded','settled_sim_second':True,'final_error_truth_m':.08,
            'final_yaw_error_truth_deg':2.1,'truth_age_wall_s':.04,
            'acceptance':{'limits':{'position_m':.15,'yaw_deg':5}}}


def test_cartesian_matrix_rechecks_the_same_physical_screen():
    root = Path(__file__).resolve().parents[1]
    report = json.loads((root/'docs/status/evidence/panda-cartesian-2026-10-06/report.json').read_text())
    assert module.measured_result(report, 'arm_cartesian_screen')
    report['planner_clean_exit'] = False
    assert not module.measured_result(report, 'arm_cartesian_screen')


def test_short_valid_screen_is_partial_and_cannot_transfer_to_other_maps(tmp_path):
    catalog = framework(tmp_path,measured_nav())
    cell = module.SupportCell('four_wheel_steer_car','nav_empty','isaac','navigation',steering_mode='crab')
    catalog._determine_support_level(cell)
    assert cell.support_level == module.SupportLevel.PARTIALLY_SUPPORTED
    for robot,world,task,pattern in [('four_wheel_steer_car','aerial_course','navigation','crab'),
                                   ('bumperbot','nav_empty','navigation',''),
                                   ('four_wheel_steer_car','nav_empty','mapping','crab'),
                                   ('four_wheel_steer_car','nav_empty','navigation','pivot')]:
        other = module.SupportCell(robot,world,'isaac',task,steering_mode=pattern)
        catalog._determine_support_level(other)
        assert other.support_level == module.SupportLevel.NOT_TESTED


@pytest.mark.parametrize('change',[{'passed':True}, {'outcome':'aborted'},
                                {'final_error_truth_m':.3},{'truth_age_wall_s':4},
                                {'final_yaw_error_truth_deg':float('nan')}])
def test_pass_marker_does_not_override_failed_measurements(tmp_path,change):
    report = measured_nav()
    if change == {'passed':True}:
        report = change
    else:
        report.update(change)
    catalog = framework(tmp_path,report)
    cell = module.SupportCell('four_wheel_steer_car','nav_empty','isaac','navigation',steering_mode='crab')
    catalog._determine_support_level(cell)
    assert cell.support_level == module.SupportLevel.EXPERIMENTAL


def test_missing_or_changed_artifact_is_unverified(tmp_path):
    catalog = framework(tmp_path,measured_nav())
    (tmp_path/'nav.json').write_text('{"passed":true}')
    cell = module.SupportCell('four_wheel_steer_car','nav_empty','isaac','navigation',steering_mode='crab')
    catalog._determine_support_level(cell)
    assert cell.support_level == module.SupportLevel.NOT_TESTED
    (tmp_path/'nav.json').unlink()
    catalog._determine_support_level(cell)
    assert cell.support_level == module.SupportLevel.NOT_TESTED


def test_source_manifest_checksum_cannot_be_replaced(tmp_path):
    catalog = framework(tmp_path, measured_nav())
    source = tmp_path/'hashes.json'
    catalog.records[0]['source_hashes_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    source.write_text(json.dumps({'different_source': 'b'*64}))
    cell = module.SupportCell('four_wheel_steer_car', 'nav_empty', 'isaac', 'navigation', steering_mode='crab')
    catalog._determine_support_level(cell)
    assert cell.support_level == module.SupportLevel.NOT_TESTED
    assert 'Source manifest hash mismatch' in cell.limitations[0]


@pytest.mark.parametrize('failure', ['no_motion', 'neutral_motion', 'no_stop', 'wrong_mount', 'no_depth', 'bad_calibration'])
def test_drive_pass_marker_cannot_override_physical_or_sensor_failure(failure):
    report = json.loads((path.parent.parent/'docs/status/evidence/turtlebot4-sensors-2026-10-06/asset_turtlebot4_standard-mujoco/report.json').read_text())
    assert module.measured_result(report, 'drive_sensor_screen')
    if failure == 'no_motion': report['phases']['forward']['tail_vx'] = 0.
    elif failure == 'neutral_motion': report['neutral_commands'] = 1
    elif failure == 'no_stop': report['phases']['publisher_loss']['tail_wz'] = .6
    elif failure == 'wrong_mount': report['sensors']['mounted_frames']['rplidar_link']['position'][2] += .1
    elif failure == 'no_depth': report['sensors']['last_depth']['finite_positive'] = 0
    else: report['sensors']['camera_info']['k'][0] = 1.
    assert not module.measured_result(report, 'drive_sensor_screen')


@pytest.mark.parametrize('failure', [
    'no_motion', 'neutral_motion', 'no_stop', 'wrong_mount', 'wrong_scan',
    'no_obstacles', 'fabricated_depth', 'static_joint', 'tilt', 'nonfinite_tilt',
])
def test_turtlebot3_lidar_screen_rechecks_physical_trace_envelopes(failure):
    report = json.loads((path.parent.parent/'docs/status/evidence/turtlebot3-sensors-2026-10-07/'
                        'asset_turtlebot3_burger-mujoco/report-measured.json').read_text())
    assert module.measured_result(report, 'drive_lidar_screen')
    if failure == 'no_motion': report['phases']['forward']['tail_vx'] = 0.
    elif failure == 'neutral_motion': report['phases']['armed_neutral']['dx'] = .3
    elif failure == 'no_stop': report['phases']['publisher_loss']['tail_wz'] = .6
    elif failure == 'wrong_mount': report['sensors']['mounted_frames']['imu_link']['position'][2] += .1
    elif failure == 'wrong_scan': report['sensors']['last_scan']['range_max'] = 12.
    elif failure == 'no_obstacles': report['sensors']['max_finite_scan_points'] = 0
    elif failure == 'fabricated_depth': report['sensors']['depth_messages'] = 100
    elif failure == 'static_joint': report['joint_position_ranges']['wheel_right_joint'] = 0.
    elif failure == 'tilt': report['max_body_tilt_rad'] = .42
    else: report['max_body_tilt_rad'] = float('nan')
    assert not module.measured_result(report, 'drive_lidar_screen')


def test_active_tasks_block_release_without_inventing_scope_approval(tmp_path):
    catalog = framework(tmp_path,measured_nav())
    catalog.ledger = {'tasks':{'R5':{'tasks':{'R5.6':{'state':'active','evidence':['nav.json']}}}}}
    report = catalog.report()
    assert not report['acceptance_criteria']['required_tasks_pass']
    assert report['status'] == 'partial'
    assert any('R5.6: active' in gate['blocking_issues'] for gate in report['release_gates'].values())
    assert all(gate['status']=='blocked' for gate in report['release_gates'].values())


def test_successful_default_mode_route_is_not_proof_of_crab(tmp_path):
    report = measured_nav()
    report['drive_configuration'] = {'passed': True, 'expected_mode': 'crab',
        'configuration': {'type': 'four_wheel_steer', 'steering_mode': 'ackermann'}}
    catalog = framework(tmp_path, report)
    cell = module.SupportCell('four_wheel_steer_car', 'nav_empty', 'isaac',
                              'navigation', steering_mode='crab')
    catalog._determine_support_level(cell)
    assert cell.support_level == module.SupportLevel.NOT_TESTED
    assert 'configuration' in cell.limitations[0]


def test_flight_requires_measured_lift_and_disarm_even_with_pass_flag():
    report={'passed':True,'truth_messages':1000,'truth_age_wall_s':.05,
        'rotor_max_measured_rad_s':80,'phases':{
            'idle':{'start_truth':[0,0,.2],'end_truth':[0,0,.2],'state':{'armed':False}},
            'hover_truth':[0,0,3.2],
            'goal':{'target_enu':[2,1,3.2],'final_truth':[2,1,3.2]},
            'drive':{'start_truth':[2,1,3.2],'stop_truth':[3.2,1,3.2],'held_truth':[3.2,1,3.2]},
            'land_truth':[3.2,1,.2],'land_state':{'armed':False,'phase':'idle'}}}
    assert module.measured_result(report,'flight_screen')
    report['phases']['hover_truth']=[0,0,.2]
    assert not module.measured_result(report,'flight_screen')
    report['phases']['hover_truth']=[0,0,3.2]
    report['phases']['land_state']['armed']=True
    assert not module.measured_result(report,'flight_screen')
