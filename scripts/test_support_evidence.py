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
ROOT = Path(__file__).resolve().parent.parent
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


@pytest.mark.parametrize('failure', ['fast_yaw', 'wrong_yaw_sign', 'no_strafe', 'drifting_stop',
                                  'moves_on_enable', 'loss_keeps_moving', 'missing_altitude', 'missing_body'])
def test_manual_flight_screen_requires_actual_neutral_seven_direction_stop_and_body_results(failure):
    root = Path(__file__).resolve().parents[1]
    report = json.loads((root/'docs/status/evidence/extensions-finish-2026-10-08/controls/px4/report-measured.json').read_text())
    assert module.measured_result(report, 'flight_manual_screen')
    if failure == 'fast_yaw':
        report['directions']['Turn Left']['command_max_yaw_rad_s'] = 1.
    elif failure == 'wrong_yaw_sign':
        report['directions']['Turn Right']['yaw_rad'] = .6
    elif failure == 'no_strafe':
        report['directions']['Strafe L']['body_frame_travel_m'] = [0., 0.]
    elif failure == 'drifting_stop':
        report['directions']['Forward']['stop']['drift_m'] = .8
    elif failure == 'moves_on_enable':
        report['neutral']['commands'] = 1
    elif failure == 'loss_keeps_moving':
        report['publisher_loss_drift_m'] = 1.
    elif failure == 'missing_altitude':
        report['altitude'].pop('down')
    else:
        report['body_trace']['samples'] = 0
    report['passed'] = True
    assert not module.measured_result(report, 'flight_manual_screen')


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


def test_imported_world_xy_success_is_not_navigation_while_falling():
    report = measured_nav()
    report['body_height'] = dict(expected_floor_m=0., minimum_m=0., maximum_m=.01, samples=100)
    assert module.measured_result(report, 'navigation_screen')
    report['body_height']['minimum_m'] = -13.
    assert not module.measured_result(report, 'navigation_screen')


def test_navigation_endpoint_does_not_override_failed_owned_gui_run():
    report = dict(outcome='succeeded', settled_sim_second=True,
        final_error_truth_m=.05, final_yaw_error_truth_deg=1., truth_age_wall_s=.01,
        acceptance={'limits': {'position_m': .15, 'yaw_deg': 5.}},
        gui_recording_completed=False)
    assert not module.measured_result(report, 'navigation_screen')


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


@pytest.mark.parametrize('backend', ['gazebo', 'mujoco', 'pybullet', 'isaac'])
def test_husky_screen_uses_all_four_current_physical_trials(backend):
    stage = 'isaac-body-feedback' if backend == 'isaac' else 'final-'+backend
    report = json.loads((path.parent.parent/'docs/status/evidence/extensions-finish-2026-10-08/'
                        'husky'/stage/'report-measured.json').read_text())
    assert module.measured_result(report, 'skid_drive_screen')


@pytest.mark.parametrize('failure', ['turn_crawl', 'turn_displacement', 'stuck_wheel',
    'no_stop', 'neutral_command', 'wrong_mount', 'missing_depth', 'freefall', 'missing_tilt'])
def test_husky_pass_marker_cannot_override_body_and_sensor_failures(failure):
    report = json.loads((path.parent.parent/'docs/status/evidence/extensions-finish-2026-10-08/'
                        'husky/final-mujoco/report-measured.json').read_text())
    if failure == 'turn_crawl': report['phases']['left_turn']['tail_vx'] = .3
    elif failure == 'turn_displacement': report['phases']['right_turn']['dx'] = .6
    elif failure == 'stuck_wheel': report['wheel_feedback']['rear_left_wheel_joint']['maximum'] = report['wheel_feedback']['rear_left_wheel_joint']['minimum']
    elif failure == 'no_stop': report['phases']['publisher_loss']['tail_wz'] = .3
    elif failure == 'neutral_command': report['neutral_commands'] = 1
    elif failure == 'wrong_mount': report['sensors']['frames']['lab_laser_link']['position_m'][2] += .2
    elif failure == 'missing_depth': report['sensors']['last_depth']['finite_positive'] = 0
    elif failure == 'freefall': report['body_height_m']['minimum'] = -13.
    else: report.pop('max_body_tilt_rad')
    assert not module.measured_result(report, 'skid_drive_screen')


def test_earlier_isaac_yaw_only_success_marker_is_rejected_for_drift():
    folder = path.parent.parent/'docs/status/evidence/extensions-finish-2026-10-08/husky/isaac-yaw-feedback'
    assert json.loads((folder/'report.json').read_text())['passed'] is True
    assert not module.measured_result(json.loads((folder/'report-measured.json').read_text()), 'skid_drive_screen')


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


def test_mission_screen_needs_every_step_and_truth_within_tolerance():
    report = json.loads((ROOT/'docs/status/evidence/p3-celisca-missions-2026-10-12/cells/'
                         'mujoco_labbot_celisca_floor_1_furniture.json').read_text())
    assert module.measured_result(report, 'mission_screen')
    far = json.loads(json.dumps(report))
    far['steps']['near_obstacle_b']['final_error_truth_m'] = .4
    assert not module.measured_result(far, 'mission_screen')
    aborted = json.loads(json.dumps(report))
    aborted['steps']['resume_a']['outcome'] = 'aborted'
    assert not module.measured_result(aborted, 'mission_screen')
    missing = json.loads(json.dumps(report))
    del missing['steps']['cancel_a']
    assert not module.measured_result(missing, 'mission_screen')


def test_drive_screen_needs_forward_motion_and_a_stop():
    report = json.loads((ROOT/'docs/status/evidence/p56-drive-patterns-2026-10-12/'
                         'gazebo_crab.json').read_text())
    assert module.measured_result(report, 'drive_screen')
    slow = json.loads(json.dumps(report))
    slow['phases'][0]['measured']['vx'] = .1
    assert not module.measured_result(slow, 'drive_screen')
    drifting = json.loads(json.dumps(report))
    drifting['phases'][-1]['measured']['vx'] = .2
    assert not module.measured_result(drifting, 'drive_screen')
    assert not module.measured_result({'error': 'no odometry on /odom/ground_truth', 'phases': []},
                                      'drive_screen')


def test_car_course_screen_needs_both_sections_and_clearance():
    report = {'passed': True, 'steps': {
        'slalom': {'passed': True, 'min_footprint_clearance_m': .12},
        'parking': {'passed': True, 'min_footprint_clearance_m': .05}}}
    assert module.measured_result(report, 'course_screen')
    report['steps']['parking']['min_footprint_clearance_m'] = -.001
    assert not module.measured_result(report, 'course_screen')
    report['steps']['parking'] = {'passed': False, 'note': 'not attempted after the slalom failed'}
    assert not module.measured_result(report, 'course_screen')


def test_calibration_screen_compares_the_fit_with_the_published_camera():
    report = {'camera_info': {'fx': 221.77, 'cx': 159.5, 'cy': 119.5},
              'script_calibration_pinhole': {'fx': 220.6, 'cx': 159.36, 'cy': 118.97, 'rms_px': .43}}
    assert module.measured_result(report, 'calibration_screen')
    report['script_calibration_pinhole']['cx'] = 157.5
    assert not module.measured_result(report, 'calibration_screen')
    assert not module.measured_result({'error': 'no camera image'}, 'calibration_screen')


def test_command_loss_screen_requires_a_bounded_coast_and_rest():
    report = {'phases': [{'phase': 'straight', 'measured': {'vx': .4}}],
              'publisher_loss': {'coast_distance_m': .155, 'tail_vx_mps': 0.0, 'tail_wz_rps': 0.0}}
    assert module.measured_result(report, 'command_loss_screen')
    report['publisher_loss']['tail_vx_mps'] = .1
    assert not module.measured_result(report, 'command_loss_screen')


def test_indexer_identities_and_report_shapes_of_the_new_screens():
    spec = importlib.util.spec_from_file_location('index_missions', path.with_name('r9_3_index_missions.py'))
    indexer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(indexer)
    robot_ids = indexer.robots()
    assert indexer.identity('isaac_anti_ackermann_car_car_course.json', robot_ids, False) == (
        'anti_ackermann_car', 'car_course', 'isaac', '')
    assert indexer.identity('gazebo_bumperbot_calibration_room.json', robot_ids, False) == (
        'bumperbot', 'calibration_room', 'gazebo', '')
    assert indexer.SHAPES['course_screen']({'steps': {}, 'error': 'no map after 600 s'})
    assert indexer.SHAPES['calibration_screen']({'pattern': '8x6', 'error': 'no camera image, camera info or truth'})
    assert not indexer.SHAPES['calibration_screen']({'error': 'checker produced no output'})
    assert indexer.SHAPES['command_loss_screen']({'phases': []})
    assert not indexer.SHAPES['mission_screen']({'samples': []})
