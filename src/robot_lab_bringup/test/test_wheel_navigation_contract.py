"""Prevent success reports without fresh truth and unreachable rotation windows."""
import copy
from pathlib import Path

import pytest
import yaml

from robot_lab_utils.qualification import navigation_acceptance


@pytest.mark.parametrize('robot', ['four_wheel_steer_car', 'mecanum_car'])
def test_rotation_window_does_not_stop_translation_before_position_goal(robot):
    src = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((src/'robot_lab_navigation/config/robots'/
                             (robot+'.yaml')).read_text())['controller_server']['ros__parameters']
    goal = params['general_goal_checker']
    assert 0 < params['FollowPath']['xy_goal_tolerance'] <= goal['xy_goal_tolerance']
    assert goal['yaw_goal_tolerance'] < 0.1


@pytest.mark.parametrize('key,value', [
    ('outcome', 'aborted'), ('final_error_truth_m', 0.25),
    ('final_yaw_error_truth_deg', -14.0), ('truth_age_wall_s', 3.0),
    ('final_error_truth_m', float('nan')), ('final_yaw_error_truth_deg', None)])
def test_action_success_alone_cannot_pass_measured_acceptance(key, value):
    report = {'outcome': 'succeeded', 'final_error_truth_m': 0.08,
              'final_yaw_error_truth_deg': -2.0, 'truth_age_wall_s': 0.01}
    assert navigation_acceptance(report, 0.15, 5)['passed']
    bad = copy.copy(report)
    bad[key] = value
    assert not navigation_acceptance(bad, 0.15, 5)['passed']
