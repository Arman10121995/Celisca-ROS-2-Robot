"""Check the four-wheel contract, limits, and independent encoder averaging."""
import math

import pytest

from robot_lab_utils.skid_steer import SkidSteerDrive, wheel_force_limit, wheel_friction_axes, bullet_contact_parameters
from robot_lab_utils.drive_kinematics import drive_from_config


def config(**changes):
    return dict(left_wheel_joints=['fl', 'rl'], right_wheel_joints=['fr', 'rr'],
        wheel_radius=.1651, wheel_separation=.5708, **changes)


def test_all_four_wheels_reverse_and_spin_without_steering():
    drive = SkidSteerDrive(config())
    forward = drive.targets(.4, 0)
    assert len(forward.velocity) == 4 and not forward.position
    assert all(rate == pytest.approx(.4/.1651) for rate in forward.velocity.values())
    reverse = drive.targets(-.4, 0)
    assert all(rate < 0 for rate in reverse.velocity.values())
    turn = drive.targets(0, .8)
    assert turn.velocity['fl'] == turn.velocity['rl'] < 0
    assert turn.velocity['fr'] == turn.velocity['rr'] > 0
    assert drive.body_twist(turn.velocity) == pytest.approx((0, .8))


def test_encoders_use_both_wheels_on_each_side():
    drive = SkidSteerDrive(config())
    vx, wz = drive.body_twist(dict(fl=1, rl=3, fr=2, rr=4))
    assert vx == pytest.approx(.1651*2.5)
    assert wz == pytest.approx(.1651/.5708)


def test_acceleration_stop_and_nonfinite_commands():
    drive = SkidSteerDrive(config(max_speed=.5, max_accel=.4,
                               max_angular_speed=1, max_angular_accel=2))
    assert drive.targets(10, 10, .1).twist == pytest.approx((.04, .2))
    drive.reset()
    assert drive.targets(0, 0, .1).twist == (0, 0)
    assert drive.targets(math.inf, math.nan).twist == (0, 0)


@pytest.mark.parametrize('changes', [dict(left_wheel_joints='fl'),
    dict(right_wheel_joints=['fr']), dict(left_wheel_joints=['fl','fl']),
    dict(right_wheel_joints=['fr','fl']), dict(left_wheel_joints=['fl',''])])
def test_reject_invalid_or_shared_physical_joints(changes):
    conf = config()
    conf.update(changes)
    with pytest.raises(ValueError):
        SkidSteerDrive(conf)


def test_missing_encoder_cannot_be_reported_as_odometry():
    drive = SkidSteerDrive(config())
    with pytest.raises(ValueError):
        drive.body_twist(dict(fl=1, fr=1))


def test_factory_selects_four_joint_controller_and_preserves_diff_default():
    drive = drive_from_config(dict(config(), type='skid_steer'))
    assert drive.wheel_joints == ['fl','rl','fr','rr']
    assert drive_from_config({}).kind == 'diff'


@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf, True])
def test_invalid_motor_caps_rejected(value):
    with pytest.raises(ValueError):
        wheel_force_limit({'wheel_force_limit':value})


def test_motor_caps_are_explicit_and_legacy_defaults_unchanged():
    assert wheel_force_limit({}) == 5
    assert wheel_force_limit({'wheel_force_limit':25}) == 25


def test_source_anisotropic_tires_are_optional_and_validated():
    assert wheel_friction_axes({}) is None
    assert wheel_friction_axes({'pybullet_anisotropic_wheel_friction':[1,.5,1]}) == [1,.5,1]
    for values in ([1,1],[1,-.5,1],[math.nan,1,1],'1 .5 1'):
        with pytest.raises(ValueError):
            wheel_friction_axes({'pybullet_anisotropic_wheel_friction':values})


def test_contact_solver_choice_does_not_override_other_robot_defaults():
    assert bullet_contact_parameters({}) == {}
    assert bullet_contact_parameters({'pybullet_enable_cone_friction':False}) == {'enableConeFriction':0}
    assert bullet_contact_parameters({'pybullet_enable_cone_friction':True}) == {'enableConeFriction':1}
    for value in (0,1,'false',[],{}):
        with pytest.raises(ValueError):
            bullet_contact_parameters({'pybullet_enable_cone_friction':value})


def test_body_yaw_feedback_cannot_survive_stop_or_watchdog_and_respects_motor_limits():
    conf=config(skid_yaw_rate_feedback=dict(kp=1.,ki=4.,max_correction=3.,max_wheel_speed=5.))
    drive=SkidSteerDrive(conf)
    for _ in range(100):
        target=drive.targets(0,1,.02,measured_wz=.2)
    assert all(abs(v)<=5 for v in target.velocity.values())
    assert target.velocity['fr']>1.0*.5708/(2*.1651)
    assert target.twist==(0,1)  # commanded body limit is unchanged
    assert drive.targets(0,0,.02,measured_wz=.4).velocity==dict.fromkeys(drive.wheel_joints,0.)
    assert drive.targets(0,1,.02,measured_wz=math.nan).velocity==dict.fromkeys(drive.wheel_joints,0.)
    assert drive.targets(math.nan,1,.02,measured_wz=.2).velocity==dict.fromkeys(drive.wheel_joints,0.)
    drive.reset()
    assert drive.targets(0,0,.02,measured_wz=0).twist==(0,0)


def test_backend_without_body_feedback_retains_the_declared_encoder_model():
    drive=SkidSteerDrive(config(skid_yaw_rate_feedback=dict(kp=1.,ki=4.,max_correction=3.,max_wheel_speed=10.)))
    assert drive.body_twist(drive.targets(.2,.8,.02).velocity)==pytest.approx((.2,.8))


def test_linear_feedback_cancels_turn_crawl_without_inventing_neutral_commands():
    drive=SkidSteerDrive(config(skid_yaw_rate_feedback=dict(kp=1.,ki=4.,max_correction=3.,max_wheel_speed=10.,
        linear_kp=1.,linear_ki=2.,max_linear_correction=.5)))
    rates=drive.targets(0,1,.02,measured_wz=.8,measured_vx=.3).velocity
    assert sum(rates.values())<0  # same signed correction on all four wheels
    assert rates['fl']==rates['rl'] and rates['fr']==rates['rr']
    assert drive.targets(0,0,.02,measured_wz=.1,measured_vx=.1).velocity==dict.fromkeys(drive.wheel_joints,0.)
    assert drive.targets(.2,0,.02,measured_wz=0,measured_vx=math.nan).twist==(0,0)
