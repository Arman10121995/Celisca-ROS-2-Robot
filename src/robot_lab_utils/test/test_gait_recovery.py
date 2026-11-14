"""Bounded slip/support-loss stance recovery (R10.6)."""
from robot_lab_utils.gait_recovery import SlipRecovery


def test_fault_pauses_then_resumes_at_half_speed_after_settling():
    r = SlipRecovery()
    assert r.fault('Measured contact-kinematic slip persisted', 10.) == 'recover'
    assert r.speed_scale(10.) == 0.
    assert r.tick(10.5, stance_reached=False, supported=True, slip_clear=True) is None
    assert r.tick(11., True, True, True) is None                 # settling starts
    assert r.tick(11.5, True, False, True) is None               # support lost: settle restarts
    assert r.tick(12., True, True, True) is None
    assert r.tick(13.05, True, True, True) == 'resumed'
    assert r.speed_scale(14.) == .5 and r.speed_scale(16.2) == 1.


def test_repeated_faults_are_one_recovery_and_the_budget_stops():
    r = SlipRecovery(budget=2)
    assert r.fault('slip', 0.) == 'recover' and r.fault('slip', .1) == 'recover'
    assert r.state(.1)['recoveries_in_window'] == 1
    r.tick(.2, True, True, True); r.tick(1.3, True, True, True)
    assert r.fault('slip', 5.) == 'recover'
    r.tick(5.1, True, True, True); r.tick(6.2, True, True, True)
    assert r.fault('slip', 10.) == 'stop'                          # third in a minute
    assert r.fault('slip', 70.) == 'recover'                       # window has passed


def test_recovery_times_out():
    r = SlipRecovery(timeout_s=10.)
    r.fault('support', 0.)
    assert r.tick(5., False, False, False) is None
    assert r.tick(10.5, False, False, False) == 'timeout' and r.speed_scale(10.5) == 1.
