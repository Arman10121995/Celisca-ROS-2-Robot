"""RMF motion-power calibration from sampled actuator power (R10.2)."""
from types import SimpleNamespace

import numpy as np
import pytest

from robot_lab_utils.fleet_energy_calibration import (G, calibrate, description_mass, fit_rmf_mechanical,
                                                      fleet_calibration, power_samples, rmf_systems)


def synthetic(mass=2., inertia=.05, friction=.04, efficiency=.75, noise=0., seed=0):
    """A drive with speed steps and turns; wheel power from the RMF model itself."""
    rng = np.random.default_rng(seed)
    t = np.arange(0., 72., .02)
    phase = (t // 4).astype(int)
    ramp = 1 - np.exp(-(t % 4) / np.where(phase % 4 < 2, .3, 1.2))
    v = np.where(phase % 3 == 0, .25, np.where(phase % 3 == 1, .1, .0)) * ramp
    w = np.where(phase % 5 < 2, .5, np.where(phase % 5 == 2, .2, 0.)) * ramp
    a, alpha = np.abs(np.gradient(v, t)), np.abs(np.gradient(w, t))
    mechanical = (mass * a * v + inertia * alpha * w + friction * mass * G * v) * efficiency
    mechanical = mechanical + noise * rng.standard_normal(len(t))
    joints = [dict(t=float(ti), names=['left', 'right'], v=[1., 1.], e=[float(p / 2), float(p / 2)], phase=int(k))
              for ti, p, k in zip(t, mechanical, phase)]
    bodies = [dict(t=float(ti), v=float(vi), w=float(wi)) for ti, vi, wi in zip(t, v, w)]
    return joints, bodies


def test_fit_recovers_rmf_parameters_and_held_out_energy():
    joints, bodies = synthetic()
    samples = power_samples(joints, bodies, ['left', 'right'], .75, smooth_s=.02)
    result = calibrate(samples, 2.)
    assert result['mass'] * result['friction'] == pytest.approx(2. * .04, rel=.05)
    assert result['mass'] == pytest.approx(2., rel=.25)
    assert result['inertia'] == pytest.approx(.05, rel=.25)
    assert result['held_out_max_error'] < .05
    assert [fold['name'] for fold in result['held_out']] == ['even_to_odd', 'odd_to_even']


def test_noisy_power_still_bounds_energy_error():
    joints, bodies = synthetic(noise=.02, seed=3)
    samples = power_samples(joints, bodies, ['left', 'right'], .75)
    result = calibrate(samples, 2.)
    assert result['held_out_max_error'] < .1


def test_missing_joints_and_flat_drives_are_rejected():
    joints, bodies = synthetic()
    with pytest.raises(ValueError):
        power_samples(joints, bodies, ['left', 'absent'], .75)
    samples = power_samples(joints, bodies, ['left', 'right'], .75)
    flat = {key: value.copy() for key, value in samples.items()}
    flat['w'][:] = 0.
    flat['alpha'][:] = 0.
    with pytest.raises(ValueError, match='excite'):
        fit_rmf_mechanical(flat)


def test_description_mass_sums_links():
    urdf = '<robot name="r"><link name="a"><inertial><mass value="1.5"/></inertial></link>' \
           '<link name="b"><inertial><mass value="0.25"/></inertial></link></robot>'
    assert description_mass(urdf) == pytest.approx(1.75)
    with pytest.raises(ValueError):
        description_mass('<robot name="r"><link name="a"/></robot>')


def test_fleet_calibration_requires_every_bound_model():
    session = dict(members=[dict(model='bumperbot', source_description_sha256='b1'),
                            dict(model='labbot', source_description_sha256='l1')])
    bumperbot = dict(source_description_sha256='b1', rmf_mechanical=dict(
        mass=1., inertia=.01, friction=.15, held_out_max_error=.06, evidence='x'))
    labbot = dict(source_description_sha256='l1', rmf_mechanical=dict(
        mass=3., inertia=.02, friction=.03, held_out_max_error=.15, evidence='y'))
    energy = dict(nominal_voltage_V=12., effort_sources=dict(bumperbot=bumperbot))
    assert fleet_calibration(session, dict(energy=energy)) is None          # labbot uncalibrated
    energy['effort_sources']['labbot'] = labbot
    worst = fleet_calibration(session, dict(energy=energy))
    # Largest friction force (mu m): 0.15 x 1 kg, expressed at the 3 kg fleet mass.
    assert (worst['mass'], worst['inertia']) == (3., .02) and worst['friction'] == pytest.approx(.05)
    labbot['rmf_mechanical']['held_out_max_error'] = .4
    assert fleet_calibration(session, dict(energy=energy)) is None          # fits too poorly
    labbot['rmf_mechanical']['held_out_max_error'] = .15
    labbot['source_description_sha256'] = 'changed'
    assert fleet_calibration(session, dict(energy=energy)) is None          # description changed since calibration
    labbot['source_description_sha256'] = 'l1'
    del energy['nominal_voltage_V']
    assert fleet_calibration(session, dict(energy=energy)) is None


def test_rmf_systems_convert_authored_battery_units():
    made = {}

    def recorder(kind):
        return SimpleNamespace(make=lambda *values: made.setdefault(kind, []).append(values) or (kind, values))
    module = SimpleNamespace(BatterySystem=recorder('battery'), MechanicalSystem=recorder('mechanical'),
                             PowerSystem=recorder('power'), SimpleMotionPowerSink=lambda *a: ('motion', a),
                             SimpleDevicePowerSink=lambda *a: ('device', a))
    energy = dict(nominal_voltage_V=12., capacity_J=180000., charge_W=120., hotel_W=3.)
    rmf_systems(module, energy, dict(mass=1.2, inertia=.01, friction=.05))
    voltage, capacity_ah, current = made['battery'][0]
    assert voltage == 12. and capacity_ah == pytest.approx(180000 / 12 / 3600) and current == pytest.approx(10.)
    assert made['mechanical'][0] == (1.2, .01, .05) and made['power'][0] == (3.,)
