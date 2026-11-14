"""Calibrate Open-RMF's motion power model from observed actuator power (R10.2).

Open-RMF (rmf_battery SimpleMotionPowerSink) predicts drive power as

    P = m |a| v + I |alpha| |w| + mu m g v

and integrates it over a planned trajectory. Here m, I and mu are effective
parameters fitted (non-negative least squares) so that the model reproduces
the energy of each drive phase, measured as sampled wheel power
sum(max(0, rate * effort)) / efficiency against the observed body speed and
acceleration. The drive is split into two halves (even and odd phases); each
half is fitted and checked on the other, and the larger held-out energy error
decides whether RMF may use the calibration. The model has no term for
turning in place at constant rate, so scrub-heavy drives fit less well.
This calibrates the simulated drivetrain, never a hardware battery; the
battery capacity, voltage and hotel load stay authored.

    python3 -m robot_lab_utils.fleet_energy_calibration --session <session.json> --member alpha --output <dir>
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

G = 9.81
# (linear m/s, angular rad/s, seconds): forward/back pairs keep the robot near its start.
PROFILE = [(.05, 0., 3.), (-.05, 0., 3.), (.15, 0., 3.), (-.15, 0., 3.), (.25, 0., 3.), (-.25, 0., 3.),
           (0., 0., 1.5), (.12, .5, 3.), (-.12, -.5, 3.), (.2, 0., 2.), (0., 0., 1.), (-.2, 0., 2.),
           (.3, 0., 2.), (-.3, 0., 2.), (0., .6, 2.), (0., -.6, 2.), (.1, -.3, 3.), (-.1, .3, 3.), (0., 0., 2.)]


def description_mass(urdf_text):
    """Total link mass of a URDF/SDF description, in kg."""
    root = ET.fromstring(urdf_text)
    total = sum(float(mass.get('value', mass.text or 0.)) for mass in root.iter('mass'))
    if not math.isfinite(total) or total <= 0:
        raise ValueError('The robot description has no positive inertial mass')
    return total


def power_samples(joint_rows, body_rows, joints, efficiency, smooth_s=.2):
    """Align wheel power with body motion on the simulation clock.

    joint_rows: [dict(t, names, v, e)]; body_rows: [dict(t, v, w)] (ground-truth
    body twist). Returns arrays t, P (W), v, w, a, alpha and the phase index of
    each sample when rows carry 'phase'.
    """
    rows = sorted((r for r in joint_rows if set(joints) <= set(r['names'])), key=lambda r: r['t'])
    body = sorted(body_rows, key=lambda r: r['t'])
    if len(rows) < 20 or len(body) < 20:
        raise ValueError('Too few complete actuator/body samples to calibrate')
    t = np.array([r['t'] for r in rows])
    power = np.array([sum(max(0., r['v'][r['names'].index(j)] * r['e'][r['names'].index(j)]) for j in joints)
                      for r in rows]) / efficiency
    bt = np.array([r['t'] for r in body])
    keep = np.concatenate([[True], np.diff(bt) > 0])
    bt = bt[keep]
    v = np.interp(t, bt, np.abs(np.array([r['v'] for r in body]))[keep])
    w = np.interp(t, bt, np.abs(np.array([r['w'] for r in body]))[keep])
    # Smooth before differentiating; contact noise would dominate raw differences.
    vs, ws = (_smooth(t, x, smooth_s) for x in (v, w))
    a, alpha = np.abs(np.gradient(vs, t)), np.abs(np.gradient(ws, t))
    phase = np.array([r.get('phase', 0) for r in rows])
    ok = np.isfinite(power) & np.isfinite(v) & np.isfinite(a) & np.isfinite(alpha)
    return dict(t=t[ok], P=power[ok], v=vs[ok], w=ws[ok], a=a[ok], alpha=alpha[ok], phase=phase[ok])


def _smooth(t, x, window):
    out = np.empty_like(x)
    start = 0
    for i, ti in enumerate(t):
        while t[start] < ti - window / 2:
            start += 1
        end = np.searchsorted(t, ti + window / 2, side='right')
        out[i] = x[start:end].mean()
    return out


def rmf_power(fit, s):
    return fit['mass'] * s['a'] * s['v'] + fit['inertia'] * s['alpha'] * s['w'] + fit['friction'] * fit['mass'] * G * s['v']


def _integral(values, t):
    # Integrate contiguous stretches only; a subset of phases has gaps.
    dt = np.diff(t)
    dt = np.where(dt < .5, dt, 0.)
    return float(np.sum((values[1:] + values[:-1]) / 2 * dt))


def fit_rmf_mechanical(samples):
    """Effective RMF mass, inertia and friction matching each phase's energy."""
    from scipy.optimize import nnls
    rows, energy = [], []
    for phase in np.unique(samples['phase']):
        part = subset(samples, samples['phase'] == phase)
        if len(part['t']) < 3:
            continue
        rows.append([_integral(part['a'] * part['v'], part['t']), _integral(part['alpha'] * part['w'], part['t']),
                     _integral(G * part['v'], part['t'])])
        energy.append(_integral(part['P'], part['t']))
    rows, energy = np.array(rows), np.array(energy)
    if len(rows) < 3 or np.linalg.matrix_rank(rows) < 3:
        raise ValueError('The drive did not excite acceleration, turning and linear speed separately')
    (mass_term, inertia, friction_term), _ = nnls(rows, energy)
    # RMF rejects zero parameters; a floor keeps a term that measured ~0 negligible.
    mass = max(float(mass_term), 1e-3)
    fit = dict(mass=mass, inertia=max(float(inertia), 1e-6), friction=max(float(friction_term), 1e-9) / mass)
    residual = samples['P'] - rmf_power(fit, samples)
    total = np.sum((samples['P'] - samples['P'].mean()) ** 2)
    fit.update(samples=int(len(samples['P'])), phases=int(len(rows)), rms_W=float(np.sqrt(np.mean(residual ** 2))),
               r2=float(1 - np.sum(residual ** 2) / total) if total > 0 else None)
    return fit


def energy_error(fit, samples):
    """Observed vs RMF-predicted drive energy over the samples (trapezoid, sim time)."""
    if len(samples['t']) < 2:
        raise ValueError('Too few samples for an energy comparison')
    observed, predicted = _integral(samples['P'], samples['t']), _integral(rmf_power(fit, samples), samples['t'])
    return dict(observed_J=observed, predicted_J=predicted,
                relative_error=(predicted - observed) / observed if observed > 0 else None)


def subset(samples, mask):
    return {key: value[mask] for key, value in samples.items()}


def calibrate(samples, description_mass_kg=None):
    """Fit each half of the drive, check it on the other, then fit all phases."""
    even = samples['phase'] % 2 == 0
    folds = []
    for name, fit_mask in (('even_to_odd', even), ('odd_to_even', ~even)):
        fit = fit_rmf_mechanical(subset(samples, fit_mask))
        folds.append(dict(name=name, fit=fit, check=energy_error(fit, subset(samples, ~fit_mask))))
    errors = [abs(f['check']['relative_error']) for f in folds if f['check']['relative_error'] is not None]
    final = fit_rmf_mechanical(samples)
    return dict(final, held_out=folds, held_out_max_error=max(errors) if len(errors) == 2 else None,
                description_mass_kg=description_mass_kg, all_samples=energy_error(final, samples),
                model='rmf_battery SimpleMotionPowerSink with effective parameters: P = m|a|v + I|alpha||w| + mu m g v')


def rmf_systems(battery_module, energy, calibration):
    """Open-RMF battery, motion and ambient sinks from authored energy + calibration."""
    voltage = float(energy['nominal_voltage_V'])
    capacity_ah = float(energy['capacity_J']) / (voltage * 3600.)
    charging_a = float(energy['charge_W']) / voltage
    battery = battery_module.BatterySystem.make(voltage, capacity_ah, charging_a)
    mechanical = battery_module.MechanicalSystem.make(calibration['mass'], calibration['inertia'], calibration['friction'])
    ambient = battery_module.PowerSystem.make(float(energy['hotel_W']))
    tool = battery_module.PowerSystem.make(1e-6)
    if not all((battery, mechanical, ambient, tool)):
        raise ValueError('Open-RMF rejected the calibrated battery/mechanical parameters')
    return (battery, battery_module.SimpleMotionPowerSink(battery, mechanical),
            battery_module.SimpleDevicePowerSink(battery, ambient), battery_module.SimpleDevicePowerSink(battery, tool))


MAX_HELD_OUT_ERROR = .25


def fleet_calibration(session, infrastructure):
    """The most power-hungry qualified calibration among the session's models, or None.

    Every member's source must carry a calibration bound to its description
    hash whose held-out energy error is within MAX_HELD_OUT_ERROR; one
    unqualified model leaves RMF energy accounting disabled.
    """
    energy = infrastructure.get('energy') or {}
    sources = energy.get('effort_sources') or {}
    chosen = []
    for member in session['members']:
        source = sources.get(member['model']) or {}
        calibration = source.get('rmf_mechanical')
        if (not calibration or source.get('source_description_sha256') != member.get('source_description_sha256')
                or 'nominal_voltage_V' not in energy
                or not 0 <= calibration.get('held_out_max_error', math.inf) <= MAX_HELD_OUT_ERROR):
            return None
        chosen.append(calibration)
    # One RMF vehicle model per fleet: take each term from its largest member.
    mass = max(c['mass'] for c in chosen)
    worst = dict(mass=mass, inertia=max(c['inertia'] for c in chosen),
                 friction=max(c['friction'] * c['mass'] for c in chosen) / mass)
    return dict(worst, evidence=sorted({c.get('evidence', '') for c in chosen}))


def main(args=None):
    """Drive one fleet member through PROFILE and write its calibration."""
    import argparse
    import time
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--session', required=True)
    parser.add_argument('--member', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--efficiency', type=float, default=None)
    parser.add_argument('--from-samples', action='store_true', help='Refit the saved samples.json; no driving')
    options = parser.parse_args(args)
    session = json.loads(Path(options.session).read_text())
    member = next(m for m in session['members'] if m['id'] == options.member)
    import yaml
    energy = (yaml.safe_load(Path(session['infrastructure_file']).read_text()).get('energy', {})
              if session.get('infrastructure_file') else {})
    efficiency = options.efficiency or float(energy.get('drive_efficiency', 1.))
    output = Path(options.output)
    output.mkdir(parents=True, exist_ok=True)
    if options.from_samples:
        saved = json.loads((output / 'samples.json').read_text())
        return _write_result(output, member, saved['joints'], saved['bodies'], efficiency, saved.get('completed_profile', True))
    rclpy.init()
    node = rclpy.create_node('fleet_energy_calibration')
    joints, bodies, state, phase = [], [], {}, [-1]
    name = options.member
    node.create_subscription(JointState, '/%s/joint_states' % name, lambda m: joints.append(dict(
        t=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, names=list(m.name), v=list(m.velocity),
        e=list(m.effort), phase=phase[0])), 50)
    node.create_subscription(Odometry, '/%s/odom/ground_truth' % name, lambda m: bodies.append(dict(
        t=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        v=math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), w=m.twist.twist.angular.z)), 50)
    node.create_subscription(String, '/fleet/state', lambda m: state.update(json.loads(m.data)), 10)
    heartbeat = node.create_publisher(String, '/fleet/operator_heartbeat', 10)
    command = node.create_publisher(String, '/fleet/command', 10)
    drive = node.create_publisher(Twist, '/%s/key_vel' % name, 10)
    operator = 'energy-calibration'
    deadline = time.monotonic() + 120.
    started, index, phase_at, requested_at = False, 0, 0., -math.inf
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if not state:
                continue
            base = dict(session_id=state['session_id'], operator_id=operator)
            heartbeat.publish(String(data=json.dumps(base)))
            now = time.monotonic()
            row = next((r for r in state.get('members', []) if r['id'] == name), {})
            if not started:
                # A stale state can still name this operator; wait for a fresh
                # lease and an acknowledged manual mode before driving.
                if row.get('mode') == 'manual':
                    started, phase_at, phase[0] = True, now, 0
                elif (state.get('operator_id') == operator and state.get('operator_fresh') and row.get('ready')
                      and now - requested_at > .5):
                    command.publish(String(data=json.dumps(dict(base, op='manual', targets=[name]))))
                    requested_at = now
                continue
            if now - phase_at >= PROFILE[index][2]:
                index += 1
                phase_at, phase[0] = now, index
                if index >= len(PROFILE):
                    break
            twist = Twist()
            twist.linear.x, twist.angular.z = PROFILE[index][0], PROFILE[index][1]
            drive.publish(twist)
        if state:
            command.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator,
                                                        op='all_stop', targets=[]))))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    completed = index >= len(PROFILE)
    (output / 'samples.json').write_text(json.dumps(dict(joints=joints, bodies=bodies, completed_profile=completed)))
    return _write_result(output, member, joints, bodies, efficiency, completed)


def _write_result(output, member, joints, bodies, efficiency, completed):
    description = ET.parse(member['description'])
    names = [joint.get('name') for joint in description.findall('.//ros2_control/joint')
             if joint.find("command_interface[@name='velocity']") is not None]
    samples = power_samples([r for r in joints if r['phase'] >= 0], bodies, names, efficiency)
    mass = description_mass(Path(member['description']).read_text())
    result = dict(calibrate(samples, mass), member=member['id'], model=member['model'], joints=names,
                  efficiency=efficiency, completed_profile=completed,
                  source_description_sha256=member['source_description_sha256'],
                  scope='Simulated drivetrain only; hotel load, voltage and capacity stay authored')
    (output / 'calibration.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != 'held_out'}, indent=2))
    return 0 if completed else 1


if __name__ == '__main__':
    raise SystemExit(main())
