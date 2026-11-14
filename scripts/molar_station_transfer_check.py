#!/usr/bin/env python3
"""MOLAR payload transfer to a station in Gazebo (ROADMAP R11): lift, extend, belt, retract.

Launches ``molar_hri`` on ``nav_empty`` (display, headless), then through the
normal articulation interface (``/molar/articulation/*``, current plant ID,
10 Hz heartbeat):

1. spawns a 0.20 x 0.15 x 0.10 m, 0.5 kg box on the deck (link pose from Gazebo);
2. lifts the deck by ``--lift`` and spawns a static station table whose top
   is level with the measured deck, its near edge 0.40 m ahead of the base;
3. extends the telescope fully (both coupled stages, 0.28 m);
4. runs the roller at ``--roller`` rad/s; the deck belt (TrackController on
   telescope_stage2) carries the box off the extended deck end;
5. stops, retracts and lowers.

The box's truth pose decides the result: it ends on the table top (resting
height, beyond the deck end, inside the table) and stays there while the
deck retracts. All processes are owned and stopped by process group.

  python3 scripts/molar_station_transfer_check.py --out result.json
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path

BOX = (0.20, 0.15, 0.10)
TABLE = (0.80, 0.60)          # length along the robot's x, width
TABLE_NEAR_EDGE = 0.40        # ahead of base_link; footprint front is 0.375 m


def poses(world, env, names):
    """World poses {name: (x, y, z, qw, qx, qy, qz)} from one /world/<w>/pose/info message."""
    out = subprocess.run(['ign', 'topic', '-e', '-t', '/world/%s/pose/info' % world, '-n', '1'], env=env,
                         capture_output=True, text=True, timeout=15).stdout
    found = {}
    for block in re.split(r'\n(?=pose \{)', out):
        m = re.search(r'name: "([^"]+)"', block)
        if not m or m.group(1) not in names:
            continue
        def number(section, key):
            s = re.search(section + r' \{([^}]*)\}', block)
            v = re.search(r'\b%s: (\S+)' % key, s.group(1)) if s else None
            return float(v.group(1)) if v else (1. if key == 'w' else 0.)
        found[m.group(1)] = tuple([number('position', k) for k in 'xyz'] + [number('orientation', k) for k in 'wxyz'])
    return found


def spawn(world, env, name, sdf):
    request = 'sdf: %s' % json.dumps(sdf)
    result = subprocess.run(['ign', 'service', '-s', '/world/%s/create' % world, '--reqtype', 'ignition.msgs.EntityFactory',
                             '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req', request],
                            env=env, capture_output=True, text=True, timeout=10)
    if 'data: true' not in result.stdout:
        raise RuntimeError('spawn %s failed: %s %s' % (name, result.stdout, result.stderr))


def box_sdf(name, size, pose, mass=None):
    static = mass is None
    inertial = '' if static else (
        '<inertial><mass>%g</mass><inertia><ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz></inertia></inertial>'
        % (mass, mass * (size[1] ** 2 + size[2] ** 2) / 12, mass * (size[0] ** 2 + size[2] ** 2) / 12,
           mass * (size[0] ** 2 + size[1] ** 2) / 12))
    return ('<sdf version="1.9"><model name="%s"><static>%s</static><pose>%s</pose><link name="link">%s'
            '<collision name="c"><geometry><box><size>%g %g %g</size></box></geometry>'
            '<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>'
            '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry></visual></link></model></sdf>'
            % (name, 'true' if static else 'false', ' '.join('%.5f' % v for v in pose), inertial, *size, *size))


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--lift', type=float, default=0.10, help='Lift joint change before the transfer (m)')
    parser.add_argument('--roller', type=float, default=8.0)
    parser.add_argument('--out', default='')
    options = parser.parse_args()
    import rclpy
    from std_msgs.msg import Empty, String

    env = dict(os.environ)
    result = dict(steps=[], error='')
    log = open((options.out or 'molar_transfer') + '.launch.log', 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'simulator:=gazebo',
                               'robot_model:=molar_hri', 'map_name:=nav_empty', 'mode:=display', 'gui:=false',
                               'start_rviz:=false'], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('molar_station_transfer_check')
    state = {}
    node.create_subscription(String, '/molar/articulation/state', lambda m: state.update(json.loads(m.data)), 10)
    heartbeat = node.create_publisher(Empty, '/molar/articulation/heartbeat', 1)
    command = node.create_publisher(String, '/molar/articulation/command', 1)

    def spin(seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            heartbeat.publish(Empty())
            rclpy.spin_once(node, timeout_sec=.1)
            if until is not None and until():
                return True
        return until is None

    def channel(name):
        return next(c for c in state.get('channels', []) if c['name'] == name)

    def send(targets):
        command.publish(String(data=json.dumps(dict(plant_id=state['plant_id'], targets=targets))))

    def settled(name, value, tolerance=.003):
        return abs((channel(name)['measured'] or 0.) - value) < tolerance and not state.get('active')

    names = ('telescope_stage2', 'base_link', 'payload', 'station')
    try:
        if not spin(120, lambda: state.get('initialized') and not state.get('fault') and state.get('channels')):
            raise RuntimeError('articulation never became ready: %s' % state.get('status'))
        spin(3)
        world = 'nav_empty'
        p = poses(world, env, names)
        deck, base = p['telescope_stage2'], p['base_link']
        yaw = yaw_of(base[3:])
        result['deck_start'] = deck
        # 1. payload on the deck, 0.10 m behind the plate centre
        bx = deck[0] - .10 * math.cos(yaw)
        by = deck[1] - .10 * math.sin(yaw)
        spawn(world, env, 'payload', box_sdf('payload', BOX, (bx, by, deck[2] + BOX[2] / 2 + .005, 0, 0, yaw), mass=.5))
        spin(3)
        result['steps'].append(dict(step='payload on deck', pose=poses(world, env, names).get('payload')))
        # 2. lift, then the station level with the measured deck
        lift = channel('conveyor_lift_joint')
        target = max(lift['limits'][0], min(lift['limits'][1], lift['measured'] + options.lift))
        send({'conveyor_lift_joint': target})
        if not spin(30, lambda: settled('conveyor_lift_joint', target)):
            raise RuntimeError('lift did not settle: %s' % state.get('status'))
        p = poses(world, env, names)
        deck = p['telescope_stage2']
        top = deck[2]
        near = TABLE_NEAR_EDGE + TABLE[0] / 2
        sx, sy = base[0] + near * math.cos(yaw), base[1] + near * math.sin(yaw)
        spawn(world, env, 'station', box_sdf('station', (TABLE[0], TABLE[1], top), (sx, sy, top / 2, 0, 0, yaw)))
        spin(2)
        result['steps'].append(dict(step='lifted; station level with deck', deck_z=top, lift=channel('conveyor_lift_joint')['measured'],
                                    payload=p.get('payload')))
        # 3. extend both coupled stages
        stage = channel('telescope_stage1_joint')['limits'][1]
        send({'telescope_stage1_joint': stage})
        if not spin(30, lambda: settled('telescope_stage1_joint', stage) and settled('telescope_stage2_joint', stage)):
            raise RuntimeError('telescope did not extend: %s' % state.get('status'))
        p = poses(world, env, names)
        result['steps'].append(dict(step='extended', deck=p['telescope_stage2'], payload=p.get('payload')))
        # 4. belt until the payload is off the deck end (or 25 s)
        send({'transfer_roller_joint': options.roller})
        deck_end = math.hypot(p['telescope_stage2'][0] - base[0], p['telescope_stage2'][1] - base[1]) + .287
        def off_deck():
            q = poses(world, env, names).get('payload')
            return q is not None and math.hypot(q[0] - base[0], q[1] - base[1]) > deck_end + BOX[0] / 2 + .02
        moved = spin(25, off_deck)
        send({'transfer_roller_joint': 0.})
        spin(2)
        p = poses(world, env, names)
        result['steps'].append(dict(step='belt', off_deck=moved, payload=p.get('payload'), deck_end_from_base_m=deck_end,
                                    roller_measured=channel('transfer_roller_joint')['measured_velocity']))
        # 5. retract and lower; the payload must stay on the station
        send({'telescope_stage1_joint': 0.})
        spin(30, lambda: settled('telescope_stage1_joint', 0.) and settled('telescope_stage2_joint', 0.))
        send({'conveyor_lift_joint': lift['measured']})
        spin(30, lambda: settled('conveyor_lift_joint', lift['measured']))
        p = poses(world, env, names)
        q = p.get('payload')
        along = (q[0] - base[0]) * math.cos(yaw) + (q[1] - base[1]) * math.sin(yaw)
        across = -(q[0] - base[0]) * math.sin(yaw) + (q[1] - base[1]) * math.cos(yaw)
        on_station = (TABLE_NEAR_EDGE + BOX[0] / 2 - .02 <= along <= TABLE_NEAR_EDGE + TABLE[0]
                      and abs(across) <= TABLE[1] / 2 and abs(q[2] - (top + BOX[2] / 2)) < .02)
        result.update(final_payload=q, payload_along_m=along, payload_across_m=across, station_top_z=top,
                      on_station=on_station, passed=bool(moved and on_station))
        result['steps'].append(dict(step='retracted and lowered', status=state.get('status')))
    except Exception as exc:  # noqa: BLE001 - recorded
        result['error'] = str(exc)
        result['passed'] = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        log.close()
    print(json.dumps(result, indent=1))
    if options.out:
        Path(options.out).write_text(json.dumps(result, indent=1) + '\n')
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
