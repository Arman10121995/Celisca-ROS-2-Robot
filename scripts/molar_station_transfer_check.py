#!/usr/bin/env python3
"""MOLAR payload transfer to a station in Gazebo, MuJoCo or PyBullet (ROADMAP R11): lift, extend, belt, retract.

Launches ``molar_hri`` on ``nav_empty`` (display, headless), then through the
normal articulation interface (``/molar/articulation/*``, current plant ID,
10 Hz heartbeat):

1. spawns a 0.20 x 0.15 x 0.10 m, 0.5 kg box on the deck (link pose from Gazebo);
2. lifts the deck by ``--lift``;
3. extends the telescope fully (both coupled stages, 0.28 m) and spawns a
   static station table end to end with the measured deck: near edge 1 cm
   past the deck end, top 5 mm below the deck;
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
DECK_HALF_LENGTH = 0.287      # telescope_stage2 plate collision, 0.574 m long
STATION_GAP = 0.01            # station near edge past the extended deck end
STATION_DROP = 0.005          # station top below the deck
# The station is a passive roller table: low friction (static boxes with
# 0.8 held the payload once most of it was across, 2026-11-15). Gazebo DART
# combines friction by minimum; MuJoCo by maximum, so there the station geom
# also gets a contact priority.
STATION_MU = 0.05


def poses(world, env, names):
    """Entity poses {name: (x, y, z, qw, qx, qy, qz)} from one pose/info message (links relative to their model)."""
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


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def in_world(model, link):
    """World pose of a link whose pose/info entry is relative to its model."""
    q = model[3:]
    v = _qmul(_qmul(q, (0.0,) + tuple(link[:3])), (q[0], -q[1], -q[2], -q[3]))[1:]
    return tuple(m + d for m, d in zip(model[:3], v)) + _qmul(q, link[3:])


def world_poses(world, env, model, links, models=()):
    """World poses of the robot's links (pose/info gives them relative to the model) and of other models."""
    found = poses(world, env, (model,) + tuple(links) + tuple(models))
    base = found.get(model)
    result = {name: found[name] for name in models if name in found}
    if base is not None:
        result.update({name: in_world(base, found[name]) for name in links if name in found})
    return result


def spawn(world, env, name, sdf):
    request = 'sdf: %s' % json.dumps(sdf)
    result = subprocess.run(['ign', 'service', '-s', '/world/%s/create' % world, '--reqtype', 'ignition.msgs.EntityFactory',
                             '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000', '--req', request],
                            env=env, capture_output=True, text=True, timeout=10)
    if 'data: true' not in result.stdout:
        raise RuntimeError('spawn %s failed: %s %s' % (name, result.stdout, result.stderr))


def box_sdf(name, size, pose, mass=None, mu=0.8):
    static = mass is None
    inertial = '' if static else (
        '<inertial><mass>%g</mass><inertia><ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz></inertia></inertial>'
        % (mass, mass * (size[1] ** 2 + size[2] ** 2) / 12, mass * (size[0] ** 2 + size[2] ** 2) / 12,
           mass * (size[0] ** 2 + size[1] ** 2) / 12))
    return ('<sdf version="1.9"><model name="%s"><static>%s</static><pose>%s</pose><link name="link">%s'
            '<collision name="c"><geometry><box><size>%g %g %g</size></box></geometry>'
            '<surface><friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction></surface></collision>'
            '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry></visual></link></model></sdf>'
            % (name, 'true' if static else 'false', ' '.join('%.5f' % v for v in pose), inertial, *size, mu, mu, *size))


# MuJoCo cannot add bodies to a running model, so the payload and the station
# are written into a copy of the nav_empty MJCF before launch, placed with the
# base-relative deck geometry measured in Gazebo (2026-11-15): deck top 0.749 m
# above base_link at zero lift, x offset 0.0005 m, 0.5675 m deck end when
# fully extended. The spawn pose is the map's (sim_maps.yaml nav_empty).
SPAWN = (-0.5, -0.5, 0.0)
SPAWN_Z = 0.1                 # the map's spawn drop: robot and payload fall it together
DECK_REL_X, DECK_TOP_Z, EXTENDED_DECK_END = 0.0005, 0.749, 0.5675


def pybullet_world(directory, lift):
    """nav_empty SDF with the payload (loose) and the station (static, low friction), placed as for MuJoCo."""
    from ament_index_python.packages import get_package_share_directory
    maps = Path(get_package_share_directory('robot_lab_maps'))
    directory.mkdir(parents=True, exist_ok=True)
    x, y, yaw = SPAWN
    c, s_ = math.cos(yaw), math.sin(yaw)
    px = DECK_REL_X - .10
    top = DECK_TOP_Z + lift - STATION_DROP
    near = EXTENDED_DECK_END + STATION_GAP
    centre = near + TABLE[0] / 2
    payload = box_sdf('payload', BOX, (x + px * c, y + px * s_, SPAWN_Z + DECK_TOP_Z + BOX[2] / 2 + .002, 0, 0, yaw), mass=.5)
    station = box_sdf('station', (TABLE[0], TABLE[1], top), (x + centre * c, y + centre * s_, top / 2, 0, 0, yaw), mu=STATION_MU)
    def model(sdf):
        return sdf[sdf.index('<model'):sdf.index('</sdf>')]
    world = directory / 'nav_empty_transfer.world'
    text = (maps / 'maps/nav_empty/worlds/nav_empty.world').read_text()
    world.write_text(text.replace('</world>', model(payload) + model(station) + '</world>', 1))
    return world, dict(near_edge=near, top=top, deck_end=EXTENDED_DECK_END)


def gazebo_world_name(map_name):
    """The Gazebo world name of a sim_maps.yaml map (its pose/info and create services use it)."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    maps = yaml.safe_load((Path(get_package_share_directory('robot_lab_bringup')) / 'config' / 'sim_maps.yaml').read_text())
    entry = (maps.get('maps', maps)).get(map_name)
    if entry is None:
        raise ValueError('unknown map %s' % map_name)
    return (entry.get('gazebo') or {}).get('world_name', map_name)


def mujoco_world(directory, lift):
    """nav_empty with the payload on the retracted deck and the station at the extended deck end."""
    from ament_index_python.packages import get_package_share_directory
    maps = Path(get_package_share_directory('robot_lab_maps'))
    (directory / 'worlds').mkdir(parents=True, exist_ok=True)
    (directory / 'mjcf').mkdir(parents=True, exist_ok=True)
    world = directory / 'worlds' / 'nav_empty.world'
    world.write_text((maps / 'maps/nav_empty/worlds/nav_empty.world').read_text())
    x, y, yaw = SPAWN
    c, s_ = math.cos(yaw), math.sin(yaw)
    px = DECK_REL_X - .10
    top = DECK_TOP_Z + lift - STATION_DROP
    near = EXTENDED_DECK_END + STATION_GAP
    centre = near + TABLE[0] / 2
    m = .5
    inertia = (m * (BOX[1] ** 2 + BOX[2] ** 2) / 12, m * (BOX[0] ** 2 + BOX[2] ** 2) / 12, m * (BOX[0] ** 2 + BOX[1] ** 2) / 12)
    bodies = ('<body name="payload" pos="%g %g %g" euler="0 0 %g"><freejoint/>'
              '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>'
              '<geom type="box" size="%g %g %g" friction="0.8" rgba="0.8 0.5 0.2 1"/></body>'
              '<geom name="station" type="box" pos="%g %g %g" euler="0 0 %g" size="%g %g %g" friction="%g" '
              'priority="1" rgba="0.5 0.5 0.55 1"/>') % (
        x + px * c, y + px * s_, SPAWN_Z + DECK_TOP_Z + BOX[2] / 2 + .002, yaw, m, *inertia, BOX[0] / 2, BOX[1] / 2, BOX[2] / 2,
        x + centre * c, y + centre * s_, top / 2, yaw, TABLE[0] / 2, TABLE[1] / 2, top / 2, STATION_MU)
    xml = (maps / 'mjcf/nav_empty.xml').read_text()
    (directory / 'mjcf' / 'nav_empty.xml').write_text(xml.replace('</worldbody>', bodies + '\n  </worldbody>', 1))
    return world, dict(near_edge=near, top=top, deck_end=EXTENDED_DECK_END)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--lift', type=float, default=0.10, help='Lift joint change before the transfer (m)')
    parser.add_argument('--roller', type=float, default=8.0)
    parser.add_argument('--out', default='')
    parser.add_argument('--simulator', choices=('gazebo', 'mujoco', 'pybullet'), default='gazebo')
    parser.add_argument('--time-scale', type=float, default=1.0,
                        help='Multiply the settle/belt waits (large worlds run Gazebo below real time)')
    parser.add_argument('--map', default='nav_empty',
                        help='Gazebo only for maps other than nav_empty: the station is spawned ahead of the '
                             "map's spawn pose, which needs ~1.4 x 0.6 m of free floor")
    options = parser.parse_args()
    if options.map != 'nav_empty' and options.simulator != 'gazebo':
        parser.error('--map other than nav_empty is Gazebo only (MuJoCo/PyBullet worlds are pre-written for nav_empty)')
    import rclpy
    from std_msgs.msg import Empty, String

    # One Gazebo transport partition shared by the launch and the ign CLI calls.
    env = dict(os.environ, IGN_PARTITION='molar_transfer_%d' % os.getpid(), GZ_PARTITION='molar_transfer_%d' % os.getpid())
    result = dict(steps=[], error='', map=options.map, simulator=options.simulator)
    log = open((options.out or 'molar_transfer') + '.launch.log', 'w')
    # MuJoCo and PyBullet: payload and station are in the world file before
    # launch; truth comes from /simulation/free_bodies and odometry.
    mujoco = options.simulator in ('mujoco', 'pybullet')
    extra, placed = [], None
    if options.simulator == 'mujoco':
        world_file, placed = mujoco_world(Path((options.out or 'molar_transfer') + '_world'), options.lift)
        extra = ['mujoco_world_xml:=%s' % (world_file.parent.parent / 'mjcf' / 'nav_empty.xml')]
        result['mujoco_world'] = dict(placed, world=str(world_file))
    elif options.simulator == 'pybullet':
        world_file, placed = pybullet_world(Path((options.out or 'molar_transfer') + '_world'), options.lift)
        extra = ['world_path:=%s' % world_file]
        result['pybullet_world'] = dict(placed, world=str(world_file))
    launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py',
                               'simulator:=' + options.simulator, 'robot_model:=molar_hri', 'map_name:=' + options.map,
                               'mode:=display', 'gui:=false', 'start_rviz:=false'] + extra,
                              env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('molar_station_transfer_check')
    truth = {}
    if mujoco:
        from nav_msgs.msg import Odometry
        node.create_subscription(String, '/simulation/free_bodies', lambda m: truth.update(json.loads(m.data)), 10)
        def on_odom(m):
            q, o = m.pose.pose.position, m.pose.pose.orientation
            truth['base_link'] = [q.x, q.y, q.z, o.w, o.x, o.y, o.z]
            v, w = m.twist.twist.linear, m.twist.twist.angular
            speed = (math.hypot(v.x, v.y), abs(w.z))
            truth['base_speed'] = speed
            peak = truth.setdefault('base_speed_peak', [0., 0.])
            peak[0], peak[1] = max(peak[0], speed[0]), max(peak[1], speed[1])
        node.create_subscription(Odometry, '/odom/ground_truth', on_odom, 10)
    state = {}
    node.create_subscription(String, '/molar/articulation/state', lambda m: state.update(json.loads(m.data)), 10)
    heartbeat = node.create_publisher(Empty, '/molar/articulation/heartbeat', 1)
    command = node.create_publisher(String, '/molar/articulation/command', 1)
    from std_msgs.msg import Float64
    belt = dict(count=0, last=None, max=0.)
    def on_belt(message):
        belt.update(count=belt['count'] + 1, last=message.data, max=max(belt['max'], abs(message.data)))
    node.create_subscription(Float64, '/molar/belt_cmd_vel', on_belt, 10)

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

    def settled(name, value=None, tolerance=.003):
        """Measured at *value* (default: the channel's own target, e.g. a coupled follower's)."""
        c = channel(name)
        goal = c['target'] if value is None else value
        return goal is not None and abs((c['measured'] or 0.) - goal) < tolerance and not state.get('active')

    def channels():
        return {c['name']: dict(measured=c['measured'], target=c['target'], limits=c['limits'])
                for c in state.get('channels', [])}

    links, models = ('telescope_stage2', 'base_link'), ('payload', 'station')

    def observe():
        """Truth poses: Gazebo pose/info, or MuJoCo free bodies + base odometry (deck from the measured joints)."""
        if not mujoco:
            return world_poses(world, env, 'molar_hri', links, models)
        spin(0.3)
        base = truth.get('base_link')
        if base is None:
            return {}
        measured = {c['name']: c['measured'] or 0. for c in state.get('channels', [])}
        rel = (DECK_REL_X + measured.get('telescope_stage1_joint', 0.) + measured.get('telescope_stage2_joint', 0.), 0.,
               DECK_TOP_Z + measured.get('conveyor_lift_joint', 0.), 1., 0., 0., 0.)
        out = dict(base_link=tuple(base), telescope_stage2=in_world(tuple(base), rel))
        if 'payload' in truth:
            out['payload'] = tuple(truth['payload'])
        return out
    world = gazebo_world_name(options.map)
    try:
        if not spin(120 * options.time_scale, lambda: state.get('initialized') and not state.get('fault') and state.get('channels')):
            raise RuntimeError('articulation never became ready: %s' % state.get('status'))
        spin(3 * options.time_scale)
        p = observe()
        deck, base = p['telescope_stage2'], p['base_link']
        yaw = yaw_of(base[3:])
        result['deck_start'] = deck
        # 1. payload on the deck, 0.10 m behind the plate centre
        bx = deck[0] - .10 * math.cos(yaw)
        by = deck[1] - .10 * math.sin(yaw)
        if not mujoco:
            spawn(world, env, 'payload', box_sdf('payload', BOX, (bx, by, deck[2] + BOX[2] / 2 + .005, 0, 0, yaw), mass=.5))
        spin(3 * options.time_scale)
        result['steps'].append(dict(step='payload on deck', pose=observe().get('payload')))
        # 2. lift
        lift = channel('conveyor_lift_joint')
        target = max(lift['limits'][0], min(lift['limits'][1], lift['measured'] + options.lift))
        send({'conveyor_lift_joint': target})
        if not spin(30 * options.time_scale, lambda: settled('conveyor_lift_joint', target)):
            raise RuntimeError('lift did not settle: %s' % state.get('status'))
        p = observe()
        result['steps'].append(dict(step='lifted', deck_z=p['telescope_stage2'][2],
                                    lift=channel('conveyor_lift_joint')['measured'], payload=p.get('payload')))
        # 3. extend both coupled stages
        stage = channel('telescope_stage1_joint')['limits'][1]
        send({'telescope_stage1_joint': stage})
        if not spin(30 * options.time_scale, lambda: settled('telescope_stage1_joint', stage) and settled('telescope_stage2_joint')):
            raise RuntimeError('telescope did not extend: %s; %s' % (state.get('status'), json.dumps(channels())))
        # The station meets the extended deck end to end, as a conveyor
        # handover: near edge STATION_GAP past the measured deck end, top
        # STATION_DROP below the measured deck.
        p = observe()
        deck = p['telescope_stage2']
        deck_end = ((deck[0] - base[0]) * math.cos(yaw) + (deck[1] - base[1]) * math.sin(yaw)) + DECK_HALF_LENGTH
        near_edge = deck_end + STATION_GAP
        top = deck[2] - STATION_DROP
        centre = near_edge + TABLE[0] / 2
        if mujoco:
            # Pre-placed before launch; judge against where it actually is.
            near_edge, top = placed['near_edge'], placed['top']
        else:
            spawn(world, env, 'station', box_sdf('station', (TABLE[0], TABLE[1], top),
                                                 (base[0] + centre * math.cos(yaw), base[1] + centre * math.sin(yaw),
                                                  top / 2, 0, 0, yaw), mu=STATION_MU))
        spin(2 * options.time_scale)
        result['steps'].append(dict(step='extended; station at the deck end', deck=deck, payload=p.get('payload'),
                                    deck_end_from_base_m=deck_end, station_near_edge_m=near_edge, station_top_z=top,
                                    channels=channels()))
        # 4. belt until the payload is off the deck end (or 25 s)
        truth['base_speed_peak'] = [0., 0.]
        send({'transfer_roller_joint': options.roller})
        spin(1.0 * options.time_scale)
        result['roller_command'] = dict(status=state.get('status'), fault=state.get('fault'),
                                        target=channel('transfer_roller_joint')['target'],
                                        base_inhibited=state.get('base_inhibited'),
                                        base_speed_peak=truth.get('base_speed_peak'))
        def off_deck():
            q = observe().get('payload')
            return q is not None and ((q[0] - base[0]) * math.cos(yaw) + (q[1] - base[1]) * math.sin(yaw)
                                      > near_edge + BOX[0] / 2 + .02)
        # Until the payload is entirely on the station, or has stopped moving
        # for 2 s (a static station has no rollers of its own to pull it on).
        trace = []
        def along_of(q):
            return (q[0] - base[0]) * math.cos(yaw) + (q[1] - base[1]) * math.sin(yaw)
        def transferred():
            q = observe().get('payload')
            trace.append(dict(t=time.monotonic(), belt=belt['last'], payload=q[:3] if q else None))
            if q is None:
                return False
            if along_of(q) - BOX[0] / 2 > deck_end:
                return True
            old = [e for e in trace if e['t'] < trace[-1]['t'] - 2.0 * options.time_scale and e['payload']]
            return bool(old) and along_of(q) > deck_end and abs(along_of(q) - along_of(old[-1]['payload'])) < .002
        moved = spin(25 * options.time_scale, transferred)
        p = observe()
        roller = channel('transfer_roller_joint')['measured_velocity']
        send({'transfer_roller_joint': 0.})
        spin(2 * options.time_scale)
        before_retract = p.get('payload')
        result['steps'].append(dict(step='belt', transferred=moved, payload=before_retract, roller_measured=roller,
                                    rear_overlap_m=round(deck_end - (along_of(before_retract) - BOX[0] / 2), 4),
                                    belt_cmd=dict(belt), trace=[dict(e, t=round(e['t'] - trace[0]['t'], 2))
                                                                for e in trace[::3]]))
        # 5. retract and lower; the payload must stay on the station
        send({'telescope_stage1_joint': 0.})
        spin(30 * options.time_scale, lambda: settled('telescope_stage1_joint', 0.) and settled('telescope_stage2_joint'))
        send({'conveyor_lift_joint': lift['measured']})
        spin(30 * options.time_scale, lambda: settled('conveyor_lift_joint', lift['measured']))
        p = observe()
        q = p.get('payload')
        along = (q[0] - base[0]) * math.cos(yaw) + (q[1] - base[1]) * math.sin(yaw)
        across = -(q[0] - base[0]) * math.sin(yaw) + (q[1] - base[1]) * math.cos(yaw)
        on_station = (near_edge + BOX[0] / 2 - .02 <= along <= near_edge + TABLE[0]
                      and abs(across) <= TABLE[1] / 2 and abs(q[2] - (top + BOX[2] / 2)) < .02)
        drag = math.hypot(q[0] - before_retract[0], q[1] - before_retract[1])
        # Passed: after the deck retracts and lowers, the payload rests on the
        # station (height, footprint), its centre past the deck end, and the
        # retraction dragged it less than 1 cm.
        result.update(final_payload=q, payload_along_m=along, payload_across_m=across, station_top_z=top,
                      on_station=on_station, retract_drag_m=round(drag, 4),
                      passed=bool(moved and on_station and along > deck_end and drag < .01))
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
