"""Cleaning core on synthetic maps, driven by a kinematic robot."""

import math
import time

import numpy as np
import pytest

from robot_lab_vacuum_cleaning import planner, rooms
from robot_lab_vacuum_cleaning.grid import Grid
from robot_lab_vacuum_cleaning.mission import Mission, PathFollower, RobotConfig, Sense
from robot_lab_vacuum_cleaning.store import Store, in_window, schedule_due

RES = 0.05


def two_rooms(width=8.0, height=4.0, door=(1.6, 2.4)):
    """Two 4x4 m rooms side by side, joined by a doorway in the middle wall."""
    w, h = int(width / RES), int(height / RES)
    data = np.zeros((h, w), dtype=np.int16)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    mid = w // 2
    data[:, mid] = 100
    data[int(door[0] / RES):int(door[1] / RES), mid] = 0
    return Grid(RES, (0.0, 0.0), data)


class World:
    """Unicycle robot in a world grid that may hold obstacles the map lacks."""

    def __init__(self, world: Grid, pose, radius=0.15, sense_range=1.5):
        self.world = world
        self.sense_range = sense_range
        self.pose = list(pose)
        self.radius = radius
        occ = world.occupied_mask()
        ys, xs = np.nonzero(occ)
        self.points = np.column_stack(((xs + 0.5) * RES + world.origin[0],
                                       (ys + 0.5) * RES + world.origin[1]))

    def sense(self) -> Sense:
        x, y, yaw = self.pose
        d = np.hypot(self.points[:, 0] - x, self.points[:, 1] - y)
        near = self.points[d < self.sense_range]
        dn = d[d < self.sense_range]
        front = math.inf
        if len(near):
            ang = np.arctan2(near[:, 1] - y, near[:, 0] - x) - yaw
            ang = np.arctan2(np.sin(ang), np.cos(ang))
            ahead = np.abs(ang) < math.radians(35)
            if ahead.any():
                front = float(dn[ahead].min()) - self.radius
        return Sense(front_clear=front, points=near, bumper=front < 0.005)

    def move(self, v, w, dt):
        x, y, yaw = self.pose
        nx, ny = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        ix, iy = self.world.world_to_cell(nx, ny)
        d = np.hypot(self.points[:, 0] - nx, self.points[:, 1] - ny).min()
        if d > self.radius - 0.01:
            x, y = nx, ny
        self.pose = [x, y, math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))]


def run(mission, world, seconds, dt=0.1, until=None):
    t = 0.0
    while t < seconds:
        v, w = mission.step(t, tuple(world.pose), world.sense())
        world.move(v, w, dt)
        t += dt
        if until is not None and until(mission):
            break
    return t


def make_mission(tmp_path, grid, pose, **settings):
    mission = Mission(RobotConfig(), Store(tmp_path), map_key='test')
    mission.settings.update(settings)
    mission.update_map(grid)
    mission.step(0.0, pose, Sense())
    return mission


def test_rooms_follow_the_doorway():
    labels = rooms.segment_rooms(two_rooms())
    assert labels.max() == 2
    left = labels[:, :70]
    right = labels[:, 90:]
    assert set(np.unique(left[left > 0])) != set(np.unique(right[right > 0]))


def test_split_and_merge_rooms():
    grid = two_rooms()
    labels = rooms.segment_rooms(grid)
    merged, keep = rooms.merge_rooms(labels, [1, 2])
    assert set(np.unique(merged[merged > 0])) == {keep}
    split, ids = rooms.split_room(grid, merged, keep, (2.0, -0.1), (2.0, 4.1))
    assert len(ids) == 2
    assert len(np.unique(split[split > 0])) == 2


def test_astar_goes_through_the_door_and_respects_a_virtual_wall():
    grid = two_rooms()
    pm = planner.PlanningMap(grid, 0.15)
    path = pm.astar((1.0, 1.0), (7.0, 1.0))
    assert path is not None
    assert any(1.6 <= y <= 2.4 for x, y in path if abs(x - 4.0) < 0.1)
    blocked = planner.PlanningMap(grid, 0.15, keepout=planner.Keepout(walls=[((4.0, 1.5), (4.0, 2.5))]))
    assert blocked.astar((1.0, 1.0), (7.0, 1.0)) is None


def test_lanes_cover_a_room_and_cross_pass_turns_them():
    grid = two_rooms()
    pm = planner.PlanningMap(grid, 0.15)
    region = pm.traversable & (np.arange(grid.width)[None, :] < 80)
    segs = planner.lane_segments(grid, region, 0.24, 'x', 0.13)
    covered = planner.sweep_mask(grid, segs, 0.13) & region
    assert covered.sum() / region.sum() > 0.99
    assert all(abs(p[1] - q[1]) < 1e-9 for p, q in segs)
    cross = planner.lane_segments(grid, region, 0.24, 'y', 0.13)
    assert all(abs(p[0] - q[0]) < 1e-9 for p, q in cross)


def test_edge_contours_follow_the_walls():
    grid = two_rooms()
    pm = planner.PlanningMap(grid, 0.15)
    region = pm.traversable & (np.arange(grid.width)[None, :] < 80)
    lines = planner.edge_contours(pm, region)
    assert lines
    pts = np.array([p for line in lines for p in line])
    # Every edge point is close to a wall, at the robot's safe distance.
    clear = [pm.obstacle_clearance_m[grid.world_to_cell(x, y)[1], grid.world_to_cell(x, y)[0]] for x, y in pts]
    assert max(clear) < 0.15 + 0.04 + 0.1


def test_follower_stops_for_an_obstacle_but_not_at_a_lane_end():
    f = PathFollower([(0, 0), (2, 0)], 0.25, 1.5)
    assert f.step((0.0, 0.0, 0.0), 0.01)[2] == 'blocked'
    g = PathFollower([(0, 0), (1, 0)], 0.25, 1.5)
    assert g.step((0.95, 0.0, 0.0), 0.01)[2] != 'blocked'


def test_whole_home_job_cleans_both_rooms_and_docks(tmp_path):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (1.0, 1.0, 0.0))
    ok, message = mission.command({'cmd': 'start', 'mode': 'all', 'edge': True})
    assert ok, message
    run(mission, world, 3000, until=lambda m: m.state in ('docked', 'charging', 'error', 'idle'))
    assert mission.state in ('docked', 'charging'), mission.events
    record = mission.last_record
    assert record['result'] == 'completed'
    assert record['coverage_pct'] > 90.0
    assert math.dist(world.pose[:2], (1.0, 1.0)) < 0.1
    history = Store(tmp_path).load_history()
    assert history[-1]['id'] == record['id']
    assert Store(tmp_path).load_history_coverage(record['id']).shape == grid.data.shape


def test_selected_room_with_a_hidden_obstacle(tmp_path):
    grid = two_rooms()
    world_grid = Grid(RES, grid.origin, grid.data.copy())
    world_grid.data[30:42, 120:132] = 100   # a box the map does not know
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(world_grid, (1.0, 1.0, 0.0))
    mission.ensure_rooms()
    right = int(mission.room_labels[40, 140])
    ok, _ = mission.command({'cmd': 'start', 'mode': 'rooms', 'rooms': [right], 'edge': False})
    assert ok
    run(mission, world, 3000, until=lambda m: m.state in ('docked', 'charging', 'error', 'idle'))
    assert mission.last_record['result'] == 'completed', list(mission.events)[-5:]
    assert mission.last_record['events'] >= 1
    assert mission.last_record['coverage_pct'] > 80.0
    # Nothing in the other room was cleaned beyond the transit corridor.
    left = mission.coverage[:, :70]
    assert left.mean() < 0.15


def test_low_battery_charges_and_resumes(tmp_path):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0), battery_time_scale=30.0,
                           resume_battery_pct=60.0)
    world = World(grid, (1.0, 1.0, 0.0))
    mission.command({'cmd': 'start', 'mode': 'all', 'edge': False})
    mission.battery = 30.0
    seen = set()

    def watch(m):
        seen.add(m.state)
        return m.job is None and m.state in ('docked', 'charging')
    run(mission, world, 8000, until=watch)
    assert {'returning', 'charging', 'cleaning'} <= seen
    assert mission.last_record['recharges'] >= 1
    assert mission.last_record['result'] == 'completed'


def test_pause_resume_and_stop(tmp_path):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (1.0, 1.0, 0.0))
    mission.command({'cmd': 'start', 'mode': 'spot', 'point': [2.0, 2.0]})
    run(mission, world, 20)
    assert mission.command({'cmd': 'pause'})[0]
    assert mission.step(100.0, tuple(world.pose), world.sense()) == (0.0, 0.0)
    left = len(mission.tasks)
    assert mission.command({'cmd': 'resume'})[0]
    assert mission.state == 'cleaning' and len(mission.tasks) >= left
    assert mission.command({'cmd': 'stop'})[0]
    assert mission.state == 'idle' and mission.last_record['result'] == 'stopped'


def test_no_go_zone_is_never_cleaned(tmp_path):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (1.0, 1.0, 0.0))
    zone = [[2.5, 2.5], [3.8, 2.5], [3.8, 3.8], [2.5, 3.8]]
    mission.command({'cmd': 'set_layout', 'no_go': [{'id': 'a', 'polygon': zone}]})
    mission.command({'cmd': 'start', 'mode': 'zones', 'zones': [[0.2, 0.2, 3.8, 3.8]]})
    trail = []
    run(mission, world, 2000, until=lambda m: trail.append(tuple(world.pose)) or m.job is None)
    assert mission.last_record['result'] == 'completed'
    assert not any(2.5 < x < 3.8 and 2.5 < y < 3.8 for x, y, _ in trail)


def test_mapping_run_explores_unknown_space(tmp_path):
    truth = two_rooms()
    known = Grid(RES, truth.origin, np.full(truth.data.shape, -1, dtype=np.int16))
    mission = make_mission(tmp_path, known, (1.0, 1.0, 0.0))
    world = World(truth, (1.0, 1.0, 0.0))

    def reveal():
        x, y, _ = world.pose
        ix, iy = truth.world_to_cell(x, y)
        r = int(2.0 / RES)
        ys, xs = np.ogrid[:truth.height, :truth.width]
        disc = (xs - ix) ** 2 + (ys - iy) ** 2 <= r * r
        # Line of sight is ignored; the doorway wall still limits the reveal
        # because the far room only shows once the robot is near the door.
        known.data[disc] = truth.data[disc]
        mission.update_map(known)
    reveal()
    assert mission.command({'cmd': 'start', 'mode': 'explore'})[0]
    t = 0.0
    while t < 1500 and mission.job is not None:
        v, w = mission.step(t, tuple(world.pose), world.sense())
        world.move(v, w, 0.1)
        t += 0.1
        if int(t * 10) % 20 == 0:
            reveal()
    assert (known.data >= 0).mean() > 0.95
    assert mission.room_labels is not None and mission.room_labels.max() >= 2


def test_store_round_trip_and_schedule(tmp_path):
    store = Store(tmp_path)
    store.save_schedule([{'id': 'a', 'days': [0], 'time': '09:30'}])
    assert store.load_schedule()[0]['time'] == '09:30'
    monday = time.strptime('2026-11-09 09:30', '%Y-%m-%d %H:%M')
    assert schedule_due({'days': [0], 'time': '09:30'}, monday)
    assert not schedule_due({'days': [1], 'time': '09:30'}, monday)
    assert in_window('22:00', '08:00', time.strptime('2026-11-09 23:10', '%Y-%m-%d %H:%M'))
    assert not in_window('22:00', '08:00', time.strptime('2026-11-09 12:00', '%Y-%m-%d %H:%M'))


def test_layout_persists_room_names(tmp_path):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    mission.ensure_rooms()
    mission.command({'cmd': 'rename_room', 'room': 1, 'name': 'Kitchen'})
    again = Mission(RobotConfig(), Store(tmp_path), map_key='test')
    again.update_map(grid)
    assert again.layout['rooms'][1]['name'] == 'Kitchen'
    assert again.room_labels is not None


@pytest.mark.parametrize('suction', ['eco', 'max'])
def test_consumables_and_dustbin_accumulate(tmp_path, suction):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (1.0, 1.0, 0.0))
    mission.command({'cmd': 'start', 'mode': 'spot', 'point': [2.0, 2.0], 'suction': suction})
    run(mission, world, 60)
    status = mission.status()
    assert status['dustbin_pct'] > 0.0
    assert status['consumables']['filter']['used_h'] > 0.0
    assert mission.command({'cmd': 'reset_consumable', 'name': 'filter'})[0]
    assert mission.status()['consumables']['filter']['used_h'] == 0.0


@pytest.mark.parametrize('point,box', [((2.0, 2.0), False), ((2.0, 2.0), True)])
def test_spot_covers_its_square_on_open_floor_and_beside_furniture(tmp_path, point, box):
    grid = two_rooms()
    if box:
        grid.data[38:42, 44:54] = 100   # a bar 0.5 m right of the spot centre
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (1.0, 1.0, 0.0))
    assert mission.command({'cmd': 'start', 'mode': 'spot', 'point': list(point)})[0]
    run(mission, world, 1500, until=lambda m: m.job is None)
    assert mission.last_record['result'] == 'completed'
    assert mission.last_record['coverage_pct'] > 85.0


def test_follower_does_not_turn_back_to_a_path_start_behind_it():
    f = PathFollower([(0.0, 0.2), (0.0, -0.2)], 0.25, 1.6, lookahead=0.15)
    pose = [0.03, 0.08, -0.2]
    for _ in range(80):
        v, w, status = f.step(tuple(pose), 5.0)
        if status == 'done':
            break
        pose = [pose[0] + v * math.cos(pose[2]) * 0.1, pose[1] + v * math.sin(pose[2]) * 0.1, pose[2] + w * 0.1]
    assert status == 'done'


def _room_scan(pose, size=(4.0, 3.0)):
    """Walls of a size[0] x size[1] room (origin corner) seen from pose, robot frame."""
    xs = np.linspace(0, size[0], 81)
    ys = np.linspace(0, size[1], 61)
    pts = np.concatenate([np.column_stack((xs, np.zeros_like(xs))), np.column_stack((xs, np.full_like(xs, size[1]))),
                          np.column_stack((np.zeros_like(ys), ys)), np.column_stack((np.full_like(ys, size[0]), ys))])
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def test_stall_detector_flags_forward_motion_without_progress():
    from robot_lab_vacuum_cleaning.mission import StallDetector
    moving, stuck, turning = StallDetector(), StallDetector(), StallDetector()
    hits_moving = hits_stuck = hits_turning = 0
    for k in range(40):
        t = k * 0.1
        est = (1.0 + 0.02 * k, 1.0 + 0.005 * k, 0.1)
        hits_moving += moving.update(t, 0.2, _room_scan(est), est)            # scan follows the estimate
        hits_stuck += stuck.update(t, 0.2, _room_scan((1.0, 1.0, 0.1)), est)  # truth stays put
        hits_turning += turning.update(t, 0.0, _room_scan((1.0, 1.0, 0.1 * k)), (1.0, 1.0, 0.1 * k))
    assert hits_moving == 0 and hits_stuck >= 1 and hits_turning == 0


def _base_points(world, pose):
    sense = world.sense()
    if sense.points is None or not len(sense.points):
        return None
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = sense.points[:, 0] - x, sense.points[:, 1] - y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def test_stall_detector_stays_quiet_on_a_normal_job(tmp_path):
    from robot_lab_vacuum_cleaning.mission import StallDetector
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (1.0, 1.0, 0.0), sense_range=8.0)
    detector = StallDetector(max_range=8.0)
    mission.command({'cmd': 'start', 'mode': 'all', 'edge': True})
    t, v, hits = 0.0, 0.0, 0
    while t < 3000 and mission.job is not None:
        hits += detector.update(t, v, _base_points(world, world.pose), tuple(world.pose))
        v, w = mission.step(t, tuple(world.pose), world.sense())
        world.move(v, w, 0.1)
        t += 0.1
    assert mission.last_record['result'] == 'completed' and hits == 0


def test_low_obstacle_below_the_lidar_is_found_by_stalling(tmp_path):
    from robot_lab_vacuum_cleaning.mission import StallDetector
    grid = two_rooms()
    low = Grid(RES, grid.origin, grid.data.copy())
    low.data[30:42, 100:112] = 100          # a 0.6 m box under the scan plane
    mission = make_mission(tmp_path, grid, (5.0, 1.0, 0.0))
    seen = World(grid, (5.0, 1.0, 0.0), sense_range=8.0)   # what the LiDAR shows: the map only
    body = World(low, (5.0, 1.0, 0.0))                     # what the body collides with
    detector = StallDetector(max_range=8.0)
    assert mission.command({'cmd': 'start', 'mode': 'zones', 'zones': [[4.2, 0.2, 7.8, 3.8]], 'edge': False})[0]
    t, v_last, odom = 0.0, 0.0, list(body.pose)
    while t < 3000 and mission.job is not None:
        seen.pose = list(body.pose)
        sense = seen.sense()
        sense.stall = detector.update(t, v_last, _base_points(seen, body.pose), tuple(odom))
        # The mission (like AMCL following odometry) sees the odometry pose.
        v, w = mission.step(t, tuple(odom), sense)
        before = list(body.pose)
        body.move(v, w, 0.1)
        if body.pose[:2] == before[:2] and abs(v) > 1e-6:
            # Blocked: the wheels turn, odometry integrates the command.
            odom = [odom[0] + v * math.cos(odom[2]) * 0.1, odom[1] + v * math.sin(odom[2]) * 0.1, body.pose[2]]
        else:
            odom = [odom[0] + body.pose[0] - before[0], odom[1] + body.pose[1] - before[1], body.pose[2]]
        v_last, t = v, t + 0.1
    assert mission.last_record['result'] == 'completed'
    assert any('Stall' in e['text'] for e in mission.events)


def test_blocked_way_home_backs_off_then_replans(tmp_path):
    grid = two_rooms()
    mission = make_mission(tmp_path, grid, (1.0, 1.0, 0.0))
    world = World(grid, (6.0, 2.0, math.pi))
    mission.layout['dock'] = [1.0, 1.0, 0.0]
    mission.step(0.0, tuple(world.pose), world.sense())
    assert mission.command({'cmd': 'dock'})[0] and mission.state == 'returning'
    blocked = Sense(front_clear=0.0, points=np.array([[5.8, 2.0]]))
    mission.step(0.1, tuple(world.pose), blocked)
    assert mission.maneuvers and mission.maneuvers[0][0] < 0   # the back-off survives
    run(mission, world, 600, until=lambda m: m.state in ('docked', 'charging', 'error'))
    assert mission.state in ('docked', 'charging')
