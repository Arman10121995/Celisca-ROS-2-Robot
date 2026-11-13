"""Bounded RTAB-Map/depth ingestion, separate from the FCU setpoint loop."""
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

from robot_lab_utils.observed_aerial_scene import VoxelObstacles, depth_points, transform_points
from robot_lab_utils.aerial_tracking import SweptMemory, Tracker, cluster_points, remove_swept, time_conflict

# Route speed of px4_ros_controller.advance_route (m/s): the time base for moving-obstacle conflicts.
ROUTE_SPEED = .4
# RTAB-Map republishes its obstacle cloud only when it adds a node, so a hovering or slow drone can see
# it unchanged for well over 10 s (2026-11-13). Mapped obstacles are static; live depth (0.75 s) guards
# the lookahead, so the mapped layer may be up to this old.
MAP_MAX_AGE_S = 60.
# The mapped layer keeps people where they walked (no new RTAB-Map node while hovering); mapped points
# in columns swept by moving tracks are dropped, re-filtering the latest map at most this often.
REFILTER_S = 1.
# Floor removal before tracking: the floor is estimated per cell from the depth within this much of the
# expected ground height.
FLOOR_CELL_M, FLOOR_SEARCH_M = 1., .6


class AerialObstacles:
    def __init__(self, node):
        self.node = node
        self.buffer = Buffer(cache_time=Duration(seconds=20.))
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        self.map = self.depth = None
        self.errors = {'map': 'Waiting for actual RTAB-Map obstacles', 'depth': 'Waiting for registered depth'}
        self.pending, self.sequence, self.last_stamp = {}, 0, {}
        self.applied_sequence = {'map': 0, 'depth': 0}
        self.lock, self.wake, self.closed = threading.Lock(), threading.Event(), threading.Event()
        self.results = queue.Queue(maxsize=4)
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2, '/rtabmap/cloud_obstacles', lambda m:self.enqueue('map', m), cloud_qos)
        node.create_subscription(Image, '/x500/camera/depth/image_raw', lambda m:self.enqueue('depth', m), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, '/x500/camera/depth/camera_info', self.info, qos_profile_sensor_data)
        self.intrinsic = None
        self.tracker = Tracker()
        self.tracked_stamp = None
        self.swept = SweptMemory()
        self.swept_snapshot = (0, [])           # shared with the worker under self.lock
        self.raw_map, self.last_refilter = None, 0.   # worker only
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def stamp(self, message, max_age):
        value = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        age = self.node.get_clock().now().nanoseconds*1e-9-value
        if not -.15 <= age <= max_age or not message.header.frame_id:
            raise ValueError('Stale/future/missing-frame observation rejected')
        return value

    def info(self, message):
        try:
            self.stamp(message, 2.)
            if message.header.frame_id != 'camera_optical_frame' or not 0 < message.width*message.height <= 300000:
                raise ValueError('Expected bounded native optical CameraInfo')
            if any(abs(value) > 1e-8 for value in message.d):
                raise ValueError('Distorted depth needs rectification before flight ingestion')
            self.intrinsic = (message, time.monotonic())
        except (ValueError, TypeError) as exc:
            self.intrinsic = None
            self.errors['depth'] = str(exc)

    def enqueue(self, kind, message):
        with self.lock:
            self.sequence += 1
            sequence = self.sequence
        try:
            timestamp = self.stamp(message, MAP_MAX_AGE_S if kind == 'map' else .75)
            if timestamp <= self.last_stamp.get(kind, -float('inf')):
                return  # Replayed/repeated stamps do not renew observation health.
            if kind == 'map':
                if message.header.frame_id not in ('aerial_map', 'map') or len(message.data) > 16*1024**2 or message.width*message.height > 300000:
                    raise ValueError('Unexpected/oversized mapped obstacle cloud')
                if message.width*message.height and (not 0 < message.point_step <= 256
                        or message.row_step != message.width*message.point_step
                        or len(message.data) != message.row_step*message.height or len(message.fields) > 32):
                    raise ValueError('Incomplete/padded/invalid mapped cloud layout')
            elif (message.header.frame_id != 'camera_optical_frame' or len(message.data) > 4*1024**2
                    or not 0 < message.width*message.height <= 300000):
                raise ValueError('Unexpected/oversized native depth image')
            self.last_stamp[kind] = timestamp
            with self.lock:
                self.pending[kind] = (sequence, message, time.monotonic(), self.intrinsic)
            self.wake.set()
        except (ValueError, TypeError) as exc:
            self.applied_sequence[kind] = sequence
            setattr(self, kind, None)
            self.errors[kind] = str(exc)

    def transform(self, message, points):
        if message.header.frame_id == 'map':
            return points
        transform = self.buffer.lookup_transform('map', message.header.frame_id,
            rclpy.time.Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.))
        t, q = transform.transform.translation, transform.transform.rotation
        return transform_points(points, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def decode(self, kind, entry):
        sequence, message, received, intrinsic = entry
        self.stamp(message, MAP_MAX_AGE_S if kind == 'map' else .75)
        metadata = {}
        if kind == 'map':
            from sensor_msgs_py.point_cloud2 import read_points_numpy
            if message.width*message.height == 0:
                points = np.empty((0, 3))
            else:
                points = read_points_numpy(message, field_names=['x', 'y', 'z'], skip_nans=True)
            if not np.all(np.isfinite(points)):
                raise ValueError('Non-finite mapped occupancy rejected')
        else:
            if not intrinsic or time.monotonic()-intrinsic[1] > 2.:
                raise ValueError('Fresh registered CameraInfo required')
            info = intrinsic[0]
            if info.width != message.width or info.height != message.height or info.header.frame_id != message.header.frame_id:
                raise ValueError('Depth and intrinsics dimensions/frame differ')
            if message.encoding not in ('32FC1', '16UC1'):
                raise ValueError('Expected 32FC1 metres or 16UC1 millimetres')
            size = 4 if message.encoding == '32FC1' else 2
            if message.step < size*message.width or len(message.data) < message.step*message.height:
                raise ValueError('Incomplete depth rows')
            dtype = np.dtype(('>' if message.is_bigendian else '<')+('f4' if size == 4 else 'u2'))
            image = np.ndarray((message.height, message.width), dtype=dtype,
                buffer=message.data, strides=(message.step, size)).astype(float)
            if size == 2:
                image *= .001
            points, metadata = depth_points(image, info.k)
        # TF can arrive just after a frame. Wait here, never in a ROS/FCU callback.
        deadline = time.monotonic()+.12
        while True:
            try:
                points = self.transform(message, points)
                break
            except Exception:
                if time.monotonic() >= deadline or self.closed.is_set():
                    raise ValueError('Capture-time map/optical -> FCU map transform unavailable')
                self.closed.wait(.01)
        if kind == 'map':
            with self.lock:
                version, columns = self.swept_snapshot
            self.raw_map = dict(points=points, swept_version=version, received=received,
                                stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                                source_frame=message.header.frame_id)
            points, metadata['swept_removed'] = remove_swept(points, columns)
        layer = VoxelObstacles(points)
        # Clusters for moving-obstacle tracking (R5.11): the depth voxel centres (about 1000, not the
        # raw ~36000 points, which delayed this worker past the freshness limit) that lie neither on
        # the ground nor within 0.35 m of the known static world.
        clusters = self.unexplained_clusters(layer) if kind == 'depth' else []
        return dict(sequence=sequence, layer=layer, received=received, clusters=clusters,
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            source_frame=message.header.frame_id, frame='map', **metadata)

    def unexplained_clusters(self, layer):
        centres = np.asarray(layer.centres)
        floor = getattr(self, 'ground_z', None)
        if floor is not None and len(centres):
            # The floor from the depth itself: the FCU height estimate is off by 0.2-0.6 m from run to run, so
            # a fixed band let the floor ahead merge with walkers into one oversized blob that was never
            # tracked (2026-11-13). Per 1 m cell, the lowest point near the expected ground is the floor there.
            cells, inverse = np.unique(np.floor(centres[:, :2]/FLOOR_CELL_M).astype(int), axis=0, return_inverse=True)
            lowest = np.full(len(cells), np.inf)
            np.minimum.at(lowest, inverse.ravel(), centres[:, 2])
            local = np.where(np.abs(lowest-floor) <= FLOOR_SEARCH_M, lowest, floor)[inverse.ravel()]
            centres = centres[centres[:, 2] > local+.3]
        static = getattr(self, 'static_scene', None)
        if static is not None and len(centres):
            keep = [static.clearance(point) > .35 for point in centres[:3000]]
            centres = centres[:3000][np.asarray(keep, dtype=bool)]
        return cluster_points(centres, voxel=.3, min_points=3) if len(centres) else []

    def work(self):
        while not self.closed.is_set():
            self.wake.wait(.2)
            self.wake.clear()
            with self.lock:
                entries, self.pending = self.pending, {}
            # One latest message per source; no unbounded ROS/image backlog.
            for kind, entry in entries.items():
                try:
                    result = self.decode(kind, entry)
                except Exception as exc:
                    result = dict(sequence=entry[0], error=str(exc), received=entry[2])
                try:
                    self.results.put_nowait((kind, result))
                except queue.Full:
                    try:
                        self.results.get_nowait()
                    except queue.Empty:
                        pass
                    self.results.put_nowait((kind, result))
            self.refilter()

    def refilter(self):
        """Re-apply the swept memory to the latest map when it has grown (ghosts of people who walked on)."""
        raw = self.raw_map
        if raw is None or time.monotonic()-self.last_refilter < REFILTER_S:
            return
        with self.lock:
            version, columns = self.swept_snapshot
            if version == raw['swept_version'] or 'map' in self.pending:
                return                          # unchanged, or a newer map is about to be decoded
            self.sequence += 1
            sequence = self.sequence
        self.last_refilter = time.monotonic()
        points, removed = remove_swept(raw['points'], columns)
        result = dict(sequence=sequence, layer=VoxelObstacles(points), received=raw['received'], clusters=[],
                      stamp=raw['stamp'], source_frame=raw['source_frame'], frame='map', swept_removed=removed)
        try:
            self.results.put_nowait(('map', result))
            raw['swept_version'] = version
        except queue.Full:
            pass                                # retried on the next pass

    def poll(self):
        while True:
            try:
                kind, result = self.results.get_nowait()
            except queue.Empty:
                return
            if result['sequence'] <= self.applied_sequence[kind]:
                continue
            self.applied_sequence[kind] = result['sequence']
            if 'error' in result:
                setattr(self, kind, None)
                self.errors[kind] = result['error']
            else:
                setattr(self, kind, result)
                self.errors[kind] = ''
                if kind == 'depth' and result['stamp'] != self.tracked_stamp:
                    self.tracked_stamp = result['stamp']
                    self.tracker.ground_z = getattr(self, 'ground_z', None)
                    self.tracker.update(result['clusters'], result['stamp'])
                    moving = self.tracker.moving()
                    if moving:
                        self.swept.record(moving, time.monotonic())
                        with self.lock:
                            self.swept_snapshot = self.swept.snapshot()

    def ready(self):
        return bool(self.map and self.depth and self.intrinsic and
            time.monotonic()-self.map['received'] < MAP_MAX_AGE_S and time.monotonic()-self.depth['received'] < .75
            and time.monotonic()-self.intrinsic[1] < 2.)

    def snapshot(self):
        if not self.ready():
            raise ValueError('Fresh mapped occupancy, registered depth and capture-time transforms required')
        return (self.map['layer'], self.depth['layer']), self.state()

    def route_guard(self, position, route):
        if not self.ready():
            return 'Observed route held: sensor/map/intrinsics stale or unavailable'
        start, remaining = np.asarray(position), 2.
        for waypoint in route:
            end = np.asarray(waypoint)
            distance = float(np.linalg.norm(end-start))
            if distance > remaining:
                end = start+(end-start)*(remaining/distance)
                distance = remaining
            for observation in (self.map['layer'], self.depth['layer']):
                if not observation.segment_clear(start, end, radius=.7):
                    return 'Observed route held: current mapped/depth occupancy intersects the 2 m lookahead'
            remaining -= distance
            if remaining <= 1e-8:
                break
            start = end
        conflict = time_conflict(position, route, ROUTE_SPEED, self.moving_tracks())
        if conflict:
            when, track = conflict
            return ('Observed route held: moving obstacle %d (%.1f m/s) predicted on the route in %.1f s'
                    % (track.id, track.speed, when))
        return ''

    def sensing_fresh(self):
        """The camera itself (depth and intrinsics) is fresh: the sensor-loss test, independent of map updates."""
        return bool(self.depth and self.intrinsic and time.monotonic()-self.depth['received'] < .75
                    and time.monotonic()-self.intrinsic[1] < 2.)

    def moving_tracks(self):
        """Moving tracks whose last update is fresh (stale tracks are not extrapolated)."""
        if not self.depth or time.monotonic()-self.depth['received'] > .75:
            return []
        return self.tracker.moving()

    def state(self):
        result = dict(enabled=True, ready=self.ready(), frame='map', radius_m=.7,
            voxel_m=.15, lookahead_m=2., scope='Observed occupancy supplements known static world; unseen/dynamic prediction unqualified')
        result['intrinsics_ready'] = bool(self.intrinsic and time.monotonic()-self.intrinsic[1] < 2.)
        result['tracks'] = [track.state() for track in self.tracker.tracks][:20]
        result['moving_tracks'] = len(self.moving_tracks())
        result['swept_columns'] = len(self.swept.columns)
        result['depth_clusters'] = len(self.depth['clusters']) if self.depth else None
        for kind in ('map', 'depth'):
            value = getattr(self, kind)
            result[kind] = dict(age_wall_s=time.monotonic()-value['received'], stamp=value['stamp'],
                occupied_cells=value['layer'].cell_count, source_points=value['layer'].source_points,
                swept_removed=value.get('swept_removed'),
                sha256=value['layer'].sha256, source_frame=value['source_frame']) if value else dict(error=self.errors[kind])
        return result

    def close(self):
        self.closed.set()
        self.wake.set()
        self.worker.join(timeout=1.)
