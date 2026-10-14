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
            timestamp = self.stamp(message, 10. if kind == 'map' else .75)
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
        self.stamp(message, 10. if kind == 'map' else .75)
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
        layer = VoxelObstacles(points)
        return dict(sequence=sequence, layer=layer, received=received,
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            source_frame=message.header.frame_id, frame='map', **metadata)

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

    def ready(self):
        return bool(self.map and self.depth and self.intrinsic and
            time.monotonic()-self.map['received'] < 10. and time.monotonic()-self.depth['received'] < .75
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
        return ''

    def state(self):
        result = dict(enabled=True, ready=self.ready(), frame='map', radius_m=.7,
            voxel_m=.15, lookahead_m=2., scope='Observed occupancy supplements known static world; unseen/dynamic prediction unqualified')
        result['intrinsics_ready'] = bool(self.intrinsic and time.monotonic()-self.intrinsic[1] < 2.)
        for kind in ('map', 'depth'):
            value = getattr(self, kind)
            result[kind] = dict(age_wall_s=time.monotonic()-value['received'], stamp=value['stamp'],
                occupied_cells=value['layer'].cell_count, source_points=value['layer'].source_points,
                sha256=value['layer'].sha256, source_frame=value['source_frame']) if value else dict(error=self.errors[kind])
        return result

    def close(self):
        self.closed.set()
        self.wake.set()
        self.worker.join(timeout=1.)
