#!/usr/bin/env python3
"""Save acquired maps, including RTAB-Map memory, without editing saved inputs."""
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time
import uuid

import numpy as np


def snapshot_database(source, destination, timeout=10.):
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError('A map snapshot must not overwrite its source')
    deadline = time.monotonic()+timeout
    temporary = destination.with_suffix('.partial')
    def progress(*args):
        if time.monotonic()>deadline:
            raise TimeoutError('Database snapshot exceeded its time budget')
    try:
        with sqlite3.connect(source.resolve().as_uri()+'?mode=ro', uri=True, timeout=2.) as src:
            with sqlite3.connect(temporary) as dst:
                src.backup(dst, pages=256, progress=progress, sleep=.01)
                if dst.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
                    raise ValueError('Acquired database did not pass SQLite consistency check')
                nodes = dst.execute('SELECT COUNT(*) FROM Node WHERE id>0').fetchone()[0]
                if nodes < 1:
                    raise ValueError('No acquired RTAB-Map nodes were flushed; map is not saved')
        temporary.replace(destination)
        return dict(nodes=nodes, bytes=destination.stat().st_size,
                    sha256=hashlib.sha256(destination.read_bytes()).hexdigest())
    finally:
        temporary.unlink(missing_ok=True)


def load_database_copy(database, destination, world_file=None):
    """Copy a saved database read-only; reject known world provenance mismatch."""
    source = Path(database).expanduser().resolve()
    if not source.is_file() or not str(source).startswith('/workspace/'):
        raise ValueError('Select an existing saved database on the workspace SSD')
    manifest = source.parent/'manifest.json'
    if world_file and manifest.is_file():
        previous = json.loads(manifest.read_text())
        expected = previous.get('world_sha256')
        if expected and expected != hashlib.sha256(Path(world_file).read_bytes()).hexdigest():
            raise ValueError('Saved map belongs to different world geometry; select its source world')
    if source != Path(destination).resolve():
        snapshot_database(source, destination)
    return source


def save_occupancy(msg, folder, world_file='', robot='', initial_pose=None):
    from PIL import Image
    import yaml
    width, height = int(msg.info.width), int(msg.info.height)
    if not 0 < width*height <= 16000000 or len(msg.data) != width*height or msg.info.resolution <= 0:
        raise ValueError('A real, finite-sized occupancy grid is required')
    cells = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
    if not np.any(cells >= 0) or np.any((cells < -1) | (cells > 100)):
        raise ValueError('No acquired known occupancy, or invalid occupancy values')
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    image = np.full(cells.shape, 205, dtype=np.uint8)
    image[(cells >= 0) & (cells <= 25)] = 254
    image[cells >= 65] = 0
    Image.fromarray(np.flipud(image)).save(folder/'map.pgm')
    p, q = msg.info.origin.position, msg.info.origin.orientation
    if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,msg.info.resolution)):
        raise ValueError('Occupancy origin and resolution must be finite')
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    data = dict(image='map.pgm', mode='trinary', resolution=float(msg.info.resolution),
                origin=[float(p.x), float(p.y), yaw], negate=0, occupied_thresh=.65, free_thresh=.25)
    (folder/'map.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
    manifest = dict(kind='acquired_occupancy', robot=robot, frame=msg.header.frame_id,
        known_cells=int(np.count_nonzero(cells >= 0)), occupied_cells=int(np.count_nonzero(cells >= 65)),
        source_world=str(world_file), world_sha256=hashlib.sha256(Path(world_file).read_bytes()).hexdigest() if world_file else None,
        map_yaml=str(folder/'map.yaml'), initial_pose_in_map=initial_pose,
        scope='Actual acquired grid; saved-map localization/navigation qualification is separate')
    (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


class RtabmapArchive:
    """Asynchronous official backup acknowledgement, then measured cloud/DB export.

    The upstream backup service flushes working memory before making .back.
    Both old Humble and newer node-private service names are supported.
    """
    def __init__(self, node, database, namespace, prefix, directory, world_file, robot, on_result=None):
        from rclpy.clock import Clock, ClockType
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Empty, Trigger
        self.node, self.database = node, str(database)
        self.directory, self.world_file, self.robot = Path(directory), str(world_file), robot
        self.on_result = on_result
        self.points = None
        self.cloud_stamp = None
        self.cloud_at = -math.inf
        self.pending = None
        self.started = 0.
        self.request_wall = 0.
        self.result = None
        self.phase, self.message = 'idle', 'Acquire a real cloud before saving'
        self.clients = [node.create_client(Empty, namespace.rstrip('/')+'/'+name)
                        for name in ('backup', 'rtabmap/backup')]
        self.publisher = node.create_publisher(String, prefix+'/state', 10)
        node.create_subscription(PointCloud2, namespace.rstrip('/')+'/cloud_map', self.cloud, 10)
        node.create_service(Trigger, prefix+'/save', self.service)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(.2, self.tick, clock=self.clock)

    def cloud(self, msg):
        if msg.header.frame_id != 'map' or msg.width*msg.height > 300000 or time.monotonic()-self.cloud_at < .5:
            return
        from sensor_msgs_py import point_cloud2
        try:
            values = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            values = np.asarray(values, dtype=np.float32).reshape(-1, 3)
            if len(values):
                self.points, self.cloud_at = values.copy(), time.monotonic()
                self.cloud_stamp = dict(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec)
        except (ValueError, TypeError, AssertionError) as exc:
            self.message = 'Acquired cloud conversion refused: '+str(exc)

    def service(self, request, response):
        try:
            self.request()
            response.success, response.message = True, 'Save requested; wait for mapping state to report saved or failed'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def request(self):
        from std_srvs.srv import Empty
        if self.pending is not None:
            raise ValueError('A map save is already waiting for the actual RTAB-Map backup')
        if not self.database or not Path(self.database).is_file() or self.points is None or time.monotonic()-self.cloud_at > 5.:
            raise ValueError('Start RGB-D SLAM and acquire a fresh real 3D cloud before saving')
        client = next((c for c in self.clients if c.service_is_ready()), None)
        if client is None:
            raise ValueError('RTAB-Map backup service is not ready; memory cannot be flushed')
        self.request_points, self.request_stamp = self.points.copy(), self.cloud_stamp
        self.result = None
        self.started, self.request_wall = time.monotonic(), time.time()
        self.pending = client.call_async(Empty.Request())
        self.phase, self.message = 'saving', 'Waiting for RTAB-Map to flush actual working memory'

    def tick(self):
        from std_msgs.msg import String
        if self.pending is not None and (self.pending.done() or time.monotonic()-self.started > 20.):
            try:
                if not self.pending.done():
                    self.pending.cancel()
                    raise TimeoutError('Actual RTAB-Map backup reply timed out')
                self.pending.result()
                source = Path(self.database+'.back')
                if not source.is_file() or source.stat().st_mtime < self.request_wall-1.:
                    raise ValueError('RTAB-Map did not produce a fresh flushed database backup')
                folder = self.directory/('map-'+uuid.uuid4().hex[:10])
                folder.mkdir(parents=True)
                database = snapshot_database(source, folder/'rtabmap.db')
                np.savez_compressed(folder/'occupied.npz', points=self.request_points, voxel=.2)
                manifest = dict(kind='acquired_rtabmap', robot=self.robot, frame='map', directory=str(folder),
                    source_world=self.world_file, world_sha256=hashlib.sha256(Path(self.world_file).read_bytes()).hexdigest(),
                    source_database=self.database, database=str(folder/'rtabmap.db'), database_observation=database,
                    points=len(self.request_points), cloud_stamp=self.request_stamp,
                    scope='Acquired RGB-D SLAM database/cloud; physical reload/mission qualification is separate')
                (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
                self.result = manifest
                self.phase, self.message = 'saved', 'Saved actual database and occupied cloud: '+str(folder)
            except (OSError, ValueError, sqlite3.Error, TimeoutError) as exc:
                self.phase, self.message = 'failed', 'Map save failed: '+str(exc)
            self.pending = None
            if self.on_result:
                self.on_result(self.phase, self.message, self.result)
        self.publisher.publish(String(data=json.dumps(dict(status=self.phase, reason=self.message,
            result=self.result, acquired_points=len(self.points) if self.points is not None else 0), allow_nan=False)))


def main(args=None):
    import os
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    node = Node('gait_map_archive')
    defaults = dict(database_path='', world_file='', robot_id='', task='slam', session_directory='')
    for key, value in defaults.items():
        node.declare_parameter(key, value)
    config = {key: node.get_parameter(key).value for key in defaults}
    base = Path(config['session_directory'] or Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
        '/workspace/molar/robot_lab_runtime'))/'mapping_sessions'/uuid.uuid4().hex)
    base.mkdir(parents=True,exist_ok=True)
    if base.stat().st_dev == Path('/').stat().st_dev:
        raise ValueError('Acquired maps must stay on the mounted workspace SSD')
    folder = base/'saved_maps'
    if config['task'] == '3d_slam':
        archive = RtabmapArchive(node, config['database_path'], '/gait/slam', '/gait/mapping',
                                 folder, config['world_file'], config['robot_id'])
    else:
        from tf2_ros import Buffer, TransformListener
        buffer=Buffer(node=node);listener=TransformListener(buffer,node)
        state = dict(msg=None)
        publisher = node.create_publisher(String, '/gait/mapping/state', 10)
        node.create_subscription(OccupancyGrid, '/gait/map', lambda msg: state.update(msg=msg), 1)
        def save(request, response):
            try:
                if state['msg'] is None:
                    raise ValueError('No acquired gait occupancy map yet')
                transform=buffer.lookup_transform('map','gait/odom',rclpy.time.Time())
                t,q=transform.transform.translation,transform.transform.rotation
                initial=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                result = save_occupancy(state['msg'], folder/('map-'+uuid.uuid4().hex[:10]),
                                        config['world_file'], config['robot_id'],initial_pose=initial)
                publisher.publish(String(data=json.dumps(dict(status='saved', result=result))))
                response.success, response.message = True, result['map_yaml']
            except Exception as exc:
                response.success, response.message = False, str(exc)
            return response
        node.create_service(Trigger, '/gait/mapping/save', save)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
