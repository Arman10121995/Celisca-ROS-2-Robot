"""2D SLAM with lab_tracked through the normal launch, scored against the installed map.

Usage: LAB_EXTENSION_RUN=<dir> tracked_slam_probe.py <simulator> [map]

mode:=slam (slam_toolbox). After readiness: forward 0.15 m/s for 10 s,
turn 0.4 rad/s for 4 s, forward 0.15 m/s for 8 s, stop. The last /map is
compared with the world's installed occupancy map: each SLAM occupied cell
goes into the world frame through the spawn pose (the SLAM map frame starts
at the robot), and its distance to the nearest installed occupied cell is
measured. Truth is /odom/ground_truth.
"""
import json, math, os, signal, subprocess, sys, time
from pathlib import Path
import numpy as np
import rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory as share
from robot_lab_gui.process_control import stop_owned_launch

simulator = sys.argv[1]; world = sys.argv[2] if len(sys.argv) > 2 else 'nav_obstacle'
folder = Path(os.environ['LAB_EXTENSION_RUN']) / ('tracked-slam-%s-%s-%d' % (simulator, world, time.time_ns())); folder.mkdir(parents=True)
cmd = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'robot_model:=lab_tracked', 'mode:=slam', 'map_name:=' + world,
       'simulator:=' + simulator, 'gui:=false', 'start_rviz:=false', 'enable_joystick:=false']
(folder / 'command.json').write_text(json.dumps(cmd)); log = (folder / 'launch.log').open('w')
process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
profile = yaml.safe_load((Path(share('robot_lab_bringup')) / 'config/sim_maps.yaml').read_text())['maps'][world]
spawn = [float(profile['spawn'][k]) for k in ('x', 'y', 'yaw')]
rclpy.init(); node = Node('tracked_slam_probe'); body = []; states = []; grids = []
node.create_subscription(Odometry, '/odom/ground_truth',
                         lambda m: body.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y]),
                         qos_profile_sensor_data)
node.create_subscription(String, '/tracked/state', lambda m: states.append(json.loads(m.data)), 10)
node.create_subscription(OccupancyGrid, '/map', grids.append,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
pub = node.create_publisher(Twist, '/key_vel', 10)
phases = [(0.15, 0., 10.), (0., 0.4, 4.), (0.15, 0., 8.), (0., 0., 6.)]
result = dict(simulator=simulator, map=world, spawn=spawn)
try:
    start = time.monotonic()
    while time.monotonic() - start < 180 and not (states and states[-1]['ready']) and process.poll() is None:
        rclpy.spin_once(node, timeout_sec=.05)
    # slam_toolbox starts 7 s after the plant (tracked_simulator TimerAction).
    wait = time.monotonic()
    while time.monotonic() - wait < 15: rclpy.spin_once(node, timeout_sec=.05)
    for linear, angular, duration in phases:
        begin = time.monotonic()
        while time.monotonic() - begin < duration:
            msg = Twist(); msg.linear.x, msg.angular.z = linear, angular
            if linear or angular: pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.05)
    if not grids: raise RuntimeError('no /map received')
    grid = grids[-1]; info = grid.info
    cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    occupied = np.argwhere(cells >= 65); known = int((cells >= 0).sum())
    ox, oy = info.origin.position.x, info.origin.position.y
    slam_xy = np.stack([ox + (occupied[:, 1] + .5) * info.resolution, oy + (occupied[:, 0] + .5) * info.resolution], axis=1)
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    world_xy = np.stack([spawn[0] + c * slam_xy[:, 0] - s * slam_xy[:, 1], spawn[1] + s * slam_xy[:, 0] + c * slam_xy[:, 1]], axis=1)
    reference_yaml = Path(share(profile['map'].get('package', 'robot_lab_maps'))) / profile['map']['path']
    meta = yaml.safe_load(reference_yaml.read_text())
    from PIL import Image
    image = np.asarray(Image.open(reference_yaml.parent / meta['image']).convert('L'), dtype=float) / 255.
    occupancy = (1. - image) if not meta.get('negate') else image
    ref = np.argwhere(occupancy > meta['occupied_thresh'])
    h = image.shape[0]; r = meta['resolution']
    ref_xy = np.stack([meta['origin'][0] + (ref[:, 1] + .5) * r, meta['origin'][1] + (h - 1 - ref[:, 0] + .5) * r], axis=1)
    errors = []
    for point in world_xy:
        errors.append(float(np.min(np.hypot(*(ref_xy - point).T))))
    errors = np.asarray(errors)
    result.update(known_cells=known, occupied_cells=int(len(occupied)), resolution=info.resolution,
                  occupied_error_median_m=round(float(np.median(errors)), 4) if len(errors) else None,
                  occupied_error_p95_m=round(float(np.percentile(errors, 95)), 4) if len(errors) else None,
                  occupied_within_0_1m=round(float((errors <= .1).mean()), 4) if len(errors) else None,
                  travel_m=round(sum(math.dist(a[1:], b[1:]) for a, b in zip(body, body[1:])), 3) if body else 0.)
    result['passed'] = bool(len(errors) >= 50 and result['occupied_within_0_1m'] >= .9)
except Exception as exc:
    result['error'] = str(exc); result['passed'] = False
finally:
    pub.publish(Twist()); node.destroy_node(); rclpy.shutdown(); stop_owned_launch(process, grace_timeout=15); log.close()
    (folder / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
