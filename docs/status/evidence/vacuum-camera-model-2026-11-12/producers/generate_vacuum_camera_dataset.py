#!/usr/bin/env python3
"""Render source-camera training scenes with actual Gazebo RGB/semantic sensors.

Only a dedicated camera fixture is repositioned. This is dataset generation,
not a robot motion/mission test. Run one invocation at a time on an unused ROS
domain and Gazebo partition, with SSD output and a sourced ROS installation.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PILImage

from capture_vacuum_detector_data import annotations_for, CLASSES
from robot_lab_vacuum_cleaning.camera_observations import aligned_pair, image_array, stamp_seconds
from vacuum_object_world import model_sdf

ROOT = Path(__file__).resolve().parents[1]
XACRO = '{http://ros.org/wiki/xacro}'


def source_camera():
    """Copy the robot's literal camera contract, not hand-chosen intrinsics."""
    source = ROOT / 'src/robot_lab_robots/oomwoo/urdf/oomwoo_gazebo.xacro'
    macro = ET.parse(source).find(f"{XACRO}macro[@name='front_camera_common']")
    if macro is None:
        raise ValueError('Source camera macro missing')
    children = [copy.deepcopy(child) for child in macro]
    camera = next(c for c in children if c.tag == 'camera')
    for child in list(camera):
        if child.tag.startswith(XACRO):
            camera.remove(child)
    pose = ET.parse(ROOT / 'src/robot_lab_robots/oomwoo/urdf/oomwoo.urdf.xacro').find(
        "joint[@name='front_camera_joint']/origin")
    pitch = float(pose.get('rpy').split()[1])
    # base_link is at the 35 mm axle; camera joint is 40 mm above it.
    height = .035 + float(pose.get('xyz').split()[2])
    return children, height, pitch


def fixture_world(map_name, seed, output):
    rng = np.random.default_rng(seed)
    source = ROOT / f'src/robot_lab_maps/maps/{map_name}/worlds/{map_name}.world'
    root = ET.parse(source).getroot()
    world = root.find('world')
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'ignition-gazebo-{filename}-system',
                               name=f'ignition::gazebo::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    center = (0., 0.) if map_name == 'nav_empty' else (4., 0.)
    objects = []
    for i, (name, delta) in enumerate(zip(CLASSES, [(-.6, -.5), (.6, .5), (-.6, .5), (.6, -.5)])):
        obj = {'class': name, 'x': center[0] + delta[0] + float(rng.uniform(-.15, .15)),
               'y': center[1] + delta[1] + float(rng.uniform(-.15, .15)),
               'yaw': float(rng.uniform(-math.pi, math.pi))}
        world.append(ET.fromstring(model_sdf(i, obj)))
        objects.append(obj)
    children, height, pitch = source_camera()
    model = ET.SubElement(world, 'model', name='dataset_camera_fixture')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'0 0 {height} 0 {pitch} 0'
    link = ET.SubElement(model, 'link', name='camera')
    for name, kind, topic in [('rgb', 'rgbd_camera', 'vacuum_camera'),
                               ('labels', 'segmentation', 'vacuum_labels')]:
        sensor = ET.SubElement(link, 'sensor', name=name, type=kind)
        for child in children:
            sensor.append(copy.deepcopy(child))
        ET.SubElement(sensor, 'topic').text = topic
        if kind == 'segmentation':
            ET.SubElement(sensor.find('camera'), 'segmentation_type').text = 'semantic'
    output.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True))
    return world.get('name'), objects, height, pitch, source


def stop_owned(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def generate(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    if (args.out / 'annotations.json').exists():
        raise ValueError('Use a fresh dataset directory; previous captures are immutable')
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'images').mkdir()
    dataset = {'images': [], 'annotations': [],
               'categories': [{'id': i, 'name': name} for name, i in CLASSES.items()]}
    rng = np.random.default_rng(args.seed)
    start = time.monotonic()
    rclpy.init()
    node = rclpy.create_node('vacuum_render_dataset')
    latest = {}
    subscriptions = [node.create_subscription(Image, topic, lambda m, k=key: latest.update({k: m}),
                                              qos_profile_sensor_data)
                     for key, topic in [('rgb', '/vacuum_camera/image'),
                                        ('labels', '/vacuum_labels/labels_map')]]
    receipt = {'scope': 'actual Gazebo source-camera fixture renders; no robot mission',
               'seed': args.seed, 'scenes': [], 'complete': False}
    try:
        scenes = [('nav_empty', args.seed, 'train'), ('nav_empty', args.seed + 1, 'train'),
                  ('nav_obstacle', args.seed + 2, 'val')]
        for scene_i, (map_name, seed, split) in enumerate(scenes):
            if time.monotonic() - start >= args.timeout:
                raise TimeoutError('Dataset generation wall budget exhausted')
            scene_id = f'{map_name}-layout-{seed}'
            case = args.out / scene_id
            case.mkdir()
            world, objects, height, pitch, source = fixture_world(map_name, seed, case / 'world.sdf')
            scene = {'scene': scene_id, 'split': split, 'map': map_name,
                     'map_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                     'world_sha256': hashlib.sha256((case / 'world.sdf').read_bytes()).hexdigest(),
                     'objects': objects, 'frames': 0}
            receipt['scenes'].append(scene)
            latest.clear()
            logs = [(case / 'gazebo.log').open('w'), (case / 'bridge.log').open('w')]
            processes = []
            try:
                processes.append(subprocess.Popen(['ign', 'gazebo', '-s', '-r', '--headless-rendering',
                                                    str(case / 'world.sdf')],
                                                   stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True))
                processes.append(subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    '/vacuum_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                    '/vacuum_labels/labels_map@sensor_msgs/msg/Image[ignition.msgs.Image'],
                    stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True))
                ready = time.monotonic() + 45
                while (not aligned_pair(latest.get('rgb'), latest.get('labels'))
                       and time.monotonic() < ready):
                    rclpy.spin_once(node, timeout_sec=.05)
                    if any(p.poll() is not None for p in processes):
                        raise RuntimeError('Rendering or bridge process exited')
                if not aligned_pair(latest.get('rgb'), latest.get('labels')):
                    raise TimeoutError('No aligned Gazebo camera frames within 45 wall seconds')
                for view in range(args.views):
                    if time.monotonic() - start >= args.timeout:
                        raise TimeoutError('Dataset generation wall budget exhausted')
                    if (case / 'gazebo.log').stat().st_size > 64 * 1024 * 1024:
                        raise RuntimeError('Rendering log exceeds the 64 MB budget')
                    target = objects[view % len(objects)]
                    yaw = float(rng.uniform(-math.pi, math.pi))
                    distance = float(rng.uniform(.28, 1.3))
                    x, y = target['x'] - distance * math.cos(yaw), target['y'] - distance * math.sin(yaw)
                    qx = -math.sin(pitch / 2) * math.sin(yaw / 2)
                    qy = math.sin(pitch / 2) * math.cos(yaw / 2)
                    qz = math.cos(pitch / 2) * math.sin(yaw / 2)
                    qw = math.cos(pitch / 2) * math.cos(yaw / 2)
                    request = (f'name: "dataset_camera_fixture", position: {{x: {x}, y: {y}, z: {height}}}, '
                               f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
                    old = stamp_seconds(latest['rgb'])
                    response = subprocess.run(['ign', 'service', '-s', f'/world/{world}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '2000', '--req', request], capture_output=True, text=True, timeout=4)
                    if response.returncode or 'data: true' not in response.stdout:
                        raise RuntimeError('Camera fixture pose request failed: ' + response.stdout + response.stderr)
                    until = time.monotonic() + 10
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.05)
                        if (aligned_pair(latest.get('rgb'), latest.get('labels'))
                                and stamp_seconds(latest['rgb']) > old + .4):
                            break
                    else:
                        raise TimeoutError('Camera did not produce a fresh registered pair after pose change')
                    rgb, labels = image_array(latest['rgb'], rgb=True), image_array(latest['labels'])
                    if rgb is None or labels is None:
                        raise ValueError('Unsupported camera image encoding')
                    image_id = len(dataset['images']) + 1
                    filename = f'{image_id:06}.png'
                    path = args.out / 'images' / filename
                    PILImage.fromarray(rgb, 'RGB').save(path)
                    rows = annotations_for(labels, image_id, len(dataset['annotations']) + 1)
                    dataset['annotations'].extend(rows)
                    dataset['images'].append({'id': image_id, 'file_name': filename,
                        'width': rgb.shape[1], 'height': rgb.shape[0], 'scene': scene_id, 'split': split,
                        'stamp': stamp_seconds(latest['rgb']), 'camera_pose': [x, y, height, 0., pitch, yaw],
                        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'source': 'actual Gazebo RGB/semantic source-camera fixture'})
                    scene['frames'] += 1
                    if view % 8 == 0:
                        print(f'{scene_id}: {view+1}/{args.views} rendered views', flush=True)
            finally:
                for process in reversed(processes):
                    stop_owned(process)
                for log in logs:
                    log.close()
        receipt['complete'] = True
    finally:
        receipt['wall_s'] = time.monotonic() - start
        (args.out / 'render-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        (args.out / 'annotations.json').write_text(json.dumps(dataset, indent=1) + '\n')
        (args.out / 'val_ids.json').write_text(json.dumps([i['id'] for i in dataset['images']
                                                         if i['split'] == 'val']) + '\n')
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(receipt), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--views', type=int, default=48, help='Views per independent scene (3 scenes)')
    parser.add_argument('--seed', type=int, default=29)
    parser.add_argument('--timeout', type=float, default=900.)
    args = parser.parse_args()
    if args.views <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Positive finite views and timeout required')
    generate(args)


if __name__ == '__main__':
    main()
