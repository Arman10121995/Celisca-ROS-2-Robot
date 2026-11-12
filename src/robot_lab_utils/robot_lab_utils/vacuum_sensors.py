"""Source-preserving Create 3 hazards and per-description Gazebo bridges."""
import copy
import xml.etree.ElementTree as ET


def create3_hazard_derivative(description, source_description, model_name):
    root, source = ET.fromstring(description), ET.fromstring(source_description)
    root.set('name', model_name)
    restored, links = [], set()
    for block in source.findall('gazebo'):
        sensor = block.find('sensor')
        frame = block.get('reference', '')
        if sensor is None or frame != 'bumper' and not frame.startswith('cliff_'):
            continue
        if root.find(f"link[@name='{frame}']") is None:
            raise ValueError('Create 3 source hazard frame is missing: ' + frame)
        mounted = copy.deepcopy(block)
        sensor = mounted.find('sensor')
        # Declared lab publication cadence; original geometry/scan bounds stay exact.
        sensor.find('update_rate').text = '20'
        for plugin in sensor.findall('plugin'):
            sensor.remove(plugin)
        ET.SubElement(sensor, 'gz_frame_id').text = frame
        if frame.startswith('cliff_'):
            ET.SubElement(sensor, 'topic').text = '/cliff/' + frame.removeprefix('cliff_')
        root.append(mounted)
        links.add(frame)
        restored.append({'frame': frame, 'sensor': sensor.get('name'), 'type': sensor.get('type')})
    if links != {'bumper', 'cliff_side_left', 'cliff_side_right', 'cliff_front_left', 'cliff_front_right'}:
        raise ValueError('A complete source bumper/four-cliff contract is required')
    # Keep scoped native contact/range sensor paths stable after URDF conversion.
    for joint in root.findall('joint'):
        if joint.find('child') is None or joint.find('child').get('link') not in links:
            continue
        block = ET.SubElement(root, 'gazebo', reference=joint.get('name'))
        ET.SubElement(block, 'preserveFixedJoint').text = 'true'
    for plugin in root.findall('gazebo/plugin'):
        for key in ('odom_topic', 'tf_topic'):
            node = plugin.find(key)
            if node is not None and node.text and node.text.startswith('/model/'):
                node.text = '/model/' + model_name + '/' + node.text.rsplit('/', 1)[1]
    return ET.tostring(root, encoding='unicode'), restored


def hazard_bridge_entries(description, world, robot):
    """Bridge source sensor names instead of assuming oomwoo link names."""
    root = ET.fromstring(description)
    entries = []
    for block in root.findall('gazebo'):
        frame = block.get('reference', '')
        for sensor in block.findall('sensor'):
            kind, name = sensor.get('type'), sensor.get('name', '')
            if kind == 'contact' and 'bumper' in (frame + name):
                topic = f'/world/{world}/model/{robot}/link/{frame}/sensor/{name}/contact'
                entries.append({'ros_topic_name': '/bumper', 'gz_topic_name': topic,
                                'ros_type_name': 'ros_gz_interfaces/msg/Contacts',
                                'gz_type_name': 'gz.msgs.Contacts', 'direction': 'GZ_TO_ROS'})
            elif kind in ('gpu_lidar', 'ray', 'gpu_ray') and frame.startswith('cliff_'):
                topic = sensor.findtext('topic')
                if not topic:
                    raise ValueError('Native cliff sensor needs a declared topic: ' + name)
                entries.append({'ros_topic_name': topic if topic.startswith('/cliff/') else '/cliff/' + frame.removeprefix('cliff_'),
                                'gz_topic_name': topic, 'ros_type_name': 'sensor_msgs/msg/LaserScan',
                                'gz_type_name': 'gz.msgs.LaserScan', 'direction': 'GZ_TO_ROS'})
    if sum(e['ros_topic_name'] == '/bumper' for e in entries) > 1:
        raise ValueError('Multiple bumper sources require an explicit aggregation contract')
    return entries
