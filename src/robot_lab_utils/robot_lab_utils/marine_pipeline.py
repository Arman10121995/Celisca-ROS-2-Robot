"""Compose real marine sensor, estimation, mission and optional RTAB-Map nodes."""
from pathlib import Path
import os


def auv_camera_kit(model, description):
    """Authored massless sensor frames; retain all source hull/force geometry."""
    import math
    import xml.etree.ElementTree as ET
    root=ET.fromstring(description)
    if root.find("link[@name='oakd_rgb_camera_frame']") is not None:
        raise ValueError('AUV camera kit is already present')
    for name,parent,xyz,rpy in [
        ('oakd_rgb_camera_frame','base_link','-1.17 0 .06',f'0 0 {math.pi}'),
        ('oakd_rgb_camera_optical_frame','oakd_rgb_camera_frame','0 0 0',f'{-math.pi/2} 0 {-math.pi/2}')]:
        ET.SubElement(root,'link',name=name)
        joint=ET.SubElement(root,'joint',name=name+'_mount',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
    base=model.find("link[@name='base_link']")
    sensor=ET.SubElement(base,'sensor',name='lab_rgbd',type='rgbd_camera')
    for key,text in [('pose',f'-1.17 0 .06 0 0 {math.pi}'),('topic','/marine/auv/rgbd'),
                     ('gz_frame_id','oakd_rgb_camera_optical_frame'),('update_rate','5'),('always_on','true')]:
        ET.SubElement(sensor,key).text=text
    camera=ET.SubElement(sensor,'camera')
    ET.SubElement(camera,'horizontal_fov').text='1.25'
    image=ET.SubElement(camera,'image')
    for key,text in [('width','320'),('height','240'),('format','R8G8B8')]:ET.SubElement(image,key).text=text
    for node in (ET.SubElement(camera,'clip'),ET.SubElement(ET.SubElement(camera,'depth_camera'),'clip')):
        ET.SubElement(node,'near').text='.1';ET.SubElement(node,'far').text='8.'
    return ET.tostring(root,encoding='unicode')


def ros_core_environment(package='rtabmap_slam'):
    """Use the ROS RTAB-Map core before an unrelated /usr/local build."""
    import sysconfig
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix(package));arch=sysconfig.get_config_var('MULTIARCH')
    candidates=([prefix/'lib'/arch] if arch else [])+[prefix/'lib']
    paths=[str(p) for p in candidates if p.is_dir()]
    return dict(LD_LIBRARY_PATH=os.pathsep.join(paths+[os.environ.get('LD_LIBRARY_PATH','')]))


def depth_registration_parameters(planar=False):
    """Use real depth geometry in scenes with few visual texture features."""
    return dict(subscribe_scan_cloud=True,qos_scan=2,
        **{'Reg/Strategy':'2','Reg/Force3DoF':'true' if planar else 'false',
           'RGBD/ProximityBySpace':'true','RGBD/ProximityPathMaxNeighbors':'1',
           'RGBD/NeighborLinkRefining':'true','Icp/VoxelSize':'0.1',
           'Icp/PointToPlane':'true','Icp/MaxCorrespondenceDistance':'0.5',
           'Icp/CorrespondenceRatio':'0.2'})


def depth_cloud_node(prefix):
    """Reuse RTAB-Map's calibrated depth projector, keeping capture headers."""
    from launch_ros.actions import Node
    return Node(package='rtabmap_util',executable='point_cloud_xyz',name='depth_cloud',namespace=prefix,
        parameters=[dict(use_sim_time=True,approx_sync=True,approx_sync_max_interval=.1,
            qos=2,qos_camera_info=2,decimation=4,voxel_size=.1,min_depth=.1,max_depth=8.)],
        remappings=[('depth/image',prefix+'/depth'),('depth/camera_info',prefix+'/info'),
                    ('cloud',prefix+'/points')],additional_env=ros_core_environment('rtabmap_util'),output='screen')


def pipeline(world, description, spawn, water, robot, folder, control=True,
             slam=False, database='', localization=False):
    from launch_ros.actions import Node
    from launch.actions import TimerAction
    if slam and 'oakd_rgb_camera_optical_frame' not in Path(description).read_text():
        raise ValueError('Marine SLAM requires the declared ROV camera or authored AUV RGB-D kit')
    if localization and (not slam or not database or not Path(database).is_file()):
        raise ValueError('Marine localization requires an existing RTAB-Map database and marine_slam:=true')
    if database:
        source = Path(database).expanduser().resolve()
        if not str(source).startswith('/workspace/'):
            raise ValueError('Marine map databases must stay on the workspace SSD')
    else:
        source = Path(folder) / 'marine-rtabmap.db'
    # Mapping/localization never edits a selected saved database in place.
    # A SQLite backup also includes a live database's WAL consistently.
    active = Path(folder) / 'marine-rtabmap.db'
    if source.is_file() and source != active:
        from .map_archive import load_database_copy
        load_database_copy(source, active, world,
            expected_estimator_contract='marine_spawn_imu_v1' if robot=='tethys_auv' else None)
    params = dict(use_sim_time=True, density=float(water['density']), surface_z=float(water['surface_z']))
    nodes = [Node(package='robot_lab_utils', executable='marine_sensors.py', arguments=['simulate'],
        parameters=[dict(params, world_file=str(world), seed=41)], output='screen'),
        Node(package='robot_lab_utils', executable='marine_sensors.py', arguments=['estimate'],
        parameters=[dict(params,initial_yaw=float(spawn['yaw']), **{'initial_'+k:float(spawn[k]) for k in 'xyz'})], output='screen'),
        Node(package='robot_lab_utils', executable='marine_navigation.py', parameters=[dict(use_sim_time=True,
            world_file=str(world), robot_id=robot, surface_z=float(water['surface_z']), bottom_z=float(water['bottom_z']),
            radius=1.3 if robot=='tethys_auv' else .55, database_path=str(active) if slam else '',
            control_allowed=bool(control), localization_mode=bool(localization),
            saved_map_source=str(source) if localization else '')], output='screen')]
    if slam:
        nodes += [Node(package='robot_lab_utils', executable='marine_camera_relay.py',
            parameters=[dict(use_sim_time=True, description_file=str(description))], output='screen'),
            depth_cloud_node('/marine/camera'),
            TimerAction(period=4., actions=[Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='marine/slam',
                additional_env=ros_core_environment(),
                parameters=[dict(use_sim_time=True, frame_id='marine_estimated_base_link',
                    odom_frame_id='marine_sensor_odom', map_frame_id='map', database_path=str(active),
                    subscribe_depth=True, subscribe_rgb=True, subscribe_odom_info=False, approx_sync=True,
                    approx_sync_max_interval=.15, wait_for_transform=.3, qos_image=2, qos_camera_info=2,
                    **depth_registration_parameters(),
                    **{'Mem/IncrementalMemory':'false' if localization else 'true', 'Mem/InitWMWithAllNodes':'true' if localization else 'false',
                       'Grid/3D':'true', 'Grid/Sensor':'1', 'Grid/RangeMax':'8.0',
                       'RGBD/LinearUpdate':'0.05', 'RGBD/AngularUpdate':'0.05'} )],
                remappings=[('rgb/image','/marine/camera/rgb'), ('depth/image','/marine/camera/depth'),
                    ('rgb/camera_info','/marine/camera/info'),('scan_cloud','/marine/camera/points'),
                    ('odom','/marine/estimated_odometry')], output='screen')])]
    else:
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='marine_prior_frame',
            arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0','--roll','0',
                       '--frame-id','map','--child-frame-id','marine_sensor_odom'], output='screen'))
    return nodes
