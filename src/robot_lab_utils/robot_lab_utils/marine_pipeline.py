"""Compose real marine sensor, estimation, mission and optional RTAB-Map nodes."""
from pathlib import Path
import os


def ros_core_environment(package='rtabmap_slam'):
    """Use the ROS RTAB-Map core before an unrelated /usr/local build."""
    import sysconfig
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix(package));arch=sysconfig.get_config_var('MULTIARCH')
    candidates=([prefix/'lib'/arch] if arch else [])+[prefix/'lib']
    paths=[str(p) for p in candidates if p.is_dir()]
    return dict(LD_LIBRARY_PATH=os.pathsep.join(paths+[os.environ.get('LD_LIBRARY_PATH','')]))


def pipeline(world, description, spawn, water, robot, folder, control=True,
             slam=False, database='', localization=False):
    from launch_ros.actions import Node
    from launch.actions import TimerAction
    if slam and robot != 'lab_rov':
        raise ValueError('RGB-D marine SLAM currently requires the ROV camera; source Tethys has no camera')
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
        import sqlite3
        with sqlite3.connect('file:' + str(source) + '?mode=ro', uri=True) as old:
            with sqlite3.connect(active) as new:
                old.backup(new)
    params = dict(use_sim_time=True, density=float(water['density']), surface_z=float(water['surface_z']))
    nodes = [Node(package='robot_lab_utils', executable='marine_sensors.py', arguments=['simulate'],
        parameters=[dict(params, world_file=str(world), seed=41)], output='screen'),
        Node(package='robot_lab_utils', executable='marine_sensors.py', arguments=['estimate'],
        parameters=[dict(params, **{'initial_'+k:float(spawn[k]) for k in 'xyz'})], output='screen'),
        Node(package='robot_lab_utils', executable='marine_navigation.py', parameters=[dict(use_sim_time=True,
            world_file=str(world), robot_id=robot, surface_z=float(water['surface_z']), bottom_z=float(water['bottom_z']),
            radius=1.3 if robot=='tethys_auv' else .55, database_path=str(active) if slam else '',
            control_allowed=bool(control))], output='screen')]
    if slam:
        nodes += [Node(package='robot_lab_utils', executable='marine_camera_relay.py',
            parameters=[dict(use_sim_time=True, description_file=str(description))], output='screen'),
            TimerAction(period=4., actions=[Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='marine/slam',
                additional_env=ros_core_environment(),
                parameters=[dict(use_sim_time=True, frame_id='marine_estimated_base_link',
                    odom_frame_id='marine_sensor_odom', map_frame_id='map', database_path=str(active),
                    subscribe_depth=True, subscribe_rgb=True, subscribe_odom_info=False, approx_sync=True,
                    approx_sync_max_interval=.15, wait_for_transform=.3, qos_image=2, qos_camera_info=2,
                    **{'Mem/IncrementalMemory':'false' if localization else 'true', 'Mem/InitWMWithAllNodes':'true' if localization else 'false',
                       'RGBD/NeighborLinkRefining':'true', 'Grid/3D':'true', 'Grid/Sensor':'1', 'Grid/RangeMax':'8.0',
                       'RGBD/LinearUpdate':'0.05', 'RGBD/AngularUpdate':'0.05'} )],
                remappings=[('rgb/image','/marine/camera/rgb'), ('depth/image','/marine/camera/depth'),
                    ('rgb/camera_info','/marine/camera/info'), ('odom','/marine/estimated_odometry')], output='screen')])]
    else:
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='marine_prior_frame',
            arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0','--roll','0',
                       '--frame-id','map','--child-frame-id','marine_sensor_odom'], output='screen'))
    return nodes
