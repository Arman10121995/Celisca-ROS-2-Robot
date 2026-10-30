"""Explicit experimental sensing/SLAM/Nav2 over real multi-leg actuators."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import numpy as np


def sensor_description(description, forward_yaw):
    """Add ideal lab sensor frames; preserve every source mass/collider/joint."""
    root=ET.fromstring(description)
    for name,parent,xyz,rpy in [
        ('gait_laser_link','base_link','0 0 .12',f'0 0 {forward_yaw}'),
        ('gait_camera_link','base_link','0 0 .14',f'0 0 {forward_yaw}'),
        ('gait_camera_optical_frame','gait_camera_link','0 0 0',f'{-math.pi/2} 0 {-math.pi/2}')]:
        if root.find("link[@name='%s']"%name) is not None:raise ValueError('Sensor kit frame already exists: '+name)
        ET.SubElement(root,'link',name=name)
        joint=ET.SubElement(root,'joint',name=name+'_mount',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
    return ET.tostring(root,encoding='unicode')


def pipeline(task, description_file, config, pose, map_profile, folder, model,
             map_yaml='', database='', localization=False, world_file=''):
    from ament_index_python.packages import get_package_share_directory as share
    from launch.actions import TimerAction
    from launch_ros.actions import Node
    from .fleet_session import navigation_parameters
    from .marine_pipeline import ros_core_environment, depth_registration_parameters, depth_cloud_node
    from .map_archive import load_database_copy
    if task not in ('loc','slam','3d_slam','nav'):raise ValueError('Unknown experimental gait task')
    serpentine=model.config.get('kind')=='serpentine'
    radius=model.footprint_radius if serpentine else max(np.linalg.norm(model.home[leg['id']][:2]+np.asarray(leg.get('body_offset',[0.,0.]))) for leg in model.legs)+.15
    nodes=[Node(package='robot_lab_utils',executable='serpentine_odometry.py' if serpentine else 'gait_odometry.py',parameters=[dict(use_sim_time=True,
        description_file=str(description_file),gait_config=str(config),initial_drive_yaw=float(pose['yaw'])+model.config['forward_yaw'])],output='screen'),
        Node(package='robot_lab_utils',executable='gait_sensor_relay.py',parameters=[dict(use_sim_time=True,
            virtual_drive_scan=serpentine,base_height=float(model.config['standing_height']),
            description_file=str(description_file),visual_spawn_x=float(pose['x']),visual_spawn_y=float(pose['y']),
            visual_spawn_yaw=float(pose['yaw'])+model.config['forward_yaw'])],output='screen')]
    if task in ('loc','nav'):
        data=map_profile.get('map',{})
        saved_map=bool(map_yaml)
        if not saved_map and not data.get('has_2d_map'):raise ValueError('Legged localization/navigation needs a reviewed 2D map or gait_map_yaml:=<saved map>')
        map_yaml=str(Path(map_yaml).expanduser().resolve()) if saved_map else str(Path(share(data.get('package','robot_lab_maps')))/data['path'])
        if not Path(map_yaml).is_file():raise ValueError('Saved occupancy YAML is unavailable: '+map_yaml)
        from .fleet_session import SpawnGrid
        SpawnGrid(map_yaml)  # Validate actual image, resolution and occupancy.
        initial=dict(pose)
        if saved_map:
            import hashlib,json
            metadata=Path(map_yaml).parent/'manifest.json'
            recorded=json.loads(metadata.read_text()) if metadata.is_file() else {}
            if recorded.get('world_sha256') and world_file and recorded['world_sha256']!=hashlib.sha256(Path(world_file).read_bytes()).hexdigest():
                raise ValueError('Saved gait map belongs to different source world geometry')
            x,y,heading=recorded.get('initial_pose_in_map') or [0.,0.,0.]
            initial.update(x=x,y=y,yaw=heading-model.config['forward_yaw'])
        member=dict(id='gait',model='labbot',spawn=initial,max_linear=float(model.config['max_speed']),max_angular=float(model.config['max_yaw']),radius=float(radius))
        params=navigation_parameters(Path(share('robot_lab_navigation'))/'config',Path(share('robot_lab_localization'))/'config',member,map_yaml)['/gait']
        def rewrite(value):
            if isinstance(value,dict):return {k:rewrite(v) for k,v in value.items()}
            if isinstance(value,list):return [rewrite(v) for v in value]
            if value=='/map':return '/gait/map'
            return '/gait/odometry' if value=='/gait/robot_lab_controller/odom' else value
        params=rewrite(params)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        follow.update(desired_linear_vel=float(model.config['max_speed']),min_approach_linear_velocity=.005,
                      regulated_linear_scaling_min_speed=.005,rotate_to_heading_angular_vel=float(model.config['max_yaw']),
                      lookahead_dist=.2,min_lookahead_dist=.1,max_lookahead_dist=.4)
        params['controller_server']['ros__parameters']['general_goal_checker'].update(xy_goal_tolerance=.1,yaw_goal_tolerance=.15)
        params['controller_server']['ros__parameters']['progress_checker'].update(required_movement_radius=.025,movement_time_allowance=45.)
        if serpentine:
            # A passive-wheel articulated body cannot pivot like a differential
            # base. Plan curvature-bounded paths and keep the local controller
            # from requesting stationary heading rotations.
            params['planner_server']['ros__parameters']['GridBased'].update(
                plugin='nav2_smac_planner/SmacPlannerHybrid',motion_model_for_search='REEDS_SHEPP',
                minimum_turning_radius=2.,angle_quantization_bins=72,allow_unknown=False)
            follow.update(use_rotate_to_heading=False,allow_reversing=True,lookahead_dist=.6,min_lookahead_dist=.4,max_lookahead_dist=1.)
            params['controller_server']['ros__parameters']['general_goal_checker'].update(xy_goal_tolerance=.04,yaw_goal_tolerance=.08)
            local=params['local_costmap']['local_costmap']['ros__parameters']
            # The complete head-centred envelope must fit inside its rolling
            # window. The differential-base 3 m window clipped this 3.5 m
            # diameter and made RPP reject every command as a collision.
            extent=int(math.ceil(2*radius+2.))
            local.update(width=max(extent,int(local['width'])),height=max(extent,int(local['height'])))
            # The head has real lateral motion during a spine wave even
            # though strafe is not an actuator command. A differential-drive
            # particle motion model discarded that encoder component.
            params['amcl']['ros__parameters'].update(robot_model_type='nav2_amcl::OmniMotionModel',
                alpha1=.01,alpha2=.01,alpha3=.03,alpha4=.01,alpha5=.03)
        params['amcl']['ros__parameters']['initial_pose']['yaw']=initial['yaw']+model.config['forward_yaw']
        params['amcl']['ros__parameters'].update(update_min_d=.01,update_min_a=.02)
        for costmap in ('local_costmap','global_costmap'):
            params[costmap][costmap]['ros__parameters']['inflation_layer']['inflation_radius']=float(radius+.15)
        file=Path(folder)/'gait-navigation.yaml';file.write_text(yaml.safe_dump({'/gait':params}))
        servers=[('nav2_amcl','amcl')]
        if task=='nav':servers += [('nav2_controller','controller_server'),('nav2_planner','planner_server'),
            ('nav2_smoother','smoother_server'),('nav2_bt_navigator','bt_navigator'),('nav2_behaviors','behavior_server')]
        nodes += [Node(package='nav2_map_server',executable='map_server',namespace='gait',
            parameters=[dict(use_sim_time=True,yaml_filename=map_yaml)],output='screen'),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='map_lifecycle',namespace='gait',
                 parameters=[dict(use_sim_time=True,autostart=True,node_names=['map_server'])],output='screen')]
        nodes.append(TimerAction(period=7.,actions=[*[Node(package=package,executable=name,name=name,namespace='gait',
            parameters=[str(file)],remappings=[('tf','/tf'),('tf_static','/tf_static')],output='screen') for package,name in servers],
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='gait',
                 parameters=[dict(use_sim_time=True,autostart=True,node_names=[name for _,name in servers])],output='screen')]))
    elif task=='slam':
        params=yaml.safe_load((Path(share('robot_lab_mapping'))/'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
        params.update(odom_frame='gait/odom',base_frame='gait/base_footprint',scan_topic='/gait/scan',
                      minimum_travel_distance=.025,minimum_travel_heading=.025)
        nodes.append(Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',namespace='gait',
            parameters=[params],remappings=[('/map','/gait/map'),('/map_metadata','/gait/map_metadata')],output='screen'))
        nodes += [Node(package='nav2_map_server',executable='map_saver_server',name='map_saver',namespace='gait',
            parameters=[dict(use_sim_time=True,save_map_timeout=5.)],output='screen'),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='map_saver_lifecycle',namespace='gait',
                 parameters=[dict(use_sim_time=True,autostart=True,node_names=['map_saver'])],output='screen')]
    else:
        active=Path(folder)/'gait-rtabmap.db'
        if localization and (not database or not Path(database).is_file()):
            raise ValueError('Gait 3D localization requires a saved acquired RTAB-Map database')
        if database:
            load_database_copy(database,active,world_file)
        nodes += [Node(package='robot_lab_utils',executable='marine_camera_relay.py',parameters=[dict(use_sim_time=True,
            description_file=str(description_file),source_camera_frame='gait_camera_optical_frame',
            estimated_base_frame='gait/base_link',target_camera_frame='gait/camera_optical_frame',output_prefix='/gait/camera')],output='screen'),
            depth_cloud_node('/gait/camera'),
            Node(package='rtabmap_slam',executable='rtabmap',namespace='gait/slam',parameters=[dict(use_sim_time=True,
                frame_id='gait/base_link' if serpentine else 'gait/base_footprint',odom_frame_id='gait/odom',map_frame_id='map',
                database_path=str(active),subscribe_depth=True,subscribe_rgb=True,subscribe_odom_info=False,
                approx_sync=True,approx_sync_max_interval=.15,qos_image=2,qos_camera_info=2,
                **depth_registration_parameters(planar=True),
                **{'RGBD/LinearUpdate':'0.025','RGBD/AngularUpdate':'0.025','Grid/3D':'true',
                   'Mem/IncrementalMemory':'false' if localization else 'true',
                   'Mem/InitWMWithAllNodes':'true' if localization else 'false'})],
                remappings=[('rgb/image','/gait/camera/rgb'),('depth/image','/gait/camera/depth'),
                    ('rgb/camera_info','/gait/camera/info'),('scan_cloud','/gait/camera/points'),
                    ('odom','/gait/odometry')],additional_env=ros_core_environment(),output='screen')]
    if task in ('slam','3d_slam'):
        nodes.append(Node(package='robot_lab_utils',executable='map_archive.py',parameters=[dict(use_sim_time=True,
            task=task,robot_id=model.config.get('robot_id',''),world_file=str(world_file),
            session_directory=str(folder),database_path=str(Path(folder)/'gait-rtabmap.db'))],output='screen'))
    return nodes
