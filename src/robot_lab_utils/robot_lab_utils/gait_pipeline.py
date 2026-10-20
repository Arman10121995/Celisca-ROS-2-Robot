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


def pipeline(task, description_file, config, pose, map_profile, folder, model):
    from ament_index_python.packages import get_package_share_directory as share
    from launch.actions import TimerAction
    from launch_ros.actions import Node
    from .fleet_session import navigation_parameters
    from .marine_pipeline import ros_core_environment
    if task not in ('loc','slam','3d_slam','nav'):raise ValueError('Unknown experimental gait task')
    radius=max(np.linalg.norm(model.home[leg['id']][:2]+np.asarray(leg.get('body_offset',[0.,0.]))) for leg in model.legs)+.15
    nodes=[Node(package='robot_lab_utils',executable='gait_odometry.py',parameters=[dict(use_sim_time=True,
        description_file=str(description_file),gait_config=str(config),initial_drive_yaw=float(pose['yaw'])+model.config['forward_yaw'])],output='screen'),
        Node(package='robot_lab_utils',executable='gait_sensor_relay.py',parameters=[dict(use_sim_time=True,
            description_file=str(description_file))],output='screen')]
    # A known spawn-prior transform keeps the original truth-based RobotModel
    # visible. This separate `odom` branch is never used by contact odometry,
    # sensor relays, SLAM or navigation (`gait/odom` and `gait/base_*`).
    yaw=pose['yaw']+model.config['forward_yaw']
    tx=ty=heading=0.
    if task in ('slam','3d_slam'):
        tx=-math.cos(yaw)*pose['x']-math.sin(yaw)*pose['y']
        ty=math.sin(yaw)*pose['x']-math.cos(yaw)*pose['y'];heading=-yaw
    nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name='gait_truth_visualization_prior',
        arguments=['--x',str(tx),'--y',str(ty),'--z','0','--yaw',str(heading),'--pitch','0','--roll','0',
                   '--frame-id','map','--child-frame-id','odom'],output='screen'))
    if task in ('loc','nav'):
        data=map_profile.get('map',{})
        if not data.get('has_2d_map'):raise ValueError('Experimental legged localization/navigation needs a reviewed 2D map or a saved SLAM map')
        map_yaml=str(Path(share(data.get('package','robot_lab_maps')))/data['path'])
        member=dict(id='gait',model='labbot',spawn=pose,max_linear=float(model.config['max_speed']),max_angular=float(model.config['max_yaw']),radius=float(radius))
        params=navigation_parameters(Path(share('robot_lab_navigation'))/'config',Path(share('robot_lab_localization'))/'config',member,map_yaml)['/gait']
        def rewrite(value):
            if isinstance(value,dict):return {k:rewrite(v) for k,v in value.items()}
            if isinstance(value,list):return [rewrite(v) for v in value]
            return '/gait/odometry' if value=='/gait/robot_lab_controller/odom' else value
        params=rewrite(params)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        follow.update(desired_linear_vel=float(model.config['max_speed']),min_approach_linear_velocity=.005,
                      regulated_linear_scaling_min_speed=.005,rotate_to_heading_angular_vel=float(model.config['max_yaw']))
        params['controller_server']['ros__parameters']['progress_checker'].update(required_movement_radius=.025,movement_time_allowance=45.)
        params['amcl']['ros__parameters']['initial_pose']['yaw']=pose['yaw']+model.config['forward_yaw']
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
            parameters=[params],output='screen'))
        nodes += [Node(package='nav2_map_server',executable='map_saver_server',namespace='gait',
            parameters=[dict(use_sim_time=True,save_map_timeout=5.)],output='screen'),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='gait',
                 parameters=[dict(use_sim_time=True,autostart=True,node_names=['map_saver_server'])],output='screen')]
    else:
        nodes += [Node(package='robot_lab_utils',executable='marine_camera_relay.py',parameters=[dict(use_sim_time=True,
            description_file=str(description_file),source_camera_frame='gait_camera_optical_frame',
            estimated_base_frame='gait/base_link',target_camera_frame='gait/camera_optical_frame',output_prefix='/gait/camera')],output='screen'),
            Node(package='rtabmap_slam',executable='rtabmap',namespace='gait/slam',parameters=[dict(use_sim_time=True,
                frame_id='gait/base_footprint',odom_frame_id='gait/odom',map_frame_id='map',
                database_path=str(Path(folder)/'gait-rtabmap.db'),subscribe_depth=True,subscribe_odom_info=False,
                approx_sync=True,approx_sync_max_interval=.15,qos_image=2,qos_camera_info=2,
                **{'RGBD/LinearUpdate':'0.025','RGBD/AngularUpdate':'0.025','Grid/3D':'true'})],
                remappings=[('rgb/image','/gait/camera/rgb'),('depth/image','/gait/camera/depth'),
                    ('rgb/camera_info','/gait/camera/info'),('odom','/gait/odometry')],additional_env=ros_core_environment(),output='screen')]
    return nodes
