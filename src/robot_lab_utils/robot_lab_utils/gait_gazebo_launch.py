"""One owned Fortress plant, bounded force servos and measured gait feedback."""
import os
from pathlib import Path
import yaml


def plant_actions(description,model,source_world,folder,pose,gui,kit):
    from ament_index_python.packages import get_package_share_directory as share
    from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent,TimerAction
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node
    from .gait_gazebo_description import prepare
    plugin=Path(share('robot_lab_maps')).parents[1]/'lib/librobot_lab_gait_contacts.so'
    if not plugin.is_file():raise ValueError('Build robot_lab_maps with Fortress SDK for the native gait contact system')
    if not plugin.with_name('librobot_lab_gait_physics.so').is_file():raise ValueError('Build robot_lab_maps native contact-force serializer for this Fortress gait world')
    native=prepare(Path(description).read_text(),model,source_world,folder,share('robot_lab_robots'),kit,plugin)
    mappings=[]
    def bridge(ros,gz,ros_type,gz_type,direction='GZ_TO_ROS'):
        mappings.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=ros_type,gz_type_name=gz_type,direction=direction))
    bridge('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock')
    bridge('/gait/native_joint_states','/gait/native_joint_states','sensor_msgs/msg/JointState','gz.msgs.Model')
    bridge('/gait/native_contact_frame','/gait/native_contact_frame','std_msgs/msg/String','gz.msgs.StringMsg')
    bridge('/odom/ground_truth','/gait/native_odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry')
    bridge('/imu/out','/gait/native_imu','sensor_msgs/msg/Imu','gz.msgs.IMU')
    for name,topic in native['topics'].items():bridge(topic,topic,'std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ')
    if kit:
        bridge('/scan','/gait/native_scan','sensor_msgs/msg/LaserScan','gz.msgs.LaserScan')
        for ros,gz,kind in [('/oakd/rgb/image_raw','image','Image'),('/oakd/depth/image_raw','depth_image','Image'),('/oakd/rgb/camera_info','camera_info','CameraInfo')]:
            bridge(ros,'/gait/native_camera/'+gz,'sensor_msgs/msg/'+kind,'gz.msgs.'+kind)
    config=Path(folder)/'native-gait-bridge.yaml';config.write_text(yaml.safe_dump(mappings))
    server=ExecuteProcess(cmd=['ign','gazebo','-r','-v','2',*([] if gui else ['-s','--headless-rendering']),native['world'],'--force-version','6'],output='screen')
    adapter=Node(package='robot_lab_utils',executable='gait_gazebo_adapter.py',arguments=['--config',native['config']],parameters=[dict(use_sim_time=True)],output='screen')
    return [server,Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[dict(use_sim_time=True,robot_description=Path(description).read_text())],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[dict(use_sim_time=True,config_file=str(config))],output='screen'),adapter,
        TimerAction(period=2.,actions=[Node(package='ros_gz_sim',executable='create',arguments=['-world',native['world_name'],'-file',native['model'],'-name',model.robot_id,
            '-x',str(pose['x']),'-y',str(pose['y']),'-z',str(pose['z']),'-Y',str(pose['yaw'])],output='screen')]),
        *[RegisterEventHandler(OnProcessExit(target_action=owner,on_exit=[EmitEvent(event=Shutdown(reason='Native gait plant/adapter exited'))])) for owner in (server,adapter)]]
