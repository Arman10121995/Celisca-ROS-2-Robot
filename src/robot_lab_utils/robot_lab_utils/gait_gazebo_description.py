"""Force-driven Fortress gait plant with an exact source joint-reference adapter."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def align_initial(description,model,world_file,pose,reference):
    """Source FK and selected static colliders place the initial stance only."""
    import PyKDL as kdl
    from .gait_terrain import StaticTerrainRay
    ray=StaticTerrainRay(world_file);tree=ET.fromstring(description)
    values=dict(zip(model.names,model.initial));rotation=Rotation.from_euler('z',pose['yaw']).as_matrix()
    lifts=[]
    for leg in model.legs:
        chain=model.tree.getChain('base_link',leg['foot'])
        names=[chain.getSegment(i).getJoint().getName() for i in range(chain.getNrOfSegments()) if chain.getSegment(i).getJoint().getType()!=kdl.Joint.Fixed]
        solver=kdl.ChainFkSolverPos_recursive(chain);frame=kdl.Frame()
        if solver.JntToCart(model.array([values[name] for name in names]),frame)!=0:raise ValueError('Source startup FK failed')
        sphere=tree.find("link[@name='%s']/collision/geometry/sphere"%leg['foot'])
        if sphere is None:raise ValueError('Initial support needs source spherical sole')
        position=np.array([pose['x'],pose['y'],pose['z']])+rotation@np.array([frame.p.x(),frame.p.y(),frame.p.z()])
        origin=position.copy();origin[2]=float(reference)+.6
        distance=ray(origin,1.2)
        if distance is None:raise ValueError('No selected-world initial source foot support')
        lifts.append(origin[2]-distance-position[2]+float(sphere.get('radius'))+.002)
    if max(lifts)-min(lifts)>.04:raise ValueError('Initial source stance needs flatter support')
    pose['z']+=max(lifts)
    return dict(placement='Initial source FK/static collision support only',lift_m=max(lifts),support_spread_m=max(lifts)-min(lifts))


def sub(parent,tag,text=None,**attributes):
    element=ET.SubElement(parent,tag,**attributes)
    if text is not None:element.text=str(text)
    return element


def shifted_description(description,model):
    """Move each native encoder zero to source stance; geometry stays equivalent.

    Source q = native q + initial. Joint transforms, limits and reporting use
    that same change of coordinates; source assets and RSP remain unchanged.
    """
    root=ET.fromstring(description)
    for tag in list(root):
        if tag.tag in ('ros2_control','transmission','mujoco'):root.remove(tag)
        elif tag.tag=='gazebo':
            for plugin in list(tag.findall('plugin')):tag.remove(plugin)
    for name,offset in zip(model.names,model.initial):
        joint=root.find("joint[@name='%s']"%name)
        if joint is None or joint.get('type') not in ('revolute','continuous'):raise ValueError('Native gait requires source revolute joints')
        origin=joint.find('origin')
        if origin is None:origin=sub(joint,'origin',xyz='0 0 0',rpy='0 0 0')
        angles=[float(v) for v in origin.get('rpy','0 0 0').split()]
        axis=np.asarray([float(v) for v in joint.find('axis').get('xyz').split()]);axis/=np.linalg.norm(axis)
        angles=(Rotation.from_euler('xyz',angles)*Rotation.from_rotvec(axis*float(offset))).as_euler('xyz')
        origin.set('rpy',' '.join(format(v,'.17g') for v in angles))
        limit=joint.find('limit')
        for key in ('lower','upper'):
            if limit is not None and limit.get(key) is not None:limit.set(key,format(float(limit.get(key))-float(offset),'.17g'))
    feet={leg['foot'] for leg in model.legs}
    for joint in root.findall('joint'):
        if joint.get('type')=='fixed' and joint.find('child').get('link') in feet:
            sub(sub(root,'gazebo',reference=joint.get('name')),'preserveFixedJoint','true')
    return ET.tostring(root,encoding='unicode')


def prepare(description,model,world_file,folder,robot_share,kit,contact_plugin):
    folder=Path(folder)
    shifted=shifted_description(description,model).replace('package://robot_lab_robots/',str(robot_share)+'/')
    urdf=folder/'native-zero-reference.urdf';urdf.write_text(shifted)
    converted=subprocess.run(['ign','sdf','-p',str(urdf)],capture_output=True,text=True,timeout=45,check=True)
    native=ET.fromstring(converted.stdout);body=native.find('model')
    if body is None:raise ValueError('Fortress source URDF conversion returned no model')
    body.set('name',model.robot_id)
    names=list(model.names);topics={name:'/gait/native_target/'+name for name in names}
    for i,name in enumerate(names):
        plugin=sub(body,'plugin',name='ignition::gazebo::systems::JointPositionController',filename='ignition-gazebo-joint-position-controller-system')
        for key,value in dict(joint_name=name,topic=topics[name],p_gain=min(80.,float(model.config['kp'])),
            d_gain=min(.1,float(model.config['kd'])),i_gain=0.,cmd_min=-float(model.torque[i]),cmd_max=float(model.torque[i]),
            use_velocity_commands='false',initial_position=0.).items():sub(plugin,key,value)
    plugin=sub(body,'plugin',name='ignition::gazebo::systems::JointStatePublisher',filename='ignition-gazebo-joint-state-publisher-system')
    sub(plugin,'topic','/gait/native_joint_states')
    for name in names:sub(plugin,'joint_name',name)
    plugin=sub(body,'plugin',name='ignition::gazebo::systems::OdometryPublisher',filename='ignition-gazebo-odometry-publisher-system')
    for key,value in dict(odom_frame='odom',robot_base_frame='base_link',dimensions=3,odom_publish_frequency=50,odom_topic='/gait/native_odometry').items():sub(plugin,key,value)
    channels={}
    contact_frame=sub(body,'plugin',name='robot_lab::GaitContactFrame',filename=str(contact_plugin))
    observed=[(leg['foot'],leg['id']) for leg in model.legs]+[(link.get('name'),'body') for link in body.findall('link') if link.get('name')=='base_link' or link.get('name','').startswith('segment_')]
    for index,(name,group) in enumerate(observed):
        link=body.find("link[@name='%s']"%name)
        if link is None:raise ValueError('Native source contact link was lost: '+name)
        channels[name]=group
    for group in set(channels.values()):
        entry=sub(contact_frame,'group',name=group)
        for name,assigned in channels.items():
            if assigned==group:sub(entry,'link',name)
    base=body.find("link[@name='base_link']")
    sensor=sub(base,'sensor',name='source_imu',type='imu');sub(sensor,'always_on','true');sub(sensor,'update_rate',100)
    sub(sensor,'topic','/gait/native_imu');sub(sensor,'gz_frame_id','base_link')
    if kit:
        laser=body.find("link[@name='gait_laser_link']")
        laser_offset=laser is None
        if laser is None:laser=base
        sensor=sub(laser,'sensor',name='source_laser',type='gpu_lidar');sub(sensor,'topic','/gait/native_scan');sub(sensor,'gz_frame_id','gait_laser_link');sub(sensor,'update_rate',10)
        if laser_offset:sub(sensor,'pose',f'0 0 .12 0 0 {model.config["forward_yaw"]}')
        ray=sub(sensor,'ray');h=sub(sub(ray,'scan'),'horizontal')
        for key,value in dict(samples=360,resolution=1,min_angle=-np.pi,max_angle=np.pi).items():sub(h,key,value)
        bounds=sub(ray,'range')
        for key,value in dict(min=.05,max=12.,resolution=.01).items():sub(bounds,key,value)
        camera=body.find("link[@name='gait_camera_link']")
        camera_offset=camera is None
        if camera is None:camera=base
        if camera is not None:
            sensor=sub(camera,'sensor',name='source_rgbd',type='rgbd_camera');sub(sensor,'topic','/gait/native_camera');sub(sensor,'gz_frame_id','gait_camera_optical_frame');sub(sensor,'update_rate',5)
            if camera_offset:sub(sensor,'pose',f'0 0 .14 0 0 {model.config["forward_yaw"]}')
            c=sub(sensor,'camera');sub(c,'horizontal_fov',1.2);image=sub(c,'image');sub(image,'width',160);sub(image,'height',120);sub(image,'format','R8G8B8')
            clip=sub(c,'clip');sub(clip,'near',.05);sub(clip,'far',8.)
    model_file=folder/'native-gait.sdf';ET.ElementTree(native).write(model_file,encoding='utf-8',xml_declaration=True)
    world=ET.parse(world_file);w=world.getroot().find('world')
    physics=w.find('physics')
    if physics is None:physics=sub(w,'physics',name='source_gait_physics',type='ode')
    timestep=physics.find('max_step_size')
    if timestep is None:timestep=sub(physics,'max_step_size')
    timestep.text='.001'
    present={p.get('name','').split('::')[-1] for p in w.findall('plugin')}
    for system,short in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Sensors','sensors'),('Imu','imu'),('Contact','contact')]:
        if system not in present:
            p=sub(w,'plugin',name='ignition::gazebo::systems::'+system,filename='ignition-gazebo-'+short+'-system')
            if system=='Sensors':sub(p,'render_engine','ogre2')
    for plugin in w.findall('plugin'):
        if plugin.get('name','').endswith('::Physics'):
            plugin.set('filename',str(Path(contact_plugin).with_name('librobot_lab_gait_physics.so')))
    for element in w.iter():
        if element.tag in ('uri','albedo_map','normal_map','roughness_map') and element.text:
            uri=element.text.strip()
            if '://' not in uri and not Path(uri).is_absolute():element.text=str((Path(world_file).parent/uri).resolve())
    output=folder/'native-gait-world.sdf';world.write(output,encoding='utf-8',xml_declaration=True)
    config=dict(robot=model.robot_id,description=str(folder/'robot.urdf'),gait_model=model.config,
        offsets=dict(zip(names,map(float,model.initial))),topics=topics,contacts=channels,source_world=str(world_file),
        probes={leg['id']:dict(root=leg['root'],foot=leg['foot'],home=model.home[leg['id']].tolist()) for leg in model.legs},
        limits={name:[float(model.lower[i]),float(model.upper[i])] for i,name in enumerate(names)})
    config_file=folder/'native-gait-adapter.json';config_file.write_text(json.dumps(config,indent=2)+'\n')
    return dict(world=str(output),model=str(model_file),world_name=w.get('name'),config=str(config_file),contacts=channels,topics=topics)
