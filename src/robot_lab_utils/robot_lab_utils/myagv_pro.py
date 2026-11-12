"""Explicit simulation adapter for the official myAGV Pro CAD description.

Source wheel labels are not geometric corner labels. Vendor Gazebo files also
use mismatched controller names and a 0.05m radius for ~0.076m CAD wheels.
The lab contact/actuator/sensor approximation is declared, never vendor fidelity.
"""
import math
import xml.etree.ElementTree as ET

from .sensor_config import sensor_parameters


ROLES = dict(front_left='left_front_wheel', front_right='left_rear_wheel',
             rear_left='right_front_wheel', rear_right='right_rear_wheel')


def adapt_source(root):
    """Keep original visuals, body inertias and frames; add declared lab physics."""
    ET.register_namespace('ignition', 'http://ignitionrobotics.org/schema')
    protected = {}
    for role, name in ROLES.items():
        joint = root.find(f"joint[@name='{name}_joint']")
        link = root.find(f"link[@name='{name}_link']")
        if joint is None or link is None or joint.get('type') != 'continuous':
            raise ValueError('Missing source wheel assembly: '+name)
        xyz = list(map(float, joint.find('origin').get('xyz').split()))
        if ((xyz[0] > 0) != role.startswith('front')
                or (xyz[1] > 0) != role.endswith('left')
                or joint.find('axis').get('xyz') != '0 1 0'):
            raise ValueError('Source wheel frame/corner contract differs: '+name)
        if joint.find('limit') is not None:
            raise ValueError('Reconcile new vendor limits before changing the adapter')
        # Conservative lab simulation envelope; not a hardware motor rating.
        ET.SubElement(joint, 'limit', effort='5', velocity='8')
        ET.SubElement(joint, 'dynamics', damping='.01', friction='0')
        protected[name] = dict(origin=dict(joint.find('origin').attrib), source_mass_kg=float(link.find('inertial/mass').get('value')))
        # Vendor exports rigid roller meshes. Approximate their directional
        # contact with a cylindrical envelope and chassis-frame anisotropy.
        # Original visual CAD and complete wheel inertias remain unchanged.
        for collision in list(link.findall('collision')):
            link.remove(collision)
        # URDF's unnamed collision gets <link>_collision in sdformat. The
        # gazebo collision-extension block below must address that same name.
        collision = ET.SubElement(link, 'collision')
        center = -.02317416 if xyz[1] > 0 else .02317416
        ET.SubElement(collision, 'origin', xyz=f'0 {center} 0', rpy=f'{math.pi/2} 0 0')
        geometry=ET.SubElement(collision,'geometry')
        ET.SubElement(geometry,'cylinder',radius='.0762',length='.04634832')
        block = ET.SubElement(root, 'gazebo', reference=name+'_link')
        surface = ET.SubElement(ET.SubElement(block, 'collision'), 'surface')
        ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
        ET.SubElement(ode, 'mu').text = '1'
        ET.SubElement(ode, 'mu2').text = '0'
        direction = ET.SubElement(ode, 'fdir1', {'{http://ignitionrobotics.org/schema}expressed_in':'base_footprint'})
        direction.text = '1 -1 0' if role in ('front_left','rear_right') else '1 1 0'
    return protected


def lab_compliance(root):
    """Declared small tire/suspension surrogate; original frames match at rest.

    Rigid four-point support can load just one diagonal in DART, making two
    independent mecanum translation axes physically unavailable. Passive
    bounded vertical springs distribute contact, without commanding the base.
    These are lab simulation parameters, not claimed vendor suspension.
    """
    added=[]
    for name in ROLES.values():
        wheel=root.find(f"joint[@name='{name}_joint']")
        origin=wheel.find('origin')
        carrier=name+'_lab_carrier'
        link=ET.SubElement(root,'link',name=carrier)
        inertial=ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'mass',value='.005')
        ET.SubElement(inertial,'inertia',ixx='.000001',iyy='.000001',izz='.000001',ixy='0',ixz='0',iyz='0')
        joint=ET.SubElement(root,'joint',name=carrier+'_joint',type='prismatic')
        ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link=carrier)
        ET.SubElement(joint,'origin',**origin.attrib)
        ET.SubElement(joint,'axis',xyz='0 0 1')
        ET.SubElement(joint,'limit',lower='-.012',upper='.012',effort='100',velocity='1')
        ET.SubElement(joint,'dynamics',damping='100',friction='0')
        block=ET.SubElement(root,'gazebo',reference=carrier+'_joint')
        ET.SubElement(block,'springStiffness').text='10000'
        ET.SubElement(block,'springReference').text='0'
        wheel.find('parent').set('link',carrier)
        origin.set('xyz','0 0 0');origin.set('rpy','0 0 0')
        added.append(carrier+'_joint')
    return dict(joints=added,travel_m=[-.012,.012],stiffness_N_per_m=10000.,damping_Ns_per_m=100.,
                added_carrier_mass_kg=.02,scope='Lab tire-compliance surrogate, not vendor suspension')


def sensor_kit(root):
    """Native simulated instruments at vendor frames; no hardware driver launch."""
    base = root.find("joint[@name='base_joint']/origin")
    if base is None or base.get('rpy') != '0 0 0':
        raise ValueError('Source base mount differs')
    offset = list(map(float, base.get('xyz').split()))
    def source_pose(joint_name):
        origin = root.find(f"joint[@name='{joint_name}']/origin")
        if origin is None:
            raise ValueError('Missing original sensor frame: '+joint_name)
        return ' '.join(str(a+b) for a,b in zip(offset,map(float,origin.get('xyz').split())))+' '+origin.get('rpy','0 0 0')
    ET.SubElement(root,'link',name='oakd_rgb_camera_frame')
    j=ET.SubElement(root,'joint',name='lab_camera_frame',type='fixed')
    ET.SubElement(j,'parent',link='camera_link');ET.SubElement(j,'child',link='oakd_rgb_camera_frame')
    ET.SubElement(root,'link',name='oakd_rgb_camera_optical_frame')
    j=ET.SubElement(root,'joint',name='lab_camera_optical_frame',type='fixed')
    ET.SubElement(j,'parent',link='oakd_rgb_camera_frame');ET.SubElement(j,'child',link='oakd_rgb_camera_optical_frame')
    ET.SubElement(j,'origin',xyz='0 0 0',rpy=f'{-math.pi/2} 0 {-math.pi/2}')
    block=ET.SubElement(root,'gazebo',reference='base_footprint')
    imu=ET.SubElement(block,'sensor',name='lab_imu',type='imu')
    for k,v in dict(pose=source_pose('imu_joint'),always_on='true',update_rate='50',topic='/imu',gz_frame_id='imu_link').items():ET.SubElement(imu,k).text=v
    laser=ET.SubElement(block,'sensor',name='lab_lidar',type='gpu_lidar')
    for k,v in dict(pose=source_pose('lidar_joint'),always_on='true',update_rate='5',topic='/scan',gz_frame_id='laser_link').items():ET.SubElement(laser,k).text=v
    lidar=ET.SubElement(laser,'lidar');horizontal=ET.SubElement(ET.SubElement(lidar,'scan'),'horizontal')
    for k,v in dict(samples='640',resolution='1',min_angle=str(-math.pi),max_angle=str(math.pi)).items():ET.SubElement(horizontal,k).text=v
    limits=ET.SubElement(lidar,'range')
    for k,v in dict(min='.1',max='15',resolution='.01').items():ET.SubElement(limits,k).text=v
    camera=ET.SubElement(block,'sensor',name='rgbd_camera',type='rgbd_camera')
    for k,v in dict(pose=source_pose('camera_joint'),always_on='true',update_rate='5',gz_frame_id='oakd_rgb_camera_optical_frame').items():ET.SubElement(camera,k).text=v
    optics=ET.SubElement(camera,'camera');ET.SubElement(optics,'horizontal_fov').text='1.25'
    image=ET.SubElement(optics,'image')
    for k,v in dict(width='320',height='240',format='R8G8B8').items():ET.SubElement(image,k).text=v
    for clip in (ET.SubElement(optics,'clip'), ET.SubElement(ET.SubElement(optics,'depth_camera'),'clip')):
        ET.SubElement(clip,'near').text='.1';ET.SubElement(clip,'far').text='8'
    for name,short in [('Imu','imu'),('Sensors','sensors')]:
        plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',name='ignition::gazebo::systems::'+name,
                            filename='ignition-gazebo-'+short+'-system')
        if name=='Sensors':ET.SubElement(plugin,'render_engine').text='ogre2'
    return sensor_parameters(dict(laser_link_name='laser_link',scan_samples=640,scan_rate=5.,scan_range_min=.1,
        scan_range_max=15.,camera_link_name='oakd_rgb_camera_frame',camera_optical_frame='oakd_rgb_camera_optical_frame',
        camera_width=320,camera_height=240,camera_rate=5.,camera_horizontal_fov=1.25,camera_near=.1,camera_far=8.))
