"""Explicit lab sensing for a complete vendor ground base; no chassis changes."""
import math
import xml.etree.ElementTree as ET

from .sensor_config import sensor_parameters


def summit_fortress_contacts(root):
    """Port the upstream friction frame spelling without changing its vector.

    Fortress DART reads ignition:expressed_in. Its URDF conversion lumps the
    source chassis into base_footprint, so that original direction frame is
    already the physical DART body and must remain unchanged.
    """
    mount=root.find("joint[@name='base_footprint_joint']")
    if (mount is None or mount.get('type')!='fixed' or mount.find('parent').get('link')!='base_footprint'
            or mount.find('child').get('link')!='base_link'
            or any(abs(float(v))>1e-12 for v in mount.find('origin').get('rpy','0 0 0').split())):
        raise ValueError('The reviewed source friction port requires zero relative base-frame rotation')
    ET.register_namespace('ignition','http://ignitionrobotics.org/schema')
    changed=[]
    for block in root.findall('gazebo'):
        for direction in block.findall('.//fdir1'):
            old=direction.attrib.pop('{http://gazebosim.org/schema}expressed_in',None)
            if old is None:continue
            if old!='base_footprint':raise ValueError('Unreviewed source friction frame: '+old)
            direction.set('{http://ignitionrobotics.org/schema}expressed_in',old)
            changed.append(dict(link=block.get('reference'),source_frame=old,physical_frame=old,
                                source_direction=direction.text.strip()))
    if len(changed)!=4:raise ValueError('Expected all four source mecanum friction directions')
    return changed


def ground_sensor_kit(root):
    # sdformat merges this zero-rotation fixed source chain into its parent.
    # Mount sensors explicitly there, avoiding renamed/lumped sensor paths.
    physical_parent='base_link';offset=[0.,0.,0.]
    mount=root.find("joint[@name='base_footprint_joint']")
    if mount is not None:
        if (mount.get('type')!='fixed' or mount.find('parent').get('link')!='base_footprint'
                or mount.find('child').get('link')!='base_link'
                or any(abs(float(v))>1e-12 for v in mount.find('origin').get('rpy','0 0 0').split())):
            raise ValueError('Sensor kit requires a reviewed fixed source base transform')
        physical_parent='base_footprint';offset=list(map(float,mount.find('origin').get('xyz','0 0 0').split()))
    def physical_pose(xyz):
        return ' '.join(str(a+b) for a,b in zip(map(float,xyz.split()),offset))+' 0 0 0'
    mounts=[('laser_link','base_link','0 0 .4','0 0 0'),
            ('imu_link','base_link','0 0 0','0 0 0'),
            ('oakd_rgb_camera_frame','base_link','.3585 0 .25062','0 0 0'),
            ('oakd_rgb_camera_optical_frame','oakd_rgb_camera_frame','0 0 0',f'{-math.pi/2} 0 {-math.pi/2}')]
    for name,parent,xyz,rpy in mounts:
        if root.find(f"link[@name='{name}']") is not None:raise ValueError('Sensor kit frame already exists: '+name)
        ET.SubElement(root,'link',name=name)
        joint=ET.SubElement(root,'joint',name=name+'_lab_mount',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
    block=ET.SubElement(root,'gazebo',reference=physical_parent)
    imu=ET.SubElement(block,'sensor',name='lab_imu',type='imu')
    ET.SubElement(imu,'pose').text=physical_pose('0 0 0')
    for key,text in dict(always_on='true',update_rate='50',topic='/imu',gz_frame_id='imu_link').items():ET.SubElement(imu,key).text=text
    laser=ET.SubElement(block,'sensor',name='lab_lidar',type='gpu_lidar')
    for key,text in dict(pose=physical_pose('0 0 .4'),always_on='true',update_rate='5',topic='/scan',gz_frame_id='laser_link').items():ET.SubElement(laser,key).text=text
    lidar=ET.SubElement(laser,'lidar');horizontal=ET.SubElement(ET.SubElement(lidar,'scan'),'horizontal')
    for key,text in dict(samples='640',resolution='1',min_angle=str(-math.pi),max_angle=str(math.pi)).items():ET.SubElement(horizontal,key).text=text
    limits=ET.SubElement(lidar,'range')
    for key,text in dict(min='.1',max='15',resolution='.01').items():ET.SubElement(limits,key).text=text
    sensor=ET.SubElement(block,'sensor',name='rgbd_camera',type='rgbd_camera')
    for key,text in dict(pose=physical_pose('.3585 0 .25062'),always_on='true',update_rate='5',gz_frame_id='oakd_rgb_camera_optical_frame').items():ET.SubElement(sensor,key).text=text
    camera=ET.SubElement(sensor,'camera');ET.SubElement(camera,'horizontal_fov').text='1.25'
    image=ET.SubElement(camera,'image')
    for key,text in dict(width='320',height='240',format='R8G8B8').items():ET.SubElement(image,key).text=text
    for node in (ET.SubElement(camera,'clip'),ET.SubElement(ET.SubElement(camera,'depth_camera'),'clip')):
        ET.SubElement(node,'near').text='.1';ET.SubElement(node,'far').text='8'
    for name,short in [('Imu','imu'),('Sensors','sensors')]:
        plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',name='ignition::gazebo::systems::'+name,
                            filename='ignition-gazebo-'+short+'-system')
        if name=='Sensors':ET.SubElement(plugin,'render_engine').text='ogre2'
    return sensor_parameters(dict(laser_link_name='laser_link',scan_rate=5.,scan_samples=640,
        scan_range_min=.1,scan_range_max=15.,camera_link_name='oakd_rgb_camera_frame',
        camera_optical_frame='oakd_rgb_camera_optical_frame',camera_rate=5.,camera_width=320,camera_height=240,
        camera_horizontal_fov=1.25,camera_near=.1,camera_far=8.),ET.tostring(root,encoding='unicode'))
