"""Declared bench/cube task fixture, separate from vendor robot mechanics."""
import xml.etree.ElementTree as ET

NAME='mobile_manipulation_lab'
BENCHES=[('pickup_bench',[0.,-.62,.25],[.5,.32,.5]),
         ('delivery_bench',[1.2,-.62,.25],[.5,.32,.5])]
OBJECT=('pickup_cube',[0.,-.62,.515],[.03,.03,.03])


def sdf():
    root=ET.Element('sdf',version='1.7');world=ET.SubElement(root,'world',name=NAME)
    ET.SubElement(world,'gravity').text='0 0 -9.81'
    physics=ET.SubElement(world,'physics',name='lab',type='ignored')
    ET.SubElement(physics,'max_step_size').text='.002'
    ET.SubElement(physics,'real_time_factor').text='1'
    ET.SubElement(world,'plugin',filename='ignition-gazebo-physics-system',name='ignition::gazebo::systems::Physics')
    ET.SubElement(world,'plugin',filename='ignition-gazebo-sensors-system',name='ignition::gazebo::systems::Sensors')
    ET.SubElement(world,'plugin',filename='ignition-gazebo-scene-broadcaster-system',name='ignition::gazebo::systems::SceneBroadcaster')
    ET.SubElement(world,'plugin',filename='ignition-gazebo-user-commands-system',name='ignition::gazebo::systems::UserCommands')
    fixtures=[('floor',[1.,0.,-.05],[6.,4.,.1]),*BENCHES,OBJECT]
    for name,position,size in fixtures:
        model=ET.SubElement(world,'model',name=name)
        ET.SubElement(model,'static').text='false' if name==OBJECT[0] else 'true'
        ET.SubElement(model,'pose').text=' '.join(map(str,[*position,0,0,0]))
        link=ET.SubElement(model,'link',name='body')
        if name==OBJECT[0]:
            inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text='.035'
            tensor=ET.SubElement(inertial,'inertia')
            for key in ('ixx','iyy','izz'):ET.SubElement(tensor,key).text=str(.035*.03**2/6)
            for key in ('ixy','ixz','iyz'):ET.SubElement(tensor,key).text='0'
        for kind in ('visual','collision'):
            element=ET.SubElement(link,kind,name=kind)
            box=ET.SubElement(ET.SubElement(element,'geometry'),'box')
            ET.SubElement(box,'size').text=' '.join(map(str,size))
            if kind=='collision':
                ode=ET.SubElement(ET.SubElement(ET.SubElement(element,'surface'),'friction'),'ode')
                ET.SubElement(ode,'mu').text='.8';ET.SubElement(ode,'mu2').text='.8'
            else:
                ET.SubElement(ET.SubElement(element,'material'),'diffuse').text=('1 .45 .05 1' if name==OBJECT[0] else '.4 .45 .5 1')
    light=ET.SubElement(world,'light',name='sun',type='directional')
    ET.SubElement(light,'direction').text='-.3 -.3 -1'
    ET.SubElement(light,'diffuse').text='.8 .8 .8 1'
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')


def mjcf(marker_texture=None):
    root=ET.Element('mujoco',model=NAME)
    if marker_texture:
        asset=ET.SubElement(root,'asset')
        ET.SubElement(asset,'texture',name='pickup_marker_texture',type='2d',file=marker_texture)
        ET.SubElement(asset,'material',name='pickup_marker_material',texture='pickup_marker_texture',
                      texrepeat='1 1',texuniform='false',rgba='1 1 1 1',specular='0')
        h=.0144;z=.000025
        vertices=[(x,y,layer) for layer in (-z,z) for x,y in ((-h,-h),(h,-h),(h,h),(-h,h))]
        ET.SubElement(asset,'mesh',name='pickup_marker_mesh',vertex=' '.join(str(v) for p in vertices for v in p),
            texcoord='0 1 1 1 1 0 0 0 0 1 1 1 1 0 0 0',face='4 5 6 4 6 7 0 2 1 0 3 2 0 1 5 0 5 4 1 2 6 1 6 5 2 3 7 2 7 6 3 0 4 3 4 7')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 0 3',dir='0 0 -1')
    for name,position,size in [('floor',[1.,0.,-.05],[6.,4.,.1]),*BENCHES,OBJECT]:
        body=ET.SubElement(world,'body',name=name,pos=' '.join(map(str,position)))
        if name==OBJECT[0]:ET.SubElement(body,'freejoint',name=name+'_free')
        attributes=dict(type='box',name=name+'_collision',size=' '.join(str(v/2) for v in size),
                        friction='.8 .005 .0001',rgba='1 .45 .05 1' if name==OBJECT[0] else '.4 .45 .5 1')
        if name==OBJECT[0]:attributes['mass']='.035'
        ET.SubElement(body,'geom',**attributes)
        if name==OBJECT[0] and marker_texture:
            ET.SubElement(body,'geom',name='pickup_marker_visual',type='mesh',mesh='pickup_marker_mesh',
                material='pickup_marker_material',pos='0 0 .01505',mass='0',contype='0',conaffinity='0',group='2')
    ET.indent(root)
    return ET.tostring(root,encoding='unicode')


def occupancy():
    width,height,resolution=120,80,.05
    pixels=bytearray([254])*(width*height)
    for row in range(height):
        y=-2+(height-row-.5)*resolution
        for column in range(width):
            x=-2+(column+.5)*resolution
            if any(abs(x-p[0])<=s[0]/2 and abs(y-p[1])<=s[1]/2 for _,p,s in BENCHES):
                pixels[row*width+column]=0
    return ('P5\n%d %d\n255\n'%(width,height)).encode()+pixels
