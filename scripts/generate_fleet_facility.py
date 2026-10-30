#!/usr/bin/env python3
"""First-party physical doors, lift and charging locations, not mission results."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from PIL import Image,ImageDraw
from generate_marine_assets import ROOT,child,model,write


def generate():
    doc=ET.Element('sdf',version='1.7');w=child(doc,'world',name='fleet_facility')
    physics=child(w,'physics',name='physics',type='ode');child(physics,'max_step_size','.002');child(physics,'real_time_factor','1')
    child(w,'gravity','0 0 -9.81')
    for system,short in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Sensors','sensors'),('Imu','imu')]:
        plugin=child(w,'plugin',name='ignition::gazebo::systems::'+system,filename='ignition-gazebo-'+short+'-system')
        if system=='Sensors':child(plugin,'render_engine','ogre2')
    light=child(w,'light',name='sun',type='directional');child(light,'pose','0 0 10 0 0 0');child(light,'diffuse','.9 .9 .9 1');child(light,'direction','-.3 .2 -1')
    # A real shaft hole under the 2 x 2 m platform; no coincident fixed floor.
    for name,pose,size in [('west_floor',(-1.5,0,-.15),(9,10,.3)),('east_floor',(5.5,0,-.15),(1,10,.3)),
            ('north_floor',(4,3,-.15),(2,4,.3)),('south_floor',(4,-3,-.15),(2,4,.3))]:model(w,name,pose,size,'.55 .55 .55 1')
    walls=[((-6,0,1.25),(.2,10,2.5)),((6,0,1.25),(.2,10,2.5)),((0,5,1.25),(12,.2,2.5)),((0,-5,1.25),(12,.2,2.5)),
           ((0,-2.9,1.25),(.15,4.2,2.5)),((0,2.9,1.25),(.15,4.2,2.5))]
    for i,(pose,size) in enumerate(walls):model(w,'wall_'+str(i),pose,size,'.65 .7 .75 1')
    # Upper landing is physical; separate floor navigation maps remain required.
    model(w,'upper_landing',(2,0,2.85),(2,3,.3),'.35 .6 .7 1')
    hardware=child(w,'model',name='facility');child(hardware,'static','false')
    anchor=child(hardware,'link',name='anchor');inertial=child(anchor,'inertial');child(inertial,'mass','1');child(inertial,'inertia');
    fixed=child(hardware,'joint',name='anchor_fixed',type='fixed');child(fixed,'parent','world');child(fixed,'child','anchor')
    definitions=[('main_door','door_main',(.16,0,1.1),(.08,1.6,2.2),'0 1 0',1.8,12.,300.,300.,80.),
                 ('lift_door','door_lift',(3.08,0,1.1),(.08,2.,2.2),'0 1 0',2.2,15.,300.,300.,80.),
                 ('lift_platform','lift_car',(4,0,-.05),(2,2,.1),'0 0 1',3.,50.,1600.,40000.,3000.)]
    joints=[]
    for link_name,name,pose,size,axis,maximum,mass,limit,kp,kd in definitions:
        link=child(hardware,'link',name=link_name);child(link,'pose',' '.join(map(str,pose))+' 0 0 0')
        inertial=child(link,'inertial');child(inertial,'mass',mass);tensor=child(inertial,'inertia')
        for key,a,b in [('ixx',size[1],size[2]),('iyy',size[0],size[2]),('izz',size[0],size[1])]:child(tensor,key,mass*(a*a+b*b)/12)
        for kind in ('collision','visual'):
            part=child(link,kind,name=link_name+'_'+kind);child(child(part,'geometry'),'box');part.find('geometry/box').append(ET.Element('size'));part.find('geometry/box/size').text=' '.join(map(str,size))
        if name=='lift_car':
            # Three moving cabin walls leave the sliding door's front rail
            # clear. The complete car mass/inertia is authored above.
            for i,(position,extent) in enumerate([((.95,0,1.15),(.08,2.,2.2)),((.1,.95,1.15),(1.8,.08,2.2)),((.1,-.95,1.15),(1.8,.08,2.2))]):
                for kind in ('collision','visual'):
                    item=child(link,kind,name='car_wall_'+str(i)+'_'+kind)
                    child(item,'pose',' '.join(map(str,position))+' 0 0 0');child(child(child(item,'geometry'),'box'),'size',' '.join(map(str,extent)))
        joint=child(hardware,'joint',name=name,type='prismatic');child(joint,'parent','lift_platform' if name=='door_lift' else 'anchor');child(joint,'child',link_name)
        drive=child(joint,'axis');child(drive,'xyz',axis);limits=child(drive,'limit')
        for key,value in dict(lower=0,upper=maximum,effort=limit,velocity=.25).items():child(limits,key,value)
        plugin=child(hardware,'plugin',name='ignition::gazebo::systems::JointPositionController',filename='ignition-gazebo-joint-position-controller-system')
        for key,value in dict(joint_name=name,topic='/facility/'+name+'/target',p_gain=kp,d_gain=kd,i_gain=100 if name=='lift_car' else 20,i_min=-8,i_max=8,cmd_min=-limit,cmd_max=limit,use_velocity_commands='false',initial_position=0).items():child(plugin,key,value)
        joints.append(name)
    p=child(hardware,'plugin',name='ignition::gazebo::systems::JointStatePublisher',filename='ignition-gazebo-joint-state-publisher-system');child(p,'topic','/facility/joint_states')
    target=ROOT/'src/robot_lab_maps/maps/fleet_facility';write(doc,target/'worlds/fleet_facility.world')
    config=dict(schema_version=1,doors={'main':dict(joint='door_main',closed=0.,opened=1.8,sweep=[-.5,.5,-1.1,3.1]),
        'lift':dict(joint='door_lift',closed=0.,opened=2.2,sweep=[2.6,3.4,-1.3,3.5],z_joint='lift_car')},
        lift=dict(name='facility_lift',joint='lift_car',door='lift',floors={'L0':0.,'L1':3.}),
        docks={'west_charger':dict(x=-4.5,y=-3.,z=0.,yaw=0.,position_tolerance=.12,height_tolerance=.12,yaw_tolerance=.15)},
        energy=dict(capacity_J=180000.,initial_fraction=.8,hotel_W=3.,drive_efficiency=.75,charge_W=120.,effort_observation_verified=False,
                    scope='Authored battery estimate from actual JointState effort/rate; unknown when motor feedback is missing'))
    (target/'infrastructure.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    (target/'SOURCE.json').write_text(json.dumps(dict(author='Robot Lab contributors',license='Apache-2.0',
        assets='Physical sliding doors, 50 kg force-driven 3 m lift car with moving walls/door, floor/shaft geometry and authored dock',
        scope='Uncalibrated first-party infrastructure; graph, lift transport, docking and energy missions need actual feedback receipts'),indent=2)+'\n')
    # Source-derived L0 occupancy slice: upper landing and dynamic leaves are
    # excluded; closed doors remain real live obstacles to the lidar/costmap.
    resolution=.05;image=Image.new('L',(240,200),254);draw=ImageDraw.Draw(image)
    for pose,size in walls:
        x,y,_=pose;dx,dy,_=size
        draw.rectangle(((x-dx/2+6)/resolution,(5-y-dy/2)/resolution,(x+dx/2+6)/resolution,(5-y+dy/2)/resolution),fill=0)
    draw.rectangle(((3+6)/resolution,(5-1)/resolution,(5+6)/resolution,(5+1)/resolution),fill=0) # shaft is not a static traversable floor
    maps=target/'maps';maps.mkdir(exist_ok=True);image.save(maps/'map.pgm')
    (maps/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=resolution,origin=[-6.,-5.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))


if __name__=='__main__':generate()
