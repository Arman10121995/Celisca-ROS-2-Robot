"""Source-bounded Bullet implicit servos, actual contacts and local terrain rays."""
import json
import math
import xml.etree.ElementTree as ET
import numpy as np


class GaitPlant:
    def __init__(self,sdk,robot,description,config,probes,contacts=None):
        self.p,self.robot=sdk,robot
        self.config=json.loads(config) if isinstance(config,str) else config
        self.probes=json.loads(probes) if isinstance(probes,str) else probes
        self.links={sdk.getBodyInfo(robot)[0].decode():-1}
        self.indices={}
        for i in range(sdk.getNumJoints(robot)):
            row=sdk.getJointInfo(robot,i);self.indices[row[1].decode()]=i;self.links[row[12].decode()]=i
        tree=ET.fromstring(description)
        for link in tree.findall('link'):
            name=link.get('name')
            if link.find('inertial') is None and name in self.links:
                index=self.links[name]
                if index<0 or sdk.getJointInfo(robot,index)[2]!=sdk.JOINT_FIXED or link.find('collision') is not None:
                    raise ValueError('Source gait needs reviewed mass properties: '+name)
                sdk.changeDynamics(robot,index,mass=0.,localInertiaDiagonal=[0.,0.,0.])
        specs={j.get('name'):j for j in tree.findall('joint')}
        self.names=self.config['joints'];count=len(self.names)
        if not count or len(set(self.names))!=count:raise ValueError('Unique source gait joints required')
        if self.config.get('mode',['position']*count)!=['position']*count:raise ValueError('Bullet source gait accepts position joints only')
        self.defaults=np.asarray(self.config['default'],dtype=float)
        if self.defaults.shape!=(count,) or not np.isfinite(self.defaults).all():raise ValueError('Finite source defaults required')
        self.limits=[]
        for name,default,cap in zip(self.names,self.defaults,self.config['torque_limit']):
            if name not in self.indices or name not in specs:raise ValueError('Source gait joint missing: '+name)
            limit=specs[name].find('limit')
            lower,upper,effort,velocity=[float(limit.get(key)) for key in ('lower','upper','effort','velocity')]
            if not all(map(math.isfinite,[lower,upper,effort,velocity,cap])) or not lower<=default<=upper or min(effort,velocity,cap)<=0:raise ValueError('Invalid source servo envelope: '+name)
            self.limits.append((lower,upper,min(effort,cap),velocity))
        if len(self.limits)!=count:raise ValueError('Complete source effort vector required')
        self.targets=self.defaults.copy();self.at=None;self.timeout=float(self.config.get('timeout_s',.5))
        self.foot_groups={self.links[probe['foot']]:name for name,probe in self.probes.items()}
        # Named contact groups by URDF collision name, as the MuJoCo plant
        # takes them (gait_contact_config). The passive-wheel snake has no
        # foot probes; its odometry needs each wheel's rolling contact.
        self.groups=[name for name in self.probes]
        contacts=json.loads(contacts) if isinstance(contacts,str) and contacts else contacts
        if contacts:
            owner={collision.get('name'):link.get('name') for link in tree.findall('link')
                   for collision in link.findall('collision') if collision.get('name')}
            self.foot_groups={}
            for group,collisions in contacts.items():
                for collision in collisions:
                    if collision not in owner or owner[collision] not in self.links:
                        raise ValueError('Configured gait contact geometry is missing: '+collision)
                    index=self.links[owner[collision]]
                    if self.foot_groups.get(index,group)!=group:raise ValueError('Contact groups must be disjoint')
                    self.foot_groups[index]=group
            self.groups=[name for name in contacts if name!='body']
            # The URDF gives these links no surface friction; MuJoCo's and
            # Gazebo's default is 1.0, Bullet's 0.5. At 0.5 the snake's passive
            # wheels slid sideways and 35 % of its rolling-odometry samples
            # failed their no-slip check (2026-11-20).
            for index,group in self.foot_groups.items():
                if group!='body':sdk.changeDynamics(robot,index,lateralFriction=1.)
        # Bullet's rolling cylinders touch the plane on alternate substeps:
        # the last substep showed 2-4 of the snake's 20 wheels, short of its
        # odometry's three supports (2026-11-20). Contact groups average each
        # substep's normal forces over the publish window instead.
        self.averaged=bool(contacts);self.window={};self.window_steps=0        # Keep ordinary collisions (mask 1) while excluding the whole robot
        # from the terrain ray's mask 1. Stepping through a hollow trunk in
        # millimetres is neither reliable nor a range observation.
        for body_index in range(sdk.getNumBodies()):
            body=sdk.getBodyUniqueId(body_index)
            if body!=robot:
                for link in range(-1,sdk.getNumJoints(body)):
                    sdk.setCollisionFilterGroupMask(body,link,collisionFilterGroup=1,collisionFilterMask=3)
        for index in range(-1,sdk.getNumJoints(robot)):
            sdk.setCollisionFilterGroupMask(robot,index,collisionFilterGroup=2,collisionFilterMask=1)

    def seed(self):
        """Initial/reset-only joint placement; never called in the motion loop."""
        for name,value in zip(self.names,self.defaults):self.p.resetJointState(self.robot,self.indices[name],float(value))

    def receive(self,values,stamp):
        values=np.asarray(values,dtype=float)
        if values.shape!=self.defaults.shape or not np.isfinite(values).all():return False
        if any(not lo<=value<=hi for value,(lo,hi,_,_) in zip(values,self.limits)):return False
        self.targets,self.at=values.copy(),stamp;return True

    def step(self,stamp):
        targets=self.targets if self.at is not None and 0<=stamp-self.at<=self.timeout else self.defaults
        for name,target,(_,_,force,velocity) in zip(self.names,targets,self.limits):
            self.p.setJointMotorControl2(self.robot,self.indices[name],self.p.POSITION_CONTROL,
                targetPosition=float(target),force=force,maxVelocity=velocity,positionGain=.1,velocityGain=1.)

    def link_pose(self,name):
        index=self.links[name]
        if index<0:
            position,quaternion=self.p.getBasePositionAndOrientation(self.robot)
            inertial=self.p.getDynamicsInfo(self.robot,-1)
            inverse=self.p.invertTransform(inertial[3],inertial[4]);position,quaternion=self.p.multiplyTransforms(position,quaternion,*inverse)
        else:
            state=self.p.getLinkState(self.robot,index,computeForwardKinematics=True);position,quaternion=state[4:6]
        return np.asarray(position),np.asarray(self.p.getMatrixFromQuaternion(quaternion)).reshape(3,3)

    def sole_offset(self,name):
        return self.p.getAABB(self.robot,self.links[name])[0][2]-self.link_pose(name)[0][2]

    def ray(self,origin,length):
        finish=np.asarray(origin)-[0.,0.,length]
        hit=self.p.rayTest(np.asarray(origin).tolist(),finish.tolist(),collisionFilterMask=1)[0]
        return float(origin[2]-hit[3][2]) if hit[0]>=0 and hit[0]!=self.robot else None

    def align(self,reference):
        lifts=[]
        supports=[probe['foot'] for probe in self.probes.values()] or [
            name for name,index in self.links.items() if self.foot_groups.get(index,'body')!='body']
        if not supports:raise ValueError('No configured support links to place on the floor')
        for link in supports:
            position=self.link_pose(link)[0];origin=position.copy();origin[2]=reference+.6
            distance=self.ray(origin,1.2)
            if distance is None:raise ValueError('No actual initial foot support')
            lifts.append(origin[2]-distance-self.p.getAABB(self.robot,self.links[link])[0][2]+.002)
        if max(lifts)-min(lifts)>.04:raise ValueError('Initial source stance needs flatter support')
        position,q=self.p.getBasePositionAndOrientation(self.robot)
        self.p.resetBasePositionAndOrientation(self.robot,[position[0],position[1],position[2]+max(lifts)],q)

    def forces(self):
        forces={name:0. for name in self.groups};forces['body']=0.
        for contact in self.p.getContactPoints(bodyA=self.robot):
            if contact[2]==self.robot or self.p.getDynamicsInfo(contact[2],-1)[0]>0:continue
            forces[self.foot_groups.get(contact[3],'body')]+=max(0.,float(contact[9]))
        return forces

    def sample(self):
        """After each physics substep: accumulate averaged contact groups."""
        if not self.averaged:return
        # At most the last 50 substeps (50 ms): a window from spawn averaged
        # the drop onto the floor and failed the arm's support check.
        if self.window_steps>=50:self.window={};self.window_steps=0
        for name,value in self.forces().items():self.window[name]=self.window.get(name,0.)+value
        self.window_steps+=1

    def measure(self,stamp):
        if self.averaged and self.window_steps:
            forces={name:self.window.get(name,0.)/self.window_steps for name in self.groups+['body']}
            self.window={};self.window_steps=0
        else:forces=self.forces()
        return dict(schema_version=1,clock_time=stamp,sim_time=stamp,normal_forces_N=forces,source='Actual Bullet static-world collision normal forces'+(' (mean over the window substeps)' if self.averaged else ''))
