"""Explicit selection of real free box objects for native Panda manipulation.

Publishes measured collision-geometry poses; selection never moves, attaches or
changes an object. Unsupported geometry is visible with a reason.
"""
import hashlib
import json
from pathlib import Path
import time
import uuid

import numpy as np
from std_msgs.msg import String


class NativeObjectCatalog:
    def __init__(self,node):
        self.node,self.model,self.data,self.mj=node,node.model,node.data,node.mujoco
        self.plant_id=uuid.uuid4().hex
        self.source_sha=hashlib.sha256(Path(node.get_parameter('native_mjcf').value).read_bytes()).hexdigest()
        self.objects={};self.supports={};self.unsupported=[];self.scene={};self.error=''
        by_body={}
        counts=np.zeros(self.model.nbody,dtype=int)
        for geom in range(self.model.ngeom):
            if self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]:
                body=int(self.model.geom_bodyid[geom]);by_body.setdefault(body,[]).append(geom);counts[body]+=1
        for body in range(self.model.nbody-1,0,-1):counts[int(self.model.body_parentid[body])]+=counts[body]
        for body in range(node.robot_bodies,self.model.nbody):
            joint=int(self.model.body_jntadr[body]);name=self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_BODY,body)
            if joint<0 or int(self.model.jnt_type[joint])!=int(self.mj.mjtJoint.mjJNT_FREE) or not name:continue
            geoms=by_body.get(body,[])
            child_collision=counts[body]!=len(geoms)
            if len(geoms)!=1 or child_collision or int(self.model.geom_type[geoms[0]])!=int(self.mj.mjtGeom.mjGEOM_BOX):
                self.unsupported.append(dict(id=name,reason='Selection requires one exact physical box; compound/mesh adapter pending'))
                continue
            if len(self.objects)>=256:
                self.unsupported.append(dict(id=name,reason='256 selectable object bound reached'));continue
            self.objects[name]=(body,geoms[0])
        self.publisher=node.create_publisher(String,'/manipulation/object_catalog',10)
        node.create_subscription(String,'/manipulation/select_object',self.select,1)
        node.create_subscription(String,'/arm/planning_scene_status',self.receive_scene,10)
        self.selected=next((name for name,(body,_) in self.objects.items() if body==node.object_body),None)
        self.explicit=False
        # Authored fixed horizontal box tops only. A box welded to a free or
        # articulated ancestor is not a stationary support surface.
        fixed=np.ones(self.model.nbody,dtype=bool)
        for body in range(1,self.model.nbody):
            fixed[body]=fixed[int(self.model.body_parentid[body])] and self.model.body_jntnum[body]==0
        for geom in range(self.model.ngeom):
            body=int(self.model.geom_bodyid[geom])
            if (0<body<node.robot_bodies or not fixed[body] or len(self.supports)>=256
                    or int(self.model.geom_type[geom])!=int(self.mj.mjtGeom.mjGEOM_BOX)
                    or not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom])):continue
            matrix=self.data.geom_xmat[geom].reshape(3,3)
            size=2*self.model.geom_size[geom]
            top=float(self.data.geom_xpos[geom,2]+size[2]/2)
            if abs(matrix[2,2])<1.-1e-6 or min(size[:2])<.04 or not .05<=top<=2.5:continue
            name=self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_GEOM,geom) or 'fixed_geom_'+str(geom)
            quaternion=np.zeros(4);self.mj.mju_mat2Quat(quaternion,self.data.geom_xmat[geom])
            self.supports[name]=dict(id=name,frame='native_world',type='box',size=size.tolist(),
                position=self.data.geom_xpos[geom].tolist(),orientation_wxyz=quaternion.tolist())

    def receive_scene(self,message):
        try:
            state=json.loads(message.data)
            if isinstance(state,dict):
                self.scene=state
                if self.selected in self.objects and state.get('object_id')==self.selected and state.get('ready'):
                    self.node.object_body=self.objects[self.selected][0]
        except (ValueError,TypeError):pass

    def select(self,message):
        try:
            if len(message.data)>4096:raise ValueError('Object selection command too large')
            request=json.loads(message.data)
            if not isinstance(request,dict):raise ValueError('Object selection must be a named record')
            if request.get('plant_id')!=self.plant_id or request.get('source_sha256')!=self.source_sha:
                return
            name=request['id']
            if name not in self.objects:raise ValueError('Choose a supported physical free box')
            if (self.node.arm.reserved or self.node.gripper.reserved or self.node.arm.servo.active()
                    or np.any(np.abs(self.data.qvel[self.node.arm.dofs])>.02)
                    or self.scene.get('object_attached') or self.scene and not self.scene.get('ready')):
                raise ValueError('Stop the robot and detach the planning object before selecting another object')
            old=self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_BODY,self.node.object_body) if self.node.object_body>=0 else None
            if old and old!=name and any(record.get('body')==old and record.get('normal_force_n',0.)>.02
                    for contacts in self.node.gripper.contacts().values() for record in contacts):
                raise ValueError('Release the current physical object before selecting another')
            self.selected=name
            self.explicit=True;self.error=''
        except (ValueError,TypeError,KeyError) as exc:self.error=str(exc)

    def snapshot(self):
        if self.selected is None:return None
        body,geom=self.objects[self.selected]
        quaternion=np.zeros(4)
        self.mj.mju_mat2Quat(quaternion,self.data.geom_xmat[geom])
        result=dict(id=self.selected,frame='native_world',type='box',
            size=(2*self.model.geom_size[geom]).tolist(),position=self.data.geom_xpos[geom].tolist(),
            orientation_wxyz=quaternion.tolist(),spatial_velocity=self.data.cvel[body].tolist(),
            time=float(self.data.time),source_sha256=self.source_sha,plant_id=self.plant_id,
            selected_by_operator=self.explicit,
            finger_links=['native_body_'+str(int(b)) for b in self.node.gripper.finger_bodies])
        # Independent physical custody/support observations for the object
        # sequencer. Planning attachment is not physical grasp evidence.
        fingers=list(map(int,self.node.gripper.finger_bodies));forces=[0.,0.];support_force=0.
        for index,contact in enumerate(self.data.contact[:self.data.ncon]):
            owners=[int(self.model.geom_bodyid[g]) for g in contact.geom]
            if body not in owners:continue
            other=owners[1] if owners[0]==body else owners[0]
            wrench=np.zeros(6);self.mj.mj_contactForce(self.model,self.data,index,wrench)
            force=abs(float(wrench[0]))
            if other in fingers:forces[fingers.index(other)]+=force
            else:support_force+=force
        reference=int(self.node.gripper.finger_bodies[0])
        relative=self.data.xmat[reference].reshape(3,3).T@(self.data.geom_xpos[geom]-self.data.xpos[reference])
        relative_quaternion=np.zeros(4)
        self.mj.mju_mat2Quat(relative_quaternion,(self.data.xmat[reference].reshape(3,3).T@self.data.geom_xmat[geom].reshape(3,3)).ravel())
        result['relative_hand_orientation_wxyz']=relative_quaternion.tolist()
        velocity=np.zeros(6);self.mj.mj_objectVelocity(self.model,self.data,self.mj.mjtObj.mjOBJ_BODY,body,velocity,0)
        result['custody']=dict(stamp=float(self.data.time),relative=relative.tolist(),left_force=forces[0],
            right_force=forces[1],supported=support_force>=.1,speed=float(np.linalg.norm(velocity[3:])))
        if self.selected.startswith('environment_'):result['world_model']=self.selected.removeprefix('environment_')
        if self.selected=='manipulation_object':
            pedestal=self.mj.mj_name2id(self.model,self.mj.mjtObj.mjOBJ_BODY,'manipulation_pedestal')
            shape=self.mj.mj_name2id(self.model,self.mj.mjtObj.mjOBJ_GEOM,'manipulation_pedestal_geom')
            if min(pedestal,shape)>=0:
                result['pedestal']=dict(type='box',size=(2*self.model.geom_size[shape]).tolist(),
                    position=self.data.xpos[pedestal].tolist(),orientation=self.data.xquat[pedestal].tolist())
        return result

    def publish(self):
        self.publisher.publish(String(data=json.dumps(dict(plant_id=self.plant_id,source_sha256=self.source_sha,
            frame='native_world',objects=list(self.objects),supports=self.supports,
            unsupported=self.unsupported[:256],selected=self.selected,error=self.error))))
        snapshot=self.snapshot()
        if snapshot is not None:self.node.object_pub.publish(String(data=json.dumps(snapshot,allow_nan=False)))
