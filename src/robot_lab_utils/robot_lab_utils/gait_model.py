"""Source-bound multi-leg IK and explicit educational gait trajectories.

KDL tree conversion is reused unchanged from the pinned MIT hexapod source.
Tripod/tetrapod/wave schedules are separate from actual contact measurements.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import yaml
from .vendor.hexapod_kinematics import urdf_to_kdl_tree


class GaitModel:
    def __init__(self,description,config):
        self.config=yaml.safe_load(Path(config).read_text()) if isinstance(config,(str,Path)) else dict(config)
        self.robot_id=self.config['robot_id']
        root=ET.fromstring(description)
        for item in list(root):
            if item.tag in ('gazebo','ros2_control','mujoco'):root.remove(item)
        # Old urdf_parser_py warns about valid named collision/visual elements;
        # they do not participate in the kinematic tree. The plant keeps them.
        for link in root.findall('link'):
            for item in list(link):
                if item.tag in ('visual','collision'):link.remove(item)
        robot=URDF.from_xml_string(ET.tostring(root,encoding='unicode'))
        tree=urdf_to_kdl_tree(robot)
        self.tree=tree
        self.legs=self.config['legs']
        self.names=[];self.initial=[];self.lower=[];self.upper=[];self.torque=[];self.velocity=[]
        self.chains={};self.fk={};self.jac={};self.indices={};self.home={}
        for leg in self.legs:
            name=leg['id'];chain=tree.getChain(leg['root'],leg['foot'])
            actual=[chain.getSegment(i).getJoint().getName() for i in range(chain.getNrOfSegments())
                    if chain.getSegment(i).getJoint().getType()!=kdl.Joint.Fixed]
            if actual!=leg['joints'] or len(actual)!=3:raise ValueError('Exact three-joint leg chain required: '+name)
            self.chains[name]=chain;self.fk[name]=kdl.ChainFkSolverPos_recursive(chain);self.jac[name]=kdl.ChainJntToJacSolver(chain)
            self.indices[name]=slice(len(self.names),len(self.names)+3)
            for joint,value in zip(actual,leg['initial']):self._append(robot.joint_map[joint],value)
            self.home[name]=self.position(name,leg['initial'])
        for spine in self.config.get('spines',[]):self._append(robot.joint_map[spine['joint']],0.)
        for name,value in self.config.get('passive_targets',{}).items():self._append(robot.joint_map[name],value)
        self.initial=np.array(self.initial);self.lower=np.array(self.lower);self.upper=np.array(self.upper)
        self.torque=np.array(self.torque);self.velocity=np.array(self.velocity)
        if len(set(self.names))!=len(self.names):raise ValueError('Duplicate gait joints')
        if not all(np.isfinite(v).all() for v in (self.initial,self.lower,self.upper,self.torque,self.velocity)):
            raise ValueError('Finite source actuator limits required')
        if (self.initial<self.lower).any() or (self.initial>self.upper).any():raise ValueError('Initial gait pose outside source limits')

    def _append(self,joint,value):
        if joint.limit is None or joint.limit.effort<=0 or joint.limit.velocity<=0:raise ValueError('Source joint limits required: '+joint.name)
        self.names.append(joint.name);self.initial.append(value)
        self.lower.append(joint.limit.lower);self.upper.append(joint.limit.upper)
        self.torque.append(min(float(joint.limit.effort),float(self.config['torque_limit'])))
        self.velocity.append(min(float(joint.limit.velocity),2.))

    @staticmethod
    def array(values):
        result=kdl.JntArray(len(values))
        for i,value in enumerate(values):result[i]=float(value)
        return result

    def position(self,leg,q):
        frame=kdl.Frame();self.fk[leg].JntToCart(self.array(q),frame)
        return np.array([frame.p.x(),frame.p.y(),frame.p.z()])

    def solve(self,leg,target,seed):
        target=np.array(target,dtype=float)
        if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('Finite XYZ foot target required')
        indices=self.indices[leg];q=np.array(seed,dtype=float)
        for _ in range(45):
            error=target-self.position(leg,q)
            if np.linalg.norm(error)<.00015:return q,float(np.linalg.norm(error))
            jac=kdl.Jacobian(3);self.jac[leg].JntToJac(self.array(q),jac)
            matrix=np.array([[jac[r,c] for c in range(3)] for r in range(3)])
            step=matrix.T@np.linalg.solve(matrix@matrix.T+.0001*np.eye(3),error)
            q=np.clip(q+np.clip(step,-.12,.12),self.lower[indices],self.upper[indices])
        return q,float(np.linalg.norm(target-self.position(leg,q)))

    def pd_config(self):
        return dict(topic='/gait/joint_targets',timeout_s=.5,joints=self.names,
                    mode=['position']*len(self.names),kp=[self.config['kp']]*len(self.names),
                    kd=[self.config['kd']]*len(self.names),torque_limit=self.torque.tolist(),default=self.initial.tolist())

    def offset(self,leg,gait):
        pair,side=leg['pair'],leg['side']
        if gait in ('tripod','tetrapod'):return .5*((pair+int(side=='r'))%2),.65
        if gait=='ripple':return ((pair*2+int(side=='r'))%len(self.legs))/len(self.legs),.8
        if gait=='wave':return ((pair+(.5 if side=='r' else 0.))/(len(self.legs)/2))%1.,.85
        raise ValueError('Unsupported gait schedule')

    def targets(self,phase,command,previous,gait):
        vx,vy,wz=map(float,command)
        if not all(math.isfinite(v) for v in (vx,vy,wz,phase)):raise ValueError('Finite gait inputs required')
        result=np.array(previous,dtype=float).copy();errors=[];planned={}
        yaw=self.config['forward_yaw'];c,s=math.cos(yaw),math.sin(yaw)
        body_velocity=np.array([c*vx-s*vy,s*vx+c*vy])
        for leg in self.legs:
            name=leg['id'];shift,duty=self.offset(leg,gait);p=(phase+shift)%1
            home=self.home[name];arm=home[:2]+np.array(leg.get('body_offset',[0.,0.]))-self.config['body_center']
            velocity=body_velocity+wz*np.array([-arm[1],arm[0]])
            stride=velocity*self.config['cycle_s']*duty
            length=np.linalg.norm(stride)
            if length>self.config['max_stride']:stride*=self.config['max_stride']/length
            target=home.copy()
            if p<duty:
                target[:2]+=(.5-p/duty)*stride;planned[name]=True
            else:
                swing=(p-duty)/(1-duty)
                smooth=swing*swing*(3-2*swing)
                target[:2]+=(-.5+smooth)*stride
                target[2]+=self.config['step_height']*math.sin(math.pi*swing)**2
                planned[name]=False
            indices=self.indices[name];q,error=self.solve(name,target,previous[indices])
            result[indices]=q;errors.append(error)
        for spine in self.config.get('spines',[]):
            index=self.names.index(spine['joint'])
            # Authored limited curvature plus a small propagated body wave.
            result[index]=max(-.12,min(.12,-.5*wz+.015*math.sin(2*math.pi*phase-spine['index']*.6)))
        return result,max(errors),planned
