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


def load_model(description, config):
    """Keep leg IK and passive-wheel serpentine kinematics distinct."""
    from pathlib import Path
    if yaml.safe_load(Path(config).read_text()).get('kind')=='serpentine':
        from .serpentine_model import SerpentineModel
        return SerpentineModel(description, config)
    return GaitModel(description, config)


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
        self.root_paths={leg['root']:self._planar_path(robot,leg['root']) for leg in self.legs}
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

    def _planar_path(self,robot,link):
        """Joints from the URDF root to a leg's root link, as planar steps (x, y, yaw, moving joint)."""
        spines={spine['joint'] for spine in self.config.get('spines',[])};steps=[]
        while link!=robot.get_root():
            joint_name,parent=robot.parent_map[link];joint=robot.joint_map[joint_name]
            origin=joint.origin;xyz=origin.xyz if origin is not None and origin.xyz else [0.,0.,0.]
            rpy=origin.rpy if origin is not None and origin.rpy else [0.,0.,0.]
            moving=None
            if joint.type!='fixed':
                axis=joint.axis or [1.,0.,0.]
                if joint_name not in spines or abs(abs(axis[2])-1.)>1e-6:
                    raise ValueError('Leg root %s moves through a non-spine or non-vertical joint %s'%(link,joint_name))
                moving=(joint_name,math.copysign(1.,axis[2]))
            steps.append((float(xyz[0]),float(xyz[1]),float(rpy[2]),moving));link=parent
        return steps[::-1]

    def root_pose(self,link,spine_angles=None):
        """Planar pose (origin, yaw) of a leg root link in the URDF root frame for the given spine angles."""
        x=y=heading=0.
        for ox,oy,oyaw,moving in self.root_paths[link]:
            x+=math.cos(heading)*ox-math.sin(heading)*oy;y+=math.sin(heading)*ox+math.cos(heading)*oy
            heading+=oyaw
            if moving:heading+=moving[1]*float((spine_angles or {}).get(moving[0],0.))
        return np.array([x,y]),heading

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

    def targets(self,phase,command,previous,gait,spine_phase=0.,spine_freq=1.,terrain=None):
        vx,vy,wz=map(float,command)
        if not all(math.isfinite(v) for v in (vx,vy,wz,phase,spine_phase,spine_freq)):raise ValueError('Finite gait inputs required')
        if not 0<=spine_phase<=1 or not .1<=spine_freq<=3:raise ValueError('Spine phase must be 0–1 and frequency 0.1–3')
        result=np.array(previous,dtype=float).copy();errors=[];planned={}
        yaw=self.config['forward_yaw'];c,s=math.cos(yaw),math.sin(yaw)
        body_velocity=np.array([c*vx-s*vy,s*vx+c*vy])
        spine_angles={}
        for spine in self.config.get('spines',[]):
            index=self.names.index(spine['joint'])
            # Authored limited curvature plus a small propagated body wave.
            result[index]=spine_angles[spine['joint']]=max(-.12,min(.12,-.5*wz+.015*math.sin(2*math.pi*(phase*spine_freq+spine_phase)-spine['index']*.6)))
        # The twist is the URDF root frame's, as Nav2 and odometry read it:
        # each foot moves with v + wz x r, r measured from the root origin.
        # Turning about body_center instead (2026-11-20) swung hexapod_ros2's
        # root, 0.163 m from its centre, 0.15 m per radian of an in-place
        # turn. A leg on a spine segment gets its root's pose from this
        # tick's spine angles and its stride turned into that segment's frame.
        # One scale for every stride keeps the foot motion rigid; clipping
        # each leg alone made the far feet slip against the near ones.
        strides={}
        for leg in self.legs:
            name=leg['id'];duty=self.offset(leg,gait)[1]
            origin,heading=self.root_pose(leg['root'],spine_angles);c,s=math.cos(heading),math.sin(heading)
            arm=origin+np.array([c*self.home[name][0]-s*self.home[name][1],s*self.home[name][0]+c*self.home[name][1]])
            velocity=body_velocity+wz*np.array([-arm[1],arm[0]])
            strides[name]=np.array([c*velocity[0]+s*velocity[1],-s*velocity[0]+c*velocity[1]])*self.config['cycle_s']*duty
        longest=max(np.linalg.norm(stride) for stride in strides.values())
        scale=min(1.,self.config['max_stride']/longest) if longest>0 else 1.
        for leg in self.legs:
            name=leg['id'];shift,duty=self.offset(leg,gait);p=(phase+shift)%1
            home=self.home[name];stride=strides[name]*scale
            target=home.copy()
            adjustment=(terrain or {}).get(name,{})
            target[2]+=float(adjustment.get('offset',0.))
            if p<duty:
                target[:2]+=(.5-p/duty)*stride;planned[name]=True
            else:
                swing=(p-duty)/(1-duty)
                smooth=swing*swing*(3-2*swing)
                target[:2]+=(-.5+smooth)*stride
                target[2]+=(self.config['step_height']+float(adjustment.get('swing_extra',0.)))*math.sin(math.pi*swing)**2
                planned[name]=False
            indices=self.indices[name];q,error=self.solve(name,target,previous[indices])
            result[indices]=q;errors.append(error)
        return result,max(errors),planned
