"""Source-limited serpenoid targets and exact planar wheel-chain kinematics."""
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml


class SerpentineModel:
    def __init__(self,description,config):
        from pathlib import Path
        self.config=yaml.safe_load(Path(config).read_text());self.robot_id=self.config['robot_id']
        if self.config.get('kind')!='serpentine':raise ValueError('A serpentine source configuration is required')
        self.count=int(self.config['segments'])
        if not 4<=self.count<=20:raise ValueError('Serpentine segment count must be 4–20')
        root=ET.fromstring(description);self.names=['spine_'+str(i) for i in range(1,self.count)]
        limits=[root.find("joint[@name='%s']/limit"%name) for name in self.names]
        if any(v is None for v in limits):raise ValueError('Complete source spine joints are required')
        self.lower=np.array([float(v.get('lower')) for v in limits]);self.upper=np.array([float(v.get('upper')) for v in limits])
        self.velocity=np.minimum(np.array([float(v.get('velocity')) for v in limits]),float(self.config.get('joint_velocity_limit',2.)))
        self.torque=np.array([float(v.get('effort')) for v in limits])
        self.initial=np.zeros(len(self.names))
        self.legs=[dict(id='wheel_'+str(i)+'_'+side,contact_geoms=['wheel_'+str(i)+'_'+side+'_collision'])
                   for i in range(self.count) for side in ('left','right')]
        self.body_geoms=['base_link_collision']+['segment_'+str(i)+'_collision' for i in range(1,self.count)]
        self.footprint_radius=self.count*float(self.config['segment_length'])+.15
        scalars=[self.config[k] for k in ('segment_length','wheel_radius','wheel_half_track','max_speed','max_yaw','cycle_s','kp','kd','torque_limit','wave_amplitude','wave_cycles')]
        if (not np.isfinite(scalars).all() or min(scalars)<=0
                or not np.isfinite([self.lower,self.upper,self.velocity,self.torque]).all()
                or (self.lower>=0).any() or (self.upper<=0).any() or (self.velocity<=0).any() or (self.torque<=0).any()):
            raise ValueError('Finite positive source geometry/gains and signed spine limits required')
        length=float(self.config['segment_length']);width=float(self.config['wheel_half_track'])
        def vector(node,attribute,expected):
            if node is None:return False
            try:return np.allclose([float(v) for v in node.get(attribute,'').split()],expected,atol=1e-9,rtol=0.)
            except (ValueError,TypeError):return False
        for i in range(self.count):
            parent='base_link' if i==0 else 'segment_'+str(i)
            if i:
                joint=root.find("joint[@name='spine_%d']"%i)
                if (joint.get('type')!='revolute'
                        or not vector(joint.find('origin'),'xyz',[-length/2 if i==1 else -length,0,0])
                        or not vector(joint.find('origin'),'rpy',[0,0,0])
                        or not vector(joint.find('axis'),'xyz',[0,0,1])
                        or joint.find('parent').get('link')!=('base_link' if i==1 else 'segment_'+str(i-1))
                        or joint.find('child').get('link')!=parent):
                    raise ValueError('Serpentine source spine geometry differs from encoder FK')
            for side,y in [('left',width),('right',-width)]:
                wheel='wheel_'+str(i)+'_'+side;joint=root.find("joint[@name='%s_joint']"%wheel)
                cylinder=root.find("link[@name='%s']/collision/geometry/cylinder"%wheel)
                if (joint is None or joint.get('type')!='continuous'
                        or not vector(joint.find('origin'),'xyz',[0 if i==0 else -length/2,y,-.02])
                        or not vector(joint.find('origin'),'rpy',[0,0,0])
                        or not vector(joint.find('axis'),'xyz',[0,1,0])
                        or joint.find('parent').get('link')!=parent
                        or cylinder is None or not math.isclose(float(cylinder.get('radius')),float(self.config['wheel_radius']),abs_tol=1e-9)):
                    raise ValueError('Serpentine source wheel geometry differs from encoder FK')

    def pd_config(self):
        n=len(self.names)
        return dict(topic='/gait/joint_targets',timeout_s=.4,stale_mode='hold',joints=self.names,
            mode=['position']*n,kp=[self.config['kp']]*n,kd=[self.config['kd']]*n,
            torque_limit=np.minimum(self.torque,self.config['torque_limit']).tolist(),default=self.initial.tolist())

    def targets(self,phase,velocity,previous,gait,spine_phase=0.,spine_freq=1.):
        if gait!='serpenoid' or abs(velocity[1])>1e-9:raise ValueError('The wheeled snake supports serpenoid surge/yaw, not strafe or leg schedules')
        linear=float(velocity[0])/self.config['max_speed'];yaw=float(velocity[2])/self.config['max_yaw']
        intensity=min(1.,max(abs(linear),abs(yaw)*.6))
        if intensity<.01:return np.asarray(previous).copy(),None,{}
        amplitude=self.config['wave_amplitude']*math.sqrt(intensity)
        # With this head-to-tail source chain, increasing spatial phase drives
        # the passive wheels backward. Reverse time phase for forward surge.
        direction=1. if linear<0 else -1.
        angles=2*math.pi*(direction*phase+np.arange(len(self.names))*self.config['wave_cycles']/len(self.names)+spine_phase)
        # Along the source head-to-tail chain, forward left curvature needs
        # negative joint bias. Reverse travel reverses that curvature/yaw law.
        target=amplitude*np.sin(angles)+direction*.07*max(-1.,min(1.,yaw))
        return np.clip(target,self.lower,self.upper),None,{}

    def wheels(self,positions):
        """Wheel centres/directions in the head frame, from actual spine angles."""
        return self.wheel_kinematics(positions,{})[0]

    def wheel_kinematics(self,positions,rates):
        """Exact wheel-centre Jacobian velocity from measured spine rates."""
        length=float(self.config['segment_length']);width=float(self.config['wheel_half_track'])
        position=np.zeros(2);velocity=np.zeros(2);heading=0.;angular=0.;result={};velocities={}
        for i in range(self.count):
            if i:
                offset=-length/2 if i==1 else -length
                offset=offset*np.array([math.cos(heading),math.sin(heading)])
                position+=offset;velocity+=angular*np.array([-offset[1],offset[0]])
                heading+=positions['spine_'+str(i)]
                angular+=rates.get('spine_'+str(i),0.)
            c,s=math.cos(heading),math.sin(heading);rotation=np.array([[c,-s],[s,c]])
            for side,y in [('left',width),('right',-width)]:
                name='wheel_'+str(i)+'_'+side
                offset=rotation@np.array([0. if i==0 else -length/2,y])
                result[name]=(position+offset,heading)
                velocities[name]=velocity+angular*np.array([-offset[1],offset[0]])
        return result,velocities

    def drive_heading(self,positions):
        """Mean segment tangent; the head oscillates during a travelling wave."""
        angles=np.r_[0.,np.cumsum([positions[name] for name in self.names])]
        vector=np.array([np.cos(angles).mean(),np.sin(angles).mean()])
        if not np.isfinite(vector).all() or np.linalg.norm(vector)<.1:
            raise ValueError('Folded body has no observable mean drive heading')
        return math.atan2(vector[1],vector[0])


def encoder_velocity(previous,current,wheel_rates,supports,dt,yaw_rate,radius,relative_velocities=None):
    """Fit planar head motion; reject inconsistent rolling/slipping support."""
    if not 0<dt<=.15 or len(supports)<3 or not math.isfinite(yaw_rate):
        raise ValueError('Fresh IMU and at least three measured rolling contacts required')
    matrix=[];values=[]
    for name in supports:
        point,heading=current[name];old=previous[name][0]
        articulated=relative_velocities[name] if relative_velocities is not None else (point-old)/dt
        rotating=yaw_rate*np.array([-point[1],point[0]])
        forward=np.array([math.cos(heading),math.sin(heading)])
        lateral=np.array([-forward[1],forward[0]])
        # Spine waves cause lateral scrub. Encoders measure longitudinal
        # rolling, not zero lateral velocity at every segment.
        matrix.append(forward)
        values.append(radius*wheel_rates[name]-forward@(articulated+rotating))
    matrix=np.asarray(matrix);values=np.asarray(values)
    if not np.isfinite(matrix).all() or not np.isfinite(values).all():raise ValueError('Finite actual rolling kinematics required')
    singular=np.linalg.svd(matrix,compute_uv=False)
    if singular[-1]<singular[0]*.01:
        # Parallel encoders cannot observe lateral speed. Use one head-near
        # contact's explicit local no-side-slip assumption in this case.
        head=min(supports,key=lambda n:np.linalg.norm(current[n][0]))
        point,heading=current[head];lateral=np.array([-math.sin(heading),math.cos(heading)])
        extra=(relative_velocities[head] if relative_velocities is not None else (point-previous[head][0])/dt)+yaw_rate*np.array([-point[1],point[0]])
        matrix=np.vstack((matrix,lateral));values=np.r_[values,-lateral@extra]
    velocity,_,rank,_=np.linalg.lstsq(matrix,values,rcond=None)
    scatter=float(np.max(np.abs(matrix@velocity-values)))
    if rank!=2 or scatter>.15 or np.linalg.norm(velocity)>.5:raise ValueError('Rolling/slip constraints inconsistent; odometry suspended')
    return velocity,scatter
