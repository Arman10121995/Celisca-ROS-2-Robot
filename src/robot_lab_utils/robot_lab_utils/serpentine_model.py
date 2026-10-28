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
        self.velocity=np.array([float(v.get('velocity')) for v in limits]);self.torque=np.array([float(v.get('effort')) for v in limits])
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
        amplitude=self.config['wave_amplitude']*min(1.,.35+.65*intensity)
        direction=-1. if linear<0 else 1.
        angles=2*math.pi*(direction*phase+np.arange(len(self.names))*self.config['wave_cycles']/len(self.names)+spine_phase)
        target=amplitude*np.sin(angles)+.07*max(-1.,min(1.,yaw))
        return np.clip(target,self.lower,self.upper),None,{}

    def wheels(self,positions):
        """Wheel centres/directions in the head frame, from actual spine angles."""
        length=float(self.config['segment_length']);width=float(self.config['wheel_half_track'])
        position=np.zeros(2);heading=0.;result={}
        for i in range(self.count):
            if i:
                offset=-length/2 if i==1 else -length
                position+=offset*np.array([math.cos(heading),math.sin(heading)])
                heading+=positions['spine_'+str(i)]
            c,s=math.cos(heading),math.sin(heading);rotation=np.array([[c,-s],[s,c]])
            for side,y in [('left',width),('right',-width)]:
                name='wheel_'+str(i)+'_'+side
                result[name]=(position+rotation@np.array([0. if i==0 else -length/2,y]),heading)
        return result


def encoder_velocity(previous,current,wheel_rates,supports,dt,yaw_rate,radius):
    """Fit planar head motion; reject inconsistent rolling/slipping support."""
    if not 0<dt<=.15 or len(supports)<6 or not math.isfinite(yaw_rate):
        raise ValueError('Fresh IMU and at least six measured rolling contacts required')
    matrix=[];values=[]
    for name in supports:
        point,heading=current[name];old=previous[name][0]
        articulated=(point-old)/dt;rotating=yaw_rate*np.array([-point[1],point[0]])
        forward=np.array([math.cos(heading),math.sin(heading)])
        lateral=np.array([-forward[1],forward[0]])
        matrix.extend((forward,lateral))
        values.extend((radius*wheel_rates[name]-forward@(articulated+rotating),-lateral@(articulated+rotating)))
    matrix=np.asarray(matrix);values=np.asarray(values)
    if not np.isfinite(matrix).all() or not np.isfinite(values).all():raise ValueError('Finite actual rolling kinematics required')
    velocity,_,rank,_=np.linalg.lstsq(matrix,values,rcond=None)
    scatter=float(np.max(np.abs(matrix@velocity-values)))
    if rank!=2 or scatter>.15 or np.linalg.norm(velocity)>.5:raise ValueError('Rolling/slip constraints inconsistent; odometry suspended')
    return velocity,scatter
