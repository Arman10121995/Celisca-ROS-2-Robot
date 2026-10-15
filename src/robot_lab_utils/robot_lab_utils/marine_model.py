"""Source-derived ROV thrust allocation; authored parameters are not calibration."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


class MarineModel:
    def __init__(self, description, density=1025.):
        root=ET.fromstring(description)
        self.mass=0.
        self.displacement=0.
        for link in root.findall('link'):
            mass=link.find('inertial/mass')
            self.mass+=float(mass.get('value')) if mass is not None else 0.
            for collision in link.findall('collision'):
                box=collision.find('geometry/box')
                if box is None:
                    raise ValueError('The authored ROV allocation currently requires box displacement geometry')
                self.displacement+=float(np.prod([float(v) for v in box.get('size').split()]))
        self.density=float(density)
        if not all(math.isfinite(v) and v>0 for v in (self.mass,self.displacement,self.density)):
            raise ValueError('Marine source mass/volume/density must be positive and finite')
        origin=root.find("link[@name='base_link']/inertial/origin")
        centre=np.array([float(v) for v in origin.get('xyz').split()])
        self.joints=[]
        self.bounds=[]
        columns=[]
        for plugin in root.findall('gazebo/plugin'):
            if not plugin.get('name','').endswith('::Thruster'):
                continue
            name=plugin.findtext('joint_name')
            joint=root.find("joint[@name='%s']"%name)
            if joint is None or joint.find('parent') is None or joint.find('parent').get('link')!='base_link':
                raise ValueError('Marine source thrusters must attach directly to base_link')
            pose=joint.find('origin')
            position=np.array([float(v) for v in pose.get('xyz').split()])
            orient=[float(v) for v in pose.get('rpy','0 0 0').split()]
            direction=rotation(orient)@np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            direction/=np.linalg.norm(direction)
            columns.append(np.r_[direction,np.cross(position-centre,direction)])
            self.joints.append(name)
            limits=[float(plugin.findtext(tag)) for tag in ('min_thrust_cmd','max_thrust_cmd')]
            if not all(math.isfinite(v) for v in limits) or limits[0]>=0 or limits[1]<=0:
                raise ValueError('Marine thrusters require finite signed force limits')
            self.bounds.append(limits)
        self.matrix=np.array(columns).T
        if self.matrix.shape!=(6,8) or np.linalg.matrix_rank(self.matrix)!=6:
            raise ValueError('The complete ROV source must provide eight thrusters with six-axis allocation')
        self.inverse=np.linalg.pinv(self.matrix)
        self.bounds=np.asarray(self.bounds)
        self.gravity_trim=(self.mass-self.displacement*self.density)*9.81

    def allocate(self,wrench):
        value=np.asarray(wrench,dtype=float)
        if value.shape!=(6,) or not np.isfinite(value).all():
            raise ValueError('Marine wrench must contain six finite force/torque values')
        forces=self.inverse@value
        scale=1.
        for force,(low,high) in zip(forces,self.bounds):
            if force>high:scale=min(scale,high/force)
            if force<low:scale=min(scale,low/force)
        return forces*scale,scale
