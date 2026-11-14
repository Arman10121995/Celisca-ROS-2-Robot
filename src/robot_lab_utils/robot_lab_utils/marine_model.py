"""Source-derived ROV thrust allocation; authored parameters are not calibration."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


COEFFICIENTS = ('xU', 'yV', 'zW', 'kP', 'mQ', 'nR')


def hydro_coefficients(description):
    """Fortress Hydrodynamics coefficients declared in the URDF (missing ones are 0)."""
    root = ET.fromstring(description)
    plugin = next((p for p in root.findall('gazebo/plugin') if p.get('name', '').endswith('::Hydrodynamics')), None)
    if plugin is None:
        raise ValueError('The description declares no Fortress Hydrodynamics plugin')
    def read(tag):
        text = plugin.findtext(tag)
        return float(text) if text is not None else 0.
    linear = np.array([read(t) for t in COEFFICIENTS])
    quadratic = np.array([read(t[0] + t[1] + 'abs' + t[1]) for t in COEFFICIENTS])   # xUabsU, ..., nRabsR
    added = np.array([read(t[0] + 'Dot' + t[1]) for t in COEFFICIENTS])
    current = np.array([float(v) for v in (plugin.findtext('default_current') or '0 0 0').split()])
    return dict(linear=linear, quadratic=quadratic, added=added, link=plugin.findtext('link_name') or 'base_link',
                default_current=current)



class MarineModel:
    thruster_count = 8
    controlled_axes = (0, 1, 2, 3, 4, 5)
    primitive_wheels = False
    source_surface_chain = False
    def __init__(self, description, density=1025.):
        root=ET.fromstring(description)
        self.mass=0.
        self.displacement=0.
        for link in root.findall('link'):
            mass=link.find('inertial/mass')
            self.mass+=float(mass.get('value')) if mass is not None else 0.
            for collision in link.findall('collision'):
                box=collision.find('geometry/box')
                cylinder=collision.find('geometry/cylinder')
                if (self.primitive_wheels or self.source_surface_chain) and cylinder is not None:
                    self.displacement+=math.pi*float(cylinder.get('radius'))**2*float(cylinder.get('length'))
                    continue
                if box is None:
                    raise ValueError('The authored ROV allocation currently requires box displacement geometry')
                self.displacement+=float(np.prod([float(v) for v in box.get('size').split()]))
        self.density=float(density)
        if not all(math.isfinite(v) and v>0 for v in (self.mass,self.displacement,self.density)):
            raise ValueError('Marine source mass/volume/density must be positive and finite')
        origin=root.find("link[@name='base_link']/inertial/origin")
        centre=np.array([float(v) for v in origin.get('xyz','0 0 0').split()]) if origin is not None else np.zeros(3)
        self.joints=[]
        self.bounds=[]
        columns=[]
        for plugin in root.findall('gazebo/plugin'):
            if not plugin.get('name','').endswith('::Thruster'):
                continue
            name=plugin.findtext('joint_name')
            joint=root.find("joint[@name='%s']"%name)
            if joint is None or joint.find('parent') is None:
                raise ValueError('Marine source thrusters must attach directly to base_link')
            pose=joint.find('origin')
            position=np.array([float(v) for v in pose.get('xyz').split()])
            orient=[float(v) for v in pose.get('rpy','0 0 0').split()]
            direction=rotation(orient)@np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            parent=joint.find('parent').get('link')
            seen=set()
            while parent!='base_link':
                if not self.source_surface_chain or parent in seen:
                    raise ValueError('Marine source thruster chain must terminate at base_link')
                seen.add(parent)
                mounts=[j for j in root.findall('joint') if j.find('child').get('link')==parent]
                if len(mounts)!=1:raise ValueError('Ambiguous source thruster mount: '+parent)
                mount=mounts[0]
                if mount.get('type')!='fixed':
                    # WAM-V H layout: source azimuth motors remain at zero.
                    holds=[p for p in root.findall('gazebo/plugin')
                           if p.get('name','').endswith('::JointPositionController')
                           and p.findtext('joint_name')==mount.get('name')
                           and float(p.findtext('initial_position','0'))==0.]
                    if not holds:raise ValueError('Allocation requires a source zero-angle azimuth hold')
                pose=mount.find('origin')
                orient=list(map(float,pose.get('rpy','0 0 0').split())) if pose is not None else [0.,0.,0.]
                offset=np.array(list(map(float,pose.get('xyz','0 0 0').split()))) if pose is not None else np.zeros(3)
                matrix=rotation(orient)
                position=offset+matrix@position
                direction=matrix@direction
                parent=mount.find('parent').get('link')
            direction/=np.linalg.norm(direction)
            columns.append(np.r_[direction,np.cross(position-centre,direction)])
            self.joints.append(name)
            limits=[float(plugin.findtext(tag)) for tag in ('min_thrust_cmd','max_thrust_cmd')]
            if not all(math.isfinite(v) for v in limits) or limits[0]>=0 or limits[1]<=0:
                raise ValueError('Marine thrusters require finite signed force limits')
            velocity_limit=joint.find('limit')
            if velocity_limit is not None and plugin.findtext('velocity_control','false').lower()=='true':
                speed=float(velocity_limit.get('velocity','inf'))
                coefficient=abs(float(plugin.findtext('thrust_coefficient','1')))
                diameter=float(plugin.findtext('propeller_diameter','.02'))
                fluid=float(plugin.findtext('fluid_density','1000'))
                if not speed>0 or not all(math.isfinite(v) and v>0 for v in (coefficient,diameter,fluid)):
                    raise ValueError('Invalid source propeller velocity/force conversion')
                maximum=fluid*coefficient*diameter**4*speed**2
                limits=[max(limits[0],-maximum),min(limits[1],maximum)]
            self.bounds.append(limits)
        self.matrix=np.array(columns).T
        axes=list(self.controlled_axes)
        if self.matrix.shape!=(6,self.thruster_count) or np.linalg.matrix_rank(self.matrix[axes])!=len(axes):
            raise ValueError('Marine source thrusters do not span the declared controlled axes')
        self.inverse=np.zeros((self.thruster_count,6))
        self.inverse[:,axes]=np.linalg.pinv(self.matrix[axes])
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


class SurfaceModel(MarineModel):
    """Two physical propellers control surge and yaw; buoyancy supplies heave."""
    thruster_count = 2
    controlled_axes = (0, 5)
    source_surface_chain = True


class AmphibiousModel(SurfaceModel):
    """Same physical two-propeller allocation plus declared buoyant tires."""
    primitive_wheels = True
