"""Measured local support-ray adaptation; no simulator XY correction or pose writes."""
import math
import numpy as np


def sample_probes(probes, link_pose, sole_offset, ray, stamp):
    """SDK adapters supply link transforms, collision sole offsets and real rays."""
    result={};requests=[];rows=[]
    for name,probe in probes.items():
        position,matrix=link_pose(probe['root'])
        nominal=np.asarray(position)+np.asarray(matrix)@np.asarray(probe['home'])
        # Differential relief is referenced to the level source stance.
        # Including measured trunk tilt here shortens the low-side legs,
        # amplifying pitch instead of describing the terrain. Native tilt
        # remains an independent body guard; XY places the range sensor only.
        sole=float(position[2])+float(probe['home'][2])+sole_offset(probe['foot'])
        rows.append((name,sole))
        for dx,dy in ((0.,0.),(.05,0.),(-.05,0.),(0.,.05),(0.,-.05)):
            origin=nominal+np.array([dx,dy,.2])
            requests.append(origin)
    distances=ray.batch(requests,.6) if hasattr(ray,'batch') else [ray(origin,.6) for origin in requests]
    if len(distances)!=len(requests):raise ValueError('Incomplete terrain range frame')
    for i,(name,sole) in enumerate(rows):
        heights=[]
        for origin,distance in zip(requests[i*5:i*5+5],distances[i*5:i*5+5]):
            if distance is None or not math.isfinite(distance) or not 0<=distance<=.6:
                raise ValueError('Missing observed terrain support beneath '+name)
            heights.append(float(origin[2]-distance-sole))
        result[name]=dict(height_offset_m=heights[0],ahead_offset_m=max(heights),spread_m=max(heights)-min(heights))
    return dict(schema_version=1,clock_time=float(stamp),feet=result,
                source='Actual static-world downward collision rays at source nominal feet; native transforms place the sensor only')


class StaticTerrainRay:
    """Exact selected static SDF collision triangles; not a dynamic actor sensor."""
    def __init__(self, world):
        from robot_lab_utils.aerial_planning import StaticScene
        import trimesh
        self.surfaces=[]
        for shape in StaticScene(world).shapes:
            kind=shape['type']
            if kind in ('mesh','heightmap'):mesh=shape['surface']
            elif kind=='box':mesh=trimesh.creation.box(extents=shape['size'])
            elif kind=='plane':
                x,y=np.asarray(shape['size'])/2
                mesh=trimesh.Trimesh(vertices=[[-x,-y,0],[x,-y,0],[x,y,0],[-x,y,0]],faces=[[0,1,2],[0,2,3]],process=False)
            else:
                # Preserve analytic curved solids instead of approximating
                # their sensor surface by a polygonal cylinder/sphere.
                mesh=None
            if mesh is not None:_=mesh.triangles_tree
            self.surfaces.append((shape,mesh))

    def __call__(self,origin,length):
        distance=math.inf
        for shape,mesh in self.surfaces:
            p=(np.asarray(origin)-shape['position'])@shape['rotation']
            direction=np.array([0.,0.,-1.])@shape['rotation']
            if mesh is not None:
                hits,_,_=mesh.ray.intersects_location([p],[direction],multiple_hits=True)
                candidates=(np.asarray(hits).reshape(-1,3)-p)@direction
            elif shape['type']=='sphere':
                radius=shape['size'][0];b=float(p@direction);c=float(p@p)-radius**2
                disc=b*b-c
                candidates=[] if disc<0 else [-b-math.sqrt(disc),-b+math.sqrt(disc)]
            else:
                radius,height=shape['size'];candidates=[]
                a=float(direction[:2]@direction[:2]);b=float(p[:2]@direction[:2]);c=float(p[:2]@p[:2])-radius**2
                disc=b*b-a*c
                if a>1e-12 and disc>=0:
                    candidates=[v for v in ((-b-math.sqrt(disc))/a,(-b+math.sqrt(disc))/a) if abs(p[2]+v*direction[2])<=height/2]
                if abs(direction[2])>1e-12:
                    for z in (-height/2,height/2):
                        v=(z-p[2])/direction[2]
                        if np.linalg.norm(p[:2]+v*direction[:2])<=radius:candidates.append(v)
            for value in candidates:
                if 0<=value<=length:distance=min(distance,float(value))
        return distance if math.isfinite(distance) else None

    def batch(self,origins,length):
        origins=np.asarray(origins,dtype=float);distance=np.full(len(origins),math.inf)
        for shape,mesh in self.surfaces:
            if mesh is None:
                # Analytic curved primitives are uncommon in the support
                # scene; retain their exact intersection (no tessellation).
                probe=object.__new__(StaticTerrainRay);probe.surfaces=[(shape,mesh)]
                for i,origin in enumerate(origins):
                    value=probe(origin,length)
                    if value is not None:distance[i]=min(distance[i],value)
                continue
            p=(origins-np.asarray(shape['position']))@shape['rotation']
            direction=np.array([0.,0.,-1.])@shape['rotation']
            hits,index,_=mesh.ray.intersects_location(p,np.tile(direction,(len(p),1)),multiple_hits=True)
            if not len(hits):continue
            values=(np.asarray(hits)-p[index])@direction
            valid=(values>=0)&(values<=length)
            np.minimum.at(distance,np.asarray(index)[valid],values[valid])
        return [float(value) if math.isfinite(value) else None for value in distance]


class TerrainAdaptation:
    def __init__(self,names,maximum=.02):
        self.names=tuple(names);self.maximum=float(maximum)
        self.offsets={name:0. for name in names}
        self.last_stamp=None

    def step(self,message,clock):
        stamp=float(message['clock_time']);feet=message['feet']
        if (not math.isfinite(stamp) or abs(clock-stamp)>.4 or set(feet)!=set(self.names)):
            raise ValueError('Fresh complete source terrain observations required')
        if self.last_stamp is not None and stamp<self.last_stamp:
            raise ValueError('Terrain clock reset requires explicit re-arm')
        heights=[feet[name]['height_offset_m'] for name in self.names]
        rises=[feet[name]['ahead_offset_m']-feet[name]['height_offset_m'] for name in self.names]
        spreads=[feet[name]['spread_m'] for name in self.names]
        if not np.isfinite([heights,rises,spreads]).all() or min(rises)<-1e-8 or min(spreads)<0:
            raise ValueError('Finite physical terrain ranges required')
        if max(heights)-min(heights)>2*self.maximum or max(spreads)>self.maximum:
            raise ValueError('Observed relief/step exceeds source gait reach envelope')
        dt=0. if self.last_stamp is None else max(0.,min(.1,stamp-self.last_stamp))
        self.last_stamp=stamp
        median=float(np.median(heights))
        if abs(median)>.04:
            raise ValueError('Observed common support gap exceeds 4 cm; hold before a drop or loss of standing height')
        result={}
        for name,rise in zip(self.names,rises):
            wanted=float(feet[name]['height_offset_m']-median)
            self.offsets[name]+=max(-.02*dt,min(.02*dt,wanted-self.offsets[name]))
            result[name]=dict(offset=self.offsets[name],swing_extra=min(self.maximum,max(0.,float(rise))))
        return result
