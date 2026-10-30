"""Source-bound 2.5D shore routes over real support geometry and water.

The plant follows the slope through wheel contacts/buoyancy, never a Z command.
Only explicitly declared first-party terrain is excluded from obstacle shapes.
"""
import copy
import math
import xml.etree.ElementTree as ET
import numpy as np
from robot_lab_utils.aerial_planning import StaticScene,plan
from robot_lab_utils.observed_aerial_scene import ObservedScene


class ShoreScene:
    def __init__(self,world,surface=0.):
        if ET.parse(world).getroot().find('world').get('name')!='amphibious_shore':
            raise ValueError('Shore route requires the declared amphibious_shore terrain')
        original=StaticScene(world);self.terrain=copy.copy(original);self.obstacles=copy.copy(original)
        names={'seafloor','west_bank','shore_ramp'}
        self.terrain.shapes=[s for s in original.shapes if s['model'] in names]
        if {s['model'] for s in self.terrain.shapes}!=names:raise ValueError('Declared shore support geometry missing')
        if any(s['type']!='box' for s in self.terrain.shapes):raise ValueError('Declared shore supports must retain their source box geometry')
        ramp=next(s for s in self.terrain.shapes if s['model']=='shore_ramp')
        normal=ramp['rotation'][:,2]
        if normal[2]<math.cos(.3):raise ValueError('Authored ramp exceeds the rover slope envelope')
        self.obstacles.shapes=[s for s in original.shapes if s['model'] not in names]
        self.surface=surface;self.cache={}

    def support_height(self,x,y):
        key=(round(float(x),5),round(float(y),5))
        if len(self.cache)>=20000:self.cache.clear()
        if key not in self.cache:
            hits=[]
            for shape in self.terrain.shapes:
                origin=shape['rotation'].T@(np.array([*key,3.])-shape['position'])
                direction=shape['rotation'].T@np.array([0.,0.,-1.]);half=np.asarray(shape['size'])/2
                near,far=0.,math.inf
                for i in range(3):
                    if abs(direction[i])<1e-10:
                        if abs(origin[i])>half[i]:far=-1.;break
                    else:
                        first,second=sorted(((-half[i]-origin[i])/direction[i],(half[i]-origin[i])/direction[i]))
                        near=max(near,first);far=min(far,second)
                if far>=near:hits.append(3.-near)
            if not hits:raise ValueError('Goal/path has no source support or water region')
            self.cache[key]=max(hits)
        return self.cache[key]

    def height(self,x,y):
        # Source wheel radius + axle offset, and declared calm-water draft.
        return max(self.support_height(x,y)+.34,self.surface+.134)

    def with_observation(self,observation):
        scene=copy.copy(self)
        if observation is not None:scene.obstacles=ObservedScene(self.obstacles,observation)
        return scene

    def above_support(self,points,tolerance=.02):
        """Exclude only returns matching the declared terrain, before voxels.

        Unknown regions and points above/below the 2 cm support band remain
        obstacles. This is map-ground classification, not measured clearance.
        """
        points=np.asarray(points,dtype=float);top=np.full(len(points),-math.inf)
        for shape in self.terrain.shapes:
            origins=np.column_stack((points[:,:2],np.full(len(points),3.)))
            origins=(origins-shape['position'])@shape['rotation']
            direction=shape['rotation'].T@np.array([0.,0.,-1.]);half=np.asarray(shape['size'])/2
            near=np.zeros(len(points));far=np.full(len(points),math.inf);valid=np.ones(len(points),dtype=bool)
            for i in range(3):
                if abs(direction[i])<1e-10:valid&=np.abs(origins[:,i])<=half[i]
                else:
                    first=(-half[i]-origins[:,i])/direction[i];second=(half[i]-origins[:,i])/direction[i]
                    near=np.maximum(near,np.minimum(first,second));far=np.minimum(far,np.maximum(first,second))
            valid&=far>=near;top[valid]=np.maximum(top[valid],3.-near[valid])
        return points[(~np.isfinite(top)) | (np.abs(points[:,2]-top)>tolerance)]

    def clearance(self,point):
        try:return self.obstacles.clearance((point[0],point[1],self.height(point[0],point[1])))
        except ValueError:return 0.

    segment=StaticScene.segment

    def route(self,start,target,radius,canceled,observation=None):
        self.height(target[0],target[1])
        path,info=plan(self.with_observation(observation),[start[0],start[1],0.],[target[0],target[1],0.],radius=radius,resolution=.4,
            min_altitude=0.,max_altitude=0.,timeout=15.,canceled=canceled)
        route=[[p[0],p[1],self.height(p[0],p[1])] for p in path]
        info.update(terrain='Declared actual source bank/ramp/seafloor; source-derived wheel height and calm-water draft prior',
            motion='Physical wheel/thrust following; no commanded Z or pose changes')
        return route,info
