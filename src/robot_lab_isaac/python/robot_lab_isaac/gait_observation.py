"""PhysX contact tensors and collision-ray terrain sensing in the SDK process."""
import importlib.util
import math
import numpy as np


def rotation(quaternion):
    w,x,y,z=map(float,quaternion)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class GaitObservation:
    def __init__(self,stage,robot_path,world,probes,helper_path):
        from pxr import UsdGeom,UsdPhysics,Usd,PhysxSchema,PhysicsSchemaTools
        try:from isaacsim.core.prims import RigidPrim
        except ImportError:from omni.isaac.core.prims import RigidPrimView as RigidPrim
        try:from isaacsim.core.api.sensors import RigidContactView
        except ImportError:from omni.isaac.core.sensors import RigidContactView
        import carb
        import os
        try:
            from omni.physics.core import get_physics_scene_query_interface as scene_query
            self.unified_queries=True
        except ImportError:
            from omni.physx import get_physx_scene_query_interface as scene_query
            self.unified_queries=False
        self.probes=probes;self.stage=stage
        self.decode_path=PhysicsSchemaTools.intToSdfPath
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                PhysxSchema.PhysxSceneAPI.Apply(prim).CreateEnableSceneQuerySupportAttr(True)
        feet={probe['foot'] for probe in probes.values()}
        foot_paths=[str(prim.GetPath()) for prim in stage.Traverse() if prim.GetName() in feet and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(foot_paths)!=len(feet):raise ValueError('Unique actual source foot rigid bodies required')
        self.robot_path=os.path.commonpath(foot_paths)
        if self.robot_path in ('/','/World'):raise ValueError('Cannot identify exclusive source robot subtree')
        self.query=scene_query();self.carb=carb
        rigid=[prim for prim in stage.Traverse() if str(prim.GetPath()).startswith(self.robot_path+'/') and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.paths=[str(prim.GetPath()) for prim in rigid]
        filters=[str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI) and not str(prim.GetPath()).startswith(self.robot_path+'/')]
        if not self.paths or not filters:raise ValueError('Source gait requires actual rigid bodies and static collision filters')
        self.view=RigidPrim(prim_paths_expr=self.paths,name='source_gait_pose_view',reset_xform_properties=False)
        # SDK 6 may initialize the pose handle before its nested contact
        # view. Own and explicitly initialize the public contact view rather
        # than treating an uninitialized sensor count as a valid zero load.
        self.forces=RigidContactView(prim_paths_expr=self.paths,filter_paths_expr=[filters for _ in self.paths],
            name='source_gait_force_view',disable_stabilization=False,max_contact_count=max(1024,len(rigid)*64))
        world.scene.add(self.view)
        self.by_name={prim.GetName():str(prim.GetPath()) for prim in rigid}
        self.boxes={}
        # Hidden guide meshes still have physical collision extents.
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'],False,True)
        for name,probe in probes.items():
            path=self.by_name.get(probe['foot'])
            if not path:raise ValueError('Source gait foot rigid body missing: '+probe['foot'])
            parent=stage.GetPrimAtPath(path)
            boxes=[cache.ComputeRelativeBound(prim,parent).ComputeAlignedRange() for prim in stage.Traverse()
                   if str(prim.GetPath()).startswith(path+'/') and prim.HasAPI(UsdPhysics.CollisionAPI)]
            if not boxes:raise ValueError('Source foot collision shape missing: '+probe['foot'])
            lower=np.min([np.asarray(box.GetMin()) for box in boxes],axis=0);upper=np.max([np.asarray(box.GetMax()) for box in boxes],axis=0)
            if not np.isfinite([lower,upper]).all() or np.any(upper<lower):raise ValueError('Finite actual source collision extents required: '+probe['foot'])
            self.boxes[probe['foot']]=((lower+upper)/2,(upper-lower)/2)
        spec=importlib.util.spec_from_file_location('robot_lab_gait_terrain',helper_path)
        self.helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.helper)

    def initialize(self):
        self.view.initialize();self.indices={path:i for i,path in enumerate(self.view.prim_paths)}
        self.forces.initialize()
        if self.forces.num_shapes!=len(self.paths):raise ValueError('Complete actual source force sensor identity required')

    def update(self):
        self.positions,self.quaternions=map(np.asarray,self.view.get_world_poses())

    def pose(self,name):
        if name=='base_footprint' and name not in self.by_name:name='base_link'
        index=self.indices[self.by_name[name]]
        return self.positions[index],rotation(self.quaternions[index])

    def sole(self,name):
        _,matrix=self.pose(name);centre,half=self.boxes[name]
        return float(matrix[2]@centre-np.abs(matrix[2])@half)

    def ray(self,origin,length):
        distances=[];self.last_ray_hits=[]
        def report(hit):
            if self.unified_queries:
                path=str(self.decode_path(hit.rigid_body or getattr(hit,'collision',getattr(hit,'collider',0))));distance=float(hit.distance)
            else:
                path=str(hit.get('rigidBody') or hit.get('collision') or '');distance=float(hit['distance'])
            self.last_ray_hits.append((path,distance))
            if path!=self.robot_path and not path.startswith(self.robot_path+'/') and math.isfinite(distance) and 0<=distance<=length:distances.append(distance)
            return True
        arguments=[self.carb.Float3(*map(float,origin)),self.carb.Float3(0.,0.,-1.),float(length),report]
        self.query.raycast_all(*arguments,*([True] if self.unified_queries else []))
        return min(distances) if distances else None

    def contacts(self,stamp,dt):
        matrix=self.forces.get_contact_force_matrix(dt=dt)
        if matrix is None:raise ValueError('PhysX gait contact tensor unavailable')
        vector=np.asarray(matrix).sum(axis=1)
        if not np.isfinite(vector).all():raise ValueError('Non-finite PhysX contact observations')
        groups={self.by_name[probe['foot']]:name for name,probe in self.probes.items()}
        forces={name:0. for name in self.probes};forces['body']=0.
        for index,path in enumerate(self.paths):forces[groups.get(path,'body')]+=float(np.linalg.norm(vector[index]))
        return dict(schema_version=1,sim_time=stamp,clock_time=stamp,normal_forces_N=forces,
                    source='Actual PhysX static-collision filtered contact tensor, impulses divided by native physics dt')

    def terrain(self,stamp):
        self.update()
        return self.helper.sample_probes(self.probes,self.pose,self.sole,self.ray,stamp)

    def align_initial(self,robot,reference):
        self.update();lifts=[]
        for probe in self.probes.values():
            position=self.pose(probe['foot'])[0].copy();origin=position.copy();origin[2]=reference+.6
            distance=self.ray(origin,1.2)
            if distance is None:raise ValueError('No actual initial PhysX foot support: '+probe['foot']+' origin='+str(origin.tolist())+' hits='+str(self.last_ray_hits[:8]))
            lifts.append(origin[2]-distance-position[2]-self.sole(probe['foot'])+.002)
        if max(lifts)-min(lifts)>.04:raise ValueError('Initial PhysX gait support is too uneven')
        if not np.isfinite(lifts).all():raise ValueError('Non-finite initial PhysX support; placement refused')
        position,orientation=robot.get_world_pose();position=np.asarray(position).copy();position[2]+=max(lifts)
        robot.set_world_pose(position=position,orientation=orientation);robot.set_default_state(position=position,orientation=orientation)
