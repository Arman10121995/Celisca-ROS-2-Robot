"""Opt-in measured normal forces; planned gait phases are never contacts."""
import json
import numpy as np
import mujoco


class GaitContacts:
    def terrain(self,model,data,stamp):
        from robot_lab_utils.gait_terrain import sample_probes
        original=model.geom_group.copy();model.geom_group[model.geom_bodyid>0]=5
        mask=np.array([1,1,0,1,1,0],dtype=np.uint8)
        def body(name):
            index=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
            if index<0 and name=='base_footprint':index=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
            if index<0 and name in ('base_link','base_footprint'):
                # The importer names the massive source root <robot>_base.
                # These configured roots share that source trunk frame.
                index=int(model.geom_bodyid[self.groups['body'][0]])
            if index<0:raise ValueError('Source probe body missing: '+name)
            return index
        def pose(name):
            index=body(name)
            return data.xpos[index],data.xmat[index].reshape(3,3)
        def sole(name):
            group=next(key for key,probe in self.probes.items() if probe['foot']==name)
            offsets=[]
            for geom in self.groups[group]:
                bounds=model.geom_aabb[geom];row=data.geom_xmat[geom].reshape(3,3)[2]
                # These source feet have an origin-centred spherical collider;
                # the importer may fuse its fixed link while retaining its geom.
                if model.geom_type[geom]!=mujoco.mjtGeom.mjGEOM_SPHERE:raise ValueError('Source terrain sole needs a reviewed collider-frame adapter')
                offsets.append(float(-model.geom_size[geom,0]))
            return min(offsets)
        def ray(origin,length):
            hit=np.array([-1],dtype=np.int32)
            distance=mujoco.mj_ray(model,data,np.asarray(origin),np.array([0.,0.,-1.]),mask,1,-1,hit)
            return float(distance) if 0<=distance<=length else None
        try:return sample_probes(self.probes,pose,sole,ray,stamp)
        finally:model.geom_group[:]=original

    def __init__(self,model,config):
        self.groups=json.loads(config)
        if not isinstance(self.groups,dict) or not 1<=len(self.groups)<=64:raise ValueError('Named contact groups required')
        self.groups={name:[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,geom) for geom in geoms]
                     for name,geoms in self.groups.items()}
        if any(not values or min(values)<0 for values in self.groups.values()):raise ValueError('Configured gait contact geometry is missing')
        self.ids={geom:name for name,values in self.groups.items() for geom in values}
        if len(self.ids)!=sum(map(len,self.groups.values())):raise ValueError('Contact groups must be disjoint')
        self.static_flex=set()
        for flex in range(model.nflex):
            start,count=model.flex_vertadr[flex],model.flex_vertnum[flex]
            if np.all(model.flex_vertbodyid[start:start+count]==0):self.static_flex.add(flex)

    def measure(self,model,data):
        forces={name:0. for name in self.groups};wrench=np.zeros(6)
        for i in range(data.ncon):
            contact=data.contact[i]
            for side,(own,other) in enumerate(((contact.geom1,contact.geom2),(contact.geom2,contact.geom1))):
                if own not in self.ids:continue
                world=(other>=0 and model.geom_bodyid[other]==0)
                if other<0:world=int(contact.flex[1-side]) in self.static_flex
                if world:
                    mujoco.mj_contactForce(model,data,i,wrench)
                    forces[self.ids[own]]+=max(0.,float(wrench[0]))
                break
        return dict(schema_version=1,sim_time=float(data.time),normal_forces_N=forces,
                    source='Actual MuJoCo static-world normal contacts, including rigid world flexes')

    def align_spawn(self,model,data,root_address,reference_z):
        """Initial/reset placement on actual static surfaces beneath the feet.

        Exclude the whole robot from range tests; querying through its own
        hull/feet would turn the robot into its own floor. No motion-time
        alignment occurs. Uneven initial support is refused by this flat gait.
        """
        original=model.geom_group.copy();model.geom_group[model.geom_bodyid>0]=5
        mask=np.array([1,1,0,1,1,0],dtype=np.uint8)
        lifts=[];floors=[]
        try:
            for name,ids in self.groups.items():
                if name=='body':continue
                geom=ids[0];position=data.geom_xpos[geom]
                origin=np.array([position[0],position[1],reference_z+.6]);hit=np.array([-1],dtype=np.int32)
                distance=mujoco.mj_ray(model,data,origin,np.array([0.,0.,-1.]),mask,1,-1,hit)
                if distance<0:raise ValueError('No actual support surface beneath '+name)
                floor=origin[2]-distance
                bounds=model.geom_aabb[geom];zrow=data.geom_xmat[geom].reshape(3,3)[2]
                bottom=position[2]+zrow@bounds[:3]-abs(zrow)@bounds[3:]
                floors.append(floor);lifts.append(floor+.002-bottom)
            if max(floors)-min(floors)>.04:raise ValueError('Initial footholds differ by more than 4 cm; choose a flatter spawn')
            data.qpos[root_address+2]+=max(lifts)
            mujoco.mj_forward(model,data)
            return dict(floor_min_m=min(floors),floor_max_m=max(floors),root_adjustment_m=max(lifts))
        finally:model.geom_group[:]=original
