"""Measured object transport using the UMI source's vertical position motor.

Other hands have fixed mounts and need an arm adapter. No pose, weld, gain,
force or transmission changes are made here.
"""
import time

import numpy as np


class NativeHandLift:
    def transport_contacts(self):
        result=self.external_finger_contacts()
        fingers=set().union(*self.object_sides)
        for index,contact in enumerate(self.data.contact[:self.data.ncon]):
            bodies=[int(self.model.geom_bodyid[g]) for g in contact.geom]
            if self.object_body not in bodies or any(body in fingers for body in bodies):continue
            force=np.zeros(6)
            self.node.mujoco.mj_contactForce(self.model,self.data,index,force)
            pair=tuple(sorted(map(int,contact.geom)))
            entry=result.setdefault(pair,dict(force=0.,distance=0.))
            entry['force']+=abs(float(force[0]));entry['distance']=min(entry['distance'],float(contact.dist))
        return result

    def lift_channel(self):
        if self.contract['adapter'] != 'umi_gripper':
            raise ValueError('This source hand needs an arm transport adapter')
        slot=next((i for i,c in enumerate(self.channels)
                   if c['name']=='gripper_joint_z_act' and c['units']=='m'),None)
        if slot is None:
            raise ValueError('Source vertical position channel missing')
        joint=int(self.model.actuator_trnid[self.channels[slot]['index'],0])
        axis=np.asarray(self.data.xaxis[joint])
        if not np.all(np.isfinite(axis)) or abs(axis[2])<.995:
            raise ValueError('Source lift axis is not vertical in this plant')
        return slot,joint,float(axis[2])

    def lift_object(self,request):
        try:
            if (self.fault or time.monotonic()-self.last_heartbeat>.8
                    or request.get('source_sha256')!=self.source_sha256
                    or request.get('contract_sha256')!=self.contract['contract_sha256']):
                raise ValueError('Fresh owned source/contract/heartbeat required')
            if self.object_body<0 or self.grasp.get('state')!='holding':
                raise ValueError('Establish a measured opposing-contact grasp first')
            slot,joint,axis_z=self.lift_channel()
            lowering=request['operation']=='hand_lower_object'
            if lowering:
                if not self.lift_run or self.lift_state['state']!='holding':
                    raise ValueError('Complete the source lift before lowering to its start')
                run=self.lift_run.copy()
                target=run['start_control']
            else:
                if self.lift_run:
                    raise ValueError('Lower or Stop the existing lift first')
                height=float(request['height_m'])
                if not np.isfinite(height) or not .02<=height<=.15:
                    raise ValueError('Lift height must be 0.02–0.15 m')
                if abs(float(self.data.qvel[int(self.model.jnt_dofadr[joint])]))>.01:
                    raise ValueError('Wait for the measured source lift axis to settle')
                start=float(self.target[slot]);target=start+height/axis_z
                run=dict(slot=slot,joint=joint,axis_z=axis_z,start_control=start,
                    start_measured=float(self.measured_controls()[slot]),
                    start_z=float(self.data.xpos[self.object_body,2]),height_m=height,
                    external=self.transport_contacts())
            low,high=self.channels[slot]['limits']
            if not low<=target<=high:
                raise ValueError('Lift exceeds original source motor bounds')
            # Retain the finger preload and PID state that produced the grasp.
            self.cancel_sequence('Object transport owns source hand targets')
            self.force_hold=None;self.force_state.update(state='interrupted',reason='Explicit object transport')
            self.guard=None
            run.update(target=target,started=time.monotonic(),settled_since=None,
                       last_sim=float(self.data.time),last_wall=time.monotonic())
            self.lift_run=run
            self.target[slot]=target
            self.lift_state=dict(state='lowering' if lowering else 'lifting',
                height_m=run['height_m'],reason='Awaiting measured object motion and sustained contacts')
            self.status='source object transport'
        except (ValueError,TypeError,KeyError,IndexError) as exc:
            self.hold('Rejected object transport: '+str(exc))

    def update_lift(self):
        if not self.lift_run:return
        run=self.lift_run
        try:
            now=time.monotonic();stamp=float(self.data.time)
            if self.fault or now-self.last_heartbeat>.8:
                raise ValueError('Object transport lost controller/operator heartbeat')
            if stamp<run['last_sim']:
                raise ValueError('Object transport simulation reset')
            if stamp>run['last_sim']:
                run['last_sim']=stamp;run['last_wall']=now
            if now-run['last_wall']>1.:
                raise ValueError('Object transport simulation clock stale')
            if self.lift_state['state'] in ('lifting','lowering') and now-run['started']>30.:
                raise ValueError('Object transport deadline exceeded')
            if self.grasp.get('state')!='holding':
                raise ValueError('Object grasp lost or slipped during transport')
            slot,joint,axis_z=self.lift_channel()
            if slot!=run['slot'] or joint!=run['joint'] or abs(axis_z-run['axis_z'])>.002:
                raise ValueError('Source lift axis changed')
            for pair,contact in self.transport_contacts().items():
                baseline=run['external'].get(pair,dict(force=0.,distance=0.))
                if contact['force']>baseline['force']*1.2+.2 or contact['distance']<min(-.002,baseline['distance']-.001):
                    raise ValueError('New/worsening external hand/object contact during transport')
            efforts=self.data.actuator_force[self.indices]
            if not np.all(np.isfinite(efforts)):
                raise ValueError('Non-finite source motor effort during transport')
            for index,force in zip(self.indices,efforts):
                if self.model.actuator_forcelimited[index]:
                    low,high=self.model.actuator_forcerange[index]
                    if not low-1e-6<=force<=high+1e-6:
                        raise ValueError('Measured source motor force exceeds its authored bounds')
            velocity=np.zeros(6)
            self.node.mujoco.mj_objectVelocity(self.model,self.data,self.node.mujoco.mjtObj.mjOBJ_BODY,
                                             self.object_body,velocity,0)
            height=float(self.data.xpos[self.object_body,2])-run['start_z']
            lowering=self.lift_state['state'] in ('lowering','supported')
            if self.lift_state['state']=='holding' and self.grasp['supported']:
                raise ValueError('Lifted object regained unexpected environmental support')
            if self.lift_state['state']=='holding' and abs(height-run['height_m'])>.01:
                raise ValueError('Object drifted from its measured lift height')
            if self.lift_state['state']=='supported' and not self.grasp['supported']:
                raise ValueError('Returned object lost environmental support')
            # Source position motors can retain a steady gravity error. Check
            # measured displacement from the settled start as well as actual
            # object height, without changing the motor gain or adding bias.
            error=abs(float(self.measured_controls()[slot])-run['start_measured']
                      -(run['target']-run['start_control']))
            settled=(error<=.003
                and np.linalg.norm(velocity[:3])<=.1 and np.linalg.norm(velocity[3:])<=.02
                and (self.grasp['supported'] and abs(height)<=.01 if lowering else
                     not self.grasp['supported'] and abs(height-run['height_m'])<=.005))
            if not settled:run['settled_since']=None
            elif run['settled_since'] is None:run['settled_since']=stamp
            if run['settled_since'] is not None and stamp-run['settled_since']>=1.:
                self.lift_state.update(state='supported' if lowering else 'holding',
                    reason='Measured supported return' if lowering else 'Measured unsupported lift and hold')
            self.lift_state.update(measured_height_m=height,source_displacement_error_m=error)
        except (ValueError,TypeError,KeyError,IndexError) as exc:
            self.hold('Object transport interrupted: '+str(exc))

