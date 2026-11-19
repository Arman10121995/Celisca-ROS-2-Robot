"""Measured object lift, hold and supported return for native hands.

UMI lifts with its source's vertical position motor. Fixed-mount hands
(Robotiq, DEX-EE) use the grasp fixture's lab support stage instead: the
pedestal drops, so the object is lifted relative to its support and hangs
in the fingers (``native_asset_display.SUPPORT_STAGE``). Heights are always
measured relative to the support. No pose, weld, gain, force or
transmission changes are made here.
"""
import time

import numpy as np

STAGE_RATE = .02            # m/s: the support stage moves no faster
STAGE_RETURN_FORCE = 2.     # N: most support force allowed while it returns
SUPPORT_LOSS_S = .1         # simulated s without support before a returned object counts as unsupported


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

    def stage_actuator(self):
        """The lab support stage's actuator index, or -1."""
        return int(self.node.mujoco.mj_name2id(self.model,self.node.mujoco.mjtObj.mjOBJ_ACTUATOR,'lab_support_stage'))

    # A lift axis is a source channel slot (UMI) or 'stage' (the support stage).
    def axis_target(self,slot):
        return self.stage_target if slot=='stage' else float(self.target[slot])

    def set_axis_target(self,slot,value):
        if slot=='stage':self.stage_target=float(value)
        else:self.target[slot]=value

    def axis_limits(self,slot):
        if slot=='stage':
            return tuple(float(v) for v in self.model.actuator_ctrlrange[self.stage_actuator()])
        return self.channels[slot]['limits']

    def axis_measured(self,slot):
        if slot=='stage':
            joint=int(self.model.actuator_trnid[self.stage_actuator(),0])
            return float(self.data.qpos[int(self.model.jnt_qposadr[joint])])
        return float(self.measured_controls()[slot])

    def pedestal_pair(self,pair):
        geom=self.node.mujoco.mj_name2id(self.model,self.node.mujoco.mjtObj.mjOBJ_GEOM,'manipulation_pedestal_geom')
        return geom in pair

    def support_z(self):
        body=self.node.mujoco.mj_name2id(self.model,self.node.mujoco.mjtObj.mjOBJ_BODY,'manipulation_pedestal')
        return float(self.data.xpos[body,2]) if body>=0 else 0.

    def lift_channel(self):
        if self.contract['adapter'] != 'umi_gripper':
            actuator=self.stage_actuator()
            if actuator<0:
                raise ValueError('This hand needs the grasp fixture support stage or an arm transport adapter')
            joint=int(self.model.actuator_trnid[actuator,0])
            # The support moves down to lift the object relative to it.
            return 'stage',joint,-1.
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
                start=self.axis_target(slot);target=start+height/axis_z
                run=dict(slot=slot,joint=joint,axis_z=axis_z,start_control=start,
                    start_measured=self.axis_measured(slot),
                    start_z=float(self.data.xpos[self.object_body,2]),start_support_z=self.support_z(),
                    height_m=height,mode='support_stage' if slot=='stage' else 'source_axis',
                    external=self.transport_contacts())
            low,high=self.axis_limits(slot)
            if not low<=target<=high:
                raise ValueError('Lift exceeds original source motor bounds')
            # Retain the finger preload and PID state that produced the grasp.
            self.cancel_sequence('Object transport owns source hand targets')
            self.force_hold=None;self.force_state.update(state='interrupted',reason='Explicit object transport')
            self.guard=None
            run.update(target=target,started=time.monotonic(),settled_since=None,
                       last_sim=float(self.data.time),last_wall=time.monotonic())
            self.lift_run=run
            if slot!='stage':      # the support stage ramps there (update_lift)
                self.set_axis_target(slot,target)
            self.lift_state=dict(state='lowering' if lowering else 'lifting',mode=run['mode'],
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
            delta=max(0.,min(.05,stamp-run['last_sim']))
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
            stage=run.get('mode')=='support_stage'
            if stage:
                step=STAGE_RATE*delta
                self.stage_target+=max(-step,min(step,run['target']-self.stage_target))
                if self.lift_state['state']=='lowering' and self.grasp['supported']:
                    # Returning support reached the object: stop pressing up.
                    run['target']=self.stage_target
            for pair,contact in self.transport_contacts().items():
                if stage and self.lift_state['state'] in ('lowering','supported') and self.pedestal_pair(pair):
                    # The returning support is expected; it must not press hard.
                    if contact['force']>STAGE_RETURN_FORCE:
                        raise ValueError('Returning support stage pressed the object with %.1f N'%contact['force'])
                    continue
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
            # Object height relative to its support (a support stage moves it).
            height=(float(self.data.xpos[self.object_body,2])-run['start_z']
                    -(self.support_z()-run.get('start_support_z',self.support_z())))
            lowering=self.lift_state['state'] in ('lowering','supported')
            # Source friction can stop the free object ~1 mm above its old
            # support. Seek real support slowly, at most 3 mm below the old
            # source setpoint; never infer support merely from a height.
            if self.lift_state['state']=='lowering' and not self.grasp['supported'] and abs(height)<.01:
                seated=(run['start_control']-run['target'])*axis_z
                extra=min(max(0.,.003-seated),.002*delta)
                target=run['target']-extra/axis_z
                low,high=self.axis_limits(slot)
                if not low<=target<=high:raise ValueError('Support seating exceeds source motor bounds')
                run['target']=target
                if slot!='stage':   # the stage ramps there at STAGE_RATE; a jump struck the DEX-EE object (2026-11-19)
                    self.set_axis_target(slot,target)
                self.lift_state['support_seating_m']=(run['start_control']-target)*axis_z
            if self.lift_state['state']=='holding' and self.grasp['supported']:
                raise ValueError('Lifted object regained unexpected environmental support')
            if self.lift_state['state']=='holding' and abs(height-run['height_m'])>.01:
                raise ValueError('Object drifted from its measured lift height')
            # A firm tripod's position servos slowly take the whole weight and
            # lift the returned box microns off the stage (DEX-EE, 2026-11-19).
            # After a debounce the stage resumes its slow seating inside the
            # same 3 mm budget; only a spent budget is a loss.
            if self.lift_state['state']=='supported' and not self.grasp['supported']:
                if run.get('support_lost_since') is None:run['support_lost_since']=stamp
                if stamp-run['support_lost_since']>=SUPPORT_LOSS_S:
                    seated=(run['start_control']-run['target'])*axis_z
                    if not stage or seated>=.003-1e-6:
                        raise ValueError('Returned object lost environmental support')
                    self.lift_state.update(state='lowering',reason='Reseating the returned object on its support')
                    run['settled_since']=run['support_lost_since']=None
            else:run['support_lost_since']=None
            # Source position motors can retain a steady gravity error. Check
            # measured displacement from the settled start as well as actual
            # object height, without changing the motor gain or adding bias.
            error=abs(self.axis_measured(slot)-run['start_measured']
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

