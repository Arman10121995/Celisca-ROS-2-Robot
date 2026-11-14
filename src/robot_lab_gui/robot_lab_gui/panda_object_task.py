"""Explicitly started Panda pick/place cycles using existing measured interfaces."""
import copy
import json
import math
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
from robot_lab_utils.guarded_sequence import GuardedSequence
from robot_lab_utils.object_custody import ObjectCustody
from robot_lab_utils.grasp_candidates import support_placement,release_on_support


STAGES=['open','plan_approach','move','plan_grasp','move','close','attach','plan_lift','move',
        'grasp_check','plan_above','move','plan_lower','move','release','release_check','detach','plan_retreat','move']


class PandaObjectTask(ttk.LabelFrame):
    def __init__(self,parent,cartesian):
        super().__init__(parent,text='Automatic pick / place · experimental',padding=6)
        self.cart,self.arm,self.app=cartesian,cartesian.arm,cartesian.arm.app
        self.grasp=cartesian.grasp_targets;self.engine=GuardedSequence(self);self.dispatching=False
        self.cycles=tk.IntVar(value=1);self.feedback=tk.StringVar(value='Select a grasp candidate and a support above; Run authorizes the complete sequence.')
        ttk.Label(self,text='Cycles').grid(row=0,column=0,sticky='w')
        ttk.Spinbox(self,textvariable=self.cycles,from_=1,to=5,width=5).grid(row=0,column=1,sticky='w')
        self.run_button=ttk.Button(self,text='Run pick / place',command=self.run)
        self.run_button.grid(row=1,column=0,sticky='ew',pady=4)
        self.stop_button=ttk.Button(self,text='Stop task',command=self.cancel)
        self.stop_button.grid(row=1,column=1,sticky='ew',pady=4)
        ttk.Label(self,textvariable=self.feedback,wraplength=280).grid(row=2,column=0,columnspan=2,sticky='w')
        ttk.Label(self,text='Plans and executes each stage through the existing native actions. '
            'Lift/release need sustained physical contact, support and slip checks. '
            'Manual controls, selection changes, Stop or stale feedback interrupt the task.',wraplength=280).grid(
                row=3,column=0,columnspan=2,sticky='w')
        self.columnconfigure((0,1),weight=1);self.log_path=None;self.carry=False

    def run(self):
        hand=getattr(self.app,'hand_tab',None)
        if self.engine.active or not self.grasp.ready() or hand is None or not hand.ready():return
        try:
            cycles=int(self.cycles.get());index=self.grasp.selector.current()
            if not 1<=cycles<=5 or not 0<=index<len(self.grasp.candidates):raise ValueError('Choose a candidate and 1–5 cycles')
            if self.cart.scene.get('object_attached') or self.cart.scene.get('grasp_contacts_ready'):
                raise ValueError('Start with a released, detached object and idle arm/hand')
            if self.arm.state.get('busy') or hand.state.get('busy') or self.arm.sending or hand.sending:
                raise ValueError('Wait for idle measured arm/hand')
            if not self.grasp.object.get('custody'):raise ValueError('The running source needs physical object custody observations')
            self.support=self.grasp.support_choice.get()
            if self.support not in self.grasp.catalog.get('supports',{}):raise ValueError('Choose a measured placement support')
            self.candidate=self.grasp.candidates[index]['id'];self.strategy=self.grasp.strategy.get()
            self.identity={k:self.grasp.object[k] for k in ('id','plant_id','source_sha256')}
            self.dimensions=list(self.grasp.object['size']);self.force=float(hand.force_var.get())
            self.release_gap=float(self.grasp.release_gap.get())
            if not math.isfinite(self.force) or not .1<=self.force<=20:raise ValueError('Use the original 0.1–20 N force range')
            if not math.isfinite(self.release_gap) or not .003<=self.release_gap<=.15:raise ValueError('Use a 3–150 mm release gap')
            root=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))
            directory=root/'panda-object-tasks';directory.mkdir(parents=True,exist_ok=True)
            if not directory.resolve().is_relative_to('/workspace') or directory.stat().st_dev==Path('/').stat().st_dev:
                raise ValueError('Object task journals require the mounted SSD')
            self.log_path=directory/(uuid.uuid4().hex+'.jsonl');self.hand=hand;self.carry=False;self.retreat=None
            self.record(operation='start',identity=self.identity,candidate=self.candidate,support=self.support,cycles=cycles,
                        force_n=self.force,scope='Explicit measured-stage task, not a mission certificate')
            self.engine.start(STAGES*cycles)
        except (ValueError,KeyError,TypeError,tk.TclError,OSError) as exc:self.feedback.set(str(exc))

    def record(self,**data):
        if self.log_path:
            with self.log_path.open('a') as stream:
                stream.write(json.dumps(dict(wall_s=time.monotonic(),**data),allow_nan=False)+'\n')

    def cancel(self,reason='Operator stopped object task'):
        self.engine.cancel(reason)

    def stop(self,reason):
        self.carry=False
        self.cart.invalidate();self.arm.stop();self.hand.stop()
        self.feedback.set(reason)
        try:self.record(operation='interrupted',reason=reason)
        except OSError:pass

    def guard(self):
        if not self.arm.ready() or not self.hand.ready() or not self.cart.owned():raise ValueError('Owned arm/hand feedback lost')
        if time.monotonic()-self.cart.scene_received>2. or time.monotonic()-self.cart.received>.5:
            raise ValueError('Planning scene/joint feedback is stale')
        obj=self.grasp.object
        if obj is None or time.monotonic()-self.grasp.received>.8 or time.monotonic()-self.grasp.catalog_received>.8:
            raise ValueError('Physical object/catalog feedback is stale')
        if any(obj.get(k)!=v for k,v in self.identity.items()) or obj['size']!=self.dimensions:
            raise ValueError('Object source, plant, identity or dimensions changed')
        if (self.grasp.object_choice.get()!=self.identity['id'] or self.grasp.support_choice.get()!=self.support
                or self.grasp.strategy.get()!=self.strategy or float(self.hand.force_var.get())!=self.force
                or float(self.grasp.release_gap.get())!=self.release_gap):
            raise ValueError('Task selection or force/gap changed; restart deliberately')
        selection=self.grasp.selector.current()
        if not 0<=selection<len(self.grasp.candidates) or self.grasp.candidates[selection]['id']!=self.candidate:
            raise ValueError('Grasp candidate changed during object task')
        if self.arm.state.get('contact_blocked'):raise ValueError('Native arm contact interruption')
        if self.carry:
            measured=obj['custody']
            if min(measured['left_force'],measured['right_force'])<.1 or math.dist(measured['relative'],self.anchor)>.01:
                raise ValueError('Measured grasp contact/slip lost during transfer')
            angle=2*math.acos(float(np.clip(abs(np.dot(self.grasp.object['relative_hand_orientation_wxyz'],self.anchor_orientation)),0.,1.)))
            if angle>.15:raise ValueError('Measured object rotation slipped during transfer')

    def dispatch(self,callback,*args):
        self.dispatching=True
        try:return callback(*args)
        finally:self.dispatching=False

    @staticmethod
    def pose(data):
        from geometry_msgs.msg import Pose
        result=Pose()
        for axis,value in zip('xyz',data['position']):setattr(result.position,axis,float(value))
        for axis,value in zip('wxyz',data['orientation_wxyz']):setattr(result.orientation,axis,float(value))
        return result

    def begin(self,stage):
        if stage in ('open','close','release'):
            if self.hand.sending or self.hand.state.get('busy') or self.arm.state.get('busy'):return None
            if stage=='release':self.carry=False
            serial=self.hand.completion_serial
            self.dispatch(self.hand.command,0. if stage=='close' else .08)
            return ('hand',serial) if self.hand.sending else None
        if stage=='move':
            if not self.cart.ready():return None
            if not self.cart.points:raise ValueError('Collision plan expired or was invalidated; restart the task')
            serial=self.arm.completion_serial;self.dispatch(self.cart.execute)
            return ('arm',serial) if self.arm.sending else None
        if stage in ('attach','detach'):
            desired=stage=='attach'
            if bool(self.cart.scene.get('object_attached'))==desired and self.cart.scene.get('ready'):return ('already',True)
            if not self.cart.ready() or desired and not self.cart.scene.get('grasp_contacts_ready'):return None
            self.dispatch(self.cart.object_command,stage)
            return ('scene',desired)
        if stage in ('grasp_check','release_check'):return ('custody',ObjectCustody())
        if not self.cart.ready() or self.hand.sending or self.hand.state.get('busy'):return None
        if stage in ('plan_approach','plan_grasp'):
            self.grasp.generate()
            index=next((i for i,c in enumerate(self.grasp.candidates) if c['id']==self.candidate),None)
            if index is None:raise ValueError('Selected grasp no longer fits the measured object')
            self.grasp.selector.current(index)
            self.dispatch(self.grasp.plan,stage.removeprefix('plan_'))
        elif stage=='plan_lift':
            if not self.cart.scene.get('object_attached') or not self.cart.scene.get('grasp_contacts_ready'):return None
            obj=self.grasp.object;attachment=self.cart.scene['object_in_tool']
            rotation=Rotation.from_quat(np.asarray(obj['orientation_wxyz'])[[1,2,3,0]])
            local=Rotation.from_quat(np.asarray(attachment['orientation_wxyz'])[[1,2,3,0]])
            tool=rotation*local.inv()
            position=np.asarray(obj['position'])-tool.apply(attachment['position'])+[0.,0.,.1]
            self.dispatch(self.cart.plan_pose,self.pose(dict(position=position,
                orientation_wxyz=tool.as_quat()[[3,0,1,2]].tolist())),'Task lift')
        elif stage in ('plan_above','plan_lower'):
            result=support_placement(self.grasp.object,self.cart.scene.get('object_in_tool'),
                self.grasp.catalog['supports'][self.support],clearance=self.release_gap,
                edge_margin=.01+float(self.cart.scene.get('link_padding_m',.01)))
            label=stage.removeprefix('plan_')
            if label=='lower':
                self.retreat=copy.deepcopy(result['poses']['above'])
                self.expected_release=np.asarray(result['object_center'])-[0.,0.,self.release_gap]
            self.dispatch(self.cart.plan_pose,self.pose(result['poses'][label]),'Task placement '+label)
        elif stage=='plan_retreat':
            if self.cart.scene.get('object_attached') or self.cart.scene.get('grasp_contacts_ready'):return None
            if self.retreat is None:raise ValueError('Placement retreat has no bound target')
            self.dispatch(self.cart.plan_pose,self.pose(self.retreat),'Task post-release retreat')
        else:raise ValueError('Unknown object task stage '+stage)
        return ('plan',self.cart.generation) if self.cart.planning else None

    def observe(self,stage,token):
        kind,value=token
        if kind=='already':return 'succeeded','Scene already acknowledged'
        if kind in ('arm','hand'):
            control=self.arm if kind=='arm' else self.hand
            if control.completion_serial==value:return 'pending','Waiting for action result'
            answer=control.last_result
            if not answer.get('ok') or kind=='hand' and stage!='close' and not answer.get('reached'):
                return 'failed','Native action failed: '+str(answer)
            return 'succeeded',dict(native_action=answer,physical_object=self.grasp.object)
        if kind=='plan':
            if self.cart.generation!=value:return 'failed','Planning generation changed'
            if self.cart.planning:return 'pending','Waiting for collision-aware IK/OMPL'
            return ('succeeded','Collision plan accepted') if self.cart.points else ('failed',self.cart.status.get())
        if kind=='scene':
            if self.cart.scene.get('object_error'):return 'failed',self.cart.scene['object_error']
            return ('succeeded','Scene acknowledged') if self.cart.scene.get('ready') and bool(self.cart.scene.get('object_attached'))==value else ('pending','Scene ACK pending')
        if kind=='custody':
            measured=self.grasp.object['custody'];released=stage=='release_check'
            if released and np.linalg.norm(np.asarray(self.grasp.object['position'])-self.expected_release)>.03:
                return 'failed','Object released away from the selected support target'
            if released and not release_on_support(self.grasp.object,self.grasp.catalog['supports'][self.support]):
                return 'failed','Released object does not fit on the selected measured support'
            if not value.observe(**measured,released=released):return 'pending','Waiting for sustained physical custody/support'
            if not released:
                self.carry=True;self.anchor=list(measured['relative'])
                self.anchor_orientation=list(self.grasp.object['relative_hand_orientation_wxyz'])
            return 'succeeded',dict(condition='released' if released else 'grasped',measurement=measured)
        return 'failed','Unknown object stage receipt'

    def poll(self):
        self.engine.tick()
        if self.engine.active:
            self.feedback.set('%d/%d · %s'%(self.engine.index+1,len(self.engine.steps),self.engine.steps[self.engine.index]))
        elif self.engine.state!='idle':self.feedback.set(self.engine.state+' · '+self.engine.reason)
        hand=getattr(self.app,'hand_tab',None)
        self.run_button.state(['!disabled'] if self.grasp.ready() and hand is not None and hand.ready()
            and self.grasp.candidates and not self.engine.active else ['disabled'])
        self.stop_button.state(['!disabled'] if self.engine.active else ['disabled'])
