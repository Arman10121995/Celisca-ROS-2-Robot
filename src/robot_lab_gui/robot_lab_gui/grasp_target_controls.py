"""Measured-object geometry candidates passed to the existing MoveIt planner."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk
from robot_lab_utils.grasp_candidates import top_grasps, side_grasps, support_placement


class GraspTargetControls(ttk.LabelFrame):
    def __init__(self,parent,cartesian):
        super().__init__(parent,text='Object grasp targets · experimental',padding=6)
        self.cartesian=cartesian;self.object=None;self.received=-math.inf
        self.catalog={};self.catalog_received=-math.inf;self.selection_publisher=None
        self.object_choice=tk.StringVar()
        self.candidates=[];self.choice=tk.StringVar();self.feedback=tk.StringVar(value='Waiting for a measured object pose.')
        self.object_selector=ttk.Combobox(self,textvariable=self.object_choice,state='readonly')
        self.object_selector.grid(row=0,column=0,columnspan=2,sticky='ew')
        self.select_button=ttk.Button(self,text='Select object',command=self.select_object)
        self.select_button.grid(row=0,column=2,sticky='ew')
        self.strategy=tk.StringVar(value='Top')
        self.strategy_selector=ttk.Combobox(self,textvariable=self.strategy,state='readonly',width=9,
            values=('Top','Side','Top + side'))
        self.strategy_selector.grid(row=1,column=0,sticky='ew')
        self.strategy_selector.bind('<<ComboboxSelected>>',self.strategy_changed)
        self.selector=ttk.Combobox(self,textvariable=self.choice,state='readonly')
        self.selector.grid(row=1,column=1,columnspan=2,sticky='ew')
        self.refresh_button=ttk.Button(self,text='Generate candidates',command=self.generate)
        self.refresh_button.grid(row=2,column=0,columnspan=2,sticky='ew',pady=3)
        self.opening_button=ttk.Button(self,text='Set gap field',command=self.set_gap)
        self.opening_button.grid(row=2,column=2,sticky='ew')
        self.buttons=[]
        for index,stage in enumerate(('approach','grasp','lift','retreat')):
            button=ttk.Button(self,text='Plan '+stage,command=lambda phase=stage:self.plan(phase))
            column=index%2
            button.grid(row=3+index//2,column=column,columnspan=1 if column==0 else 2,sticky='ew',padx=2)
            self.buttons.append(button)
        ttk.Label(self,textvariable=self.feedback,wraplength=280).grid(row=6,column=0,columnspan=3,sticky='w')
        ttk.Label(self,text='Simulator-truth object pose. Geometry candidates are not grasp certificates. '
                  'Open the Hand first. These buttons only plan; Execute Plan starts motion. '
                  'Close and confirm contacts before planning attachment/lift. Retreat requires an open, detached hand.',wraplength=280).grid(row=5,column=0,columnspan=3,sticky='w')
        ttk.Separator(self).grid(row=7,column=0,columnspan=3,sticky='ew',pady=6)
        ttk.Label(self,text='Place on a measured support surface',wraplength=280).grid(row=8,column=0,columnspan=3,sticky='w')
        self.support_choice=tk.StringVar()
        self.support_selector=ttk.Combobox(self,textvariable=self.support_choice,state='readonly')
        self.support_selector.grid(row=9,column=0,columnspan=3,sticky='ew')
        self.release_gap=tk.DoubleVar(value=.03)
        ttk.Label(self,text='Release gap above surface (m)').grid(row=10,column=0,columnspan=2,sticky='w')
        ttk.Spinbox(self,textvariable=self.release_gap,from_=.003,to=.15,increment=.005,width=7).grid(row=10,column=2)
        self.place_buttons=[]
        for column,stage,label in ((0,'above','Plan above surface'),(1,'lower','Plan lower')):
            button=ttk.Button(self,text=label,command=lambda phase=stage:self.plan_placement(phase))
            button.grid(row=11,column=column,columnspan=1 if column==0 else 2,sticky='ew',pady=3)
            self.place_buttons.append(button)
        ttk.Label(self,text='Uses the acknowledged grasp offset. These buttons only plan. '
                  'Execute each plan, open Hand to release, then use a positive Z step to retreat. '
                  'A release gap leaves the object to settle under gravity.',wraplength=280).grid(row=12,column=0,columnspan=3,sticky='w')
        self.columnconfigure((0,1,2),weight=1)

    def receive_catalog(self,message):
        try:
            if len(message.data)>2*1024*1024:return
            data=json.loads(message.data)
            if (not isinstance(data,dict) or not isinstance(data.get('objects'),list)
                    or len(data['objects'])>256 or not data.get('plant_id') or not data.get('source_sha256')):
                return
            if (any(not isinstance(name,str) for name in data['objects'])
                    or not isinstance(data.get('supports',{}),dict) or len(data.get('supports',{}))>256):return
            if data.get('plant_id')!=self.catalog.get('plant_id'):
                self.candidates=[];self.object=None
            self.catalog=data;self.catalog_received=time.monotonic()
            self.object_selector.configure(values=data['objects'])
            supports=list(data.get('supports',{}))
            self.support_selector.configure(values=supports)
            if self.support_choice.get() not in supports:self.support_choice.set(supports[0] if supports else '')
            if self.object_choice.get() not in data['objects']:
                self.object_choice.set(data.get('selected') if data.get('selected') in data['objects'] else
                                       data['objects'][0] if data['objects'] else '')
            if data.get('error'):self.feedback.set(data['error'])
            elif not data['objects']:self.feedback.set('No selectable free box; compound/mesh adapters remain pending.')
        except (ValueError,TypeError):pass

    def select_object(self):
        if (not self.cartesian.owned() or not self.cartesian.arm.ready() or self.selection_publisher is None
                or time.monotonic()-self.catalog_received>.8 or self.cartesian.planning
                or self.cartesian.scene.get('object_attached')):return
        name=self.object_choice.get()
        if name not in self.catalog.get('objects',[]):return
        from std_msgs.msg import String
        self.cartesian.invalidate();self.candidates=[];self.object=None
        self.selection_publisher.publish(String(data=json.dumps(dict(plant_id=self.catalog['plant_id'],
            source_sha256=self.catalog['source_sha256'],id=name))))
        self.feedback.set('Requested measured-object selection; waiting for the acknowledged scene.')

    def receive(self,message):
        try:
            if len(message.data)>2*1024*1024:
                raise ValueError('Object geometry message exceeds the input bound')
            obj=json.loads(message.data)
            top_grasps(obj)
            self.object=obj;self.received=time.monotonic()
        except (ValueError,TypeError,KeyError):
            self.object=None;self.candidates=[]

    def ready(self):
        return (self.object is not None and time.monotonic()-self.received<.8 and self.cartesian.ready()
                and not getattr(self.cartesian,'planning',False) and not getattr(self.cartesian,'reading_pose',False)
                and self.object.get('plant_id')==self.catalog.get('plant_id')
                and self.object.get('source_sha256')==self.catalog.get('source_sha256')
                and self.cartesian.scene.get('object_id')==self.object.get('id'))

    def generate(self):
        if not self.ready():return
        try:
            padding=float(self.cartesian.scene.get('link_padding_m',.01))
            mode=self.strategy.get()
            if mode not in ('Top','Side','Top + side'):raise ValueError('Select Top, Side or Top + side')
            self.candidates=(top_grasps(self.object,clearance=.004+2*padding) if mode!='Side' else [])
            if mode!='Top':self.candidates.extend(side_grasps(self.object,clearance=.004+2*padding))
            labels=[c['id']+' · gap %.1f mm'%(c['opening_m']*1000) for c in self.candidates]
            self.selector.configure(values=labels)
            self.choice.set(labels[0] if labels else '')
            self.reference_object=json.loads(json.dumps(self.object))
            self.feedback.set(str(len(labels))+' geometry candidates; choose one and Plan an approach.' if labels else
                              'Object does not fit the source Panda 80 mm opening with current planning padding.')
        except (ValueError,TypeError,KeyError) as exc:
            self.feedback.set(str(exc))

    def strategy_changed(self,*_):
        self.candidates=[];self.choice.set('');self.selector.configure(values=[])
        self.cartesian.invalidate()
        self.feedback.set('Strategy changed; generate fresh candidates before planning.')

    def set_gap(self):
        if not self.ready() or not self.candidates:return
        index=self.selector.current()
        if not 0<=index<len(self.candidates):return
        try:
            self.cartesian.arm.app.hand_tab.gap_var.set(1000*self.candidates[index]['opening_m'])
            self.feedback.set('Opening field set in Hand; explicitly Set Opening there before grasp planning.')
        except AttributeError:self.feedback.set('Open the Hand panel to set the candidate gap.')

    def plan(self,stage):
        if stage not in ('approach','grasp','lift','retreat') or not self.ready() or not self.candidates:return
        index=self.selector.current()
        if not 0<=index<len(self.candidates):return
        if stage=='lift' and not (self.cartesian.scene.get('object_attached') and self.cartesian.scene.get('grasp_contacts_ready')):
            self.feedback.set('Confirm finger contacts and attach the measured planning object before lift.');return
        if stage!='lift' and (self.cartesian.scene.get('object_attached') or self.cartesian.scene.get('grasp_contacts_ready')):
            self.feedback.set('Release and detach the object before a new approach, grasp or retreat.');return
        if stage in ('grasp','retreat'):
            opening=sum(getattr(self.cartesian,'joints',{}).get(name,0.) for name in ('finger_joint1','finger_joint2'))
            if opening<self.candidates[index]['opening_m']-.002:
                self.feedback.set('Open Hand to the candidate gap first; setting the field alone does not open it.');return
        current=self.object['orientation_wxyz'];previous=self.reference_object['orientation_wxyz']
        orientation_dot=abs(sum(a*b for a,b in zip(current,previous)))/math.sqrt(sum(v*v for v in current)*sum(v*v for v in previous))
        if (self.object.get('id')!=self.reference_object.get('id') or
                math.dist(self.object['position'],self.reference_object['position'])>.005
                or orientation_dot<.999
                or self.object['size']!=self.reference_object['size']):
            self.feedback.set('Object moved; generate fresh candidates before planning.')
            self.candidates=[];return
        from geometry_msgs.msg import Pose
        selected=self.candidates[index]['poses'][stage];pose=Pose()
        for axis,value in zip('xyz',selected['position']):setattr(pose.position,axis,value)
        for axis,value in zip('wxyz',selected['orientation_wxyz']):setattr(pose.orientation,axis,value)
        self.cartesian.plan_pose(pose,'Object '+self.candidates[index]['object_id']+' · '+stage)

    def plan_placement(self,stage):
        if stage not in ('above','lower') or not self.ready():return
        if not (self.cartesian.scene.get('object_attached') and self.cartesian.scene.get('grasp_contacts_ready')):
            self.feedback.set('Placement needs a measured two-finger grasp and acknowledged attachment.');return
        if time.monotonic()-self.catalog_received>.8:return
        try:
            gap=float(self.release_gap.get())
            if not math.isfinite(gap) or not .003<=gap<=.15:
                raise ValueError('Use a finite release gap from 3 to 150 mm.')
            support=self.catalog.get('supports',{}).get(self.support_choice.get())
            result=support_placement(self.object,self.cartesian.scene.get('object_in_tool'),support,
                clearance=gap,edge_margin=.01+float(self.cartesian.scene.get('link_padding_m',.01)))
            from geometry_msgs.msg import Pose
            target=result['poses'][stage];pose=Pose()
            for axis,value in zip('xyz',target['position']):setattr(pose.position,axis,value)
            for axis,value in zip('wxyz',target['orientation_wxyz']):setattr(pose.orientation,axis,value)
            self.cartesian.plan_pose(pose,'Place '+result['object_id']+' · '+result['support_id']+' · '+stage)
            self.feedback.set('Placement plan requested; inspect and Execute Plan. Hand release remains explicit.')
        except (ValueError,TypeError,KeyError,tk.TclError) as exc:self.feedback.set(str(exc))

    def poll(self):
        ready=self.ready()
        selectable=(self.cartesian.owned() and self.cartesian.arm.ready() and self.selection_publisher is not None
                    and time.monotonic()-self.catalog_received<.8 and bool(self.catalog.get('objects'))
                    and not self.cartesian.scene.get('object_attached') and not self.cartesian.planning)
        self.select_button.state(['!disabled'] if selectable else ['disabled'])
        self.refresh_button.state(['!disabled'] if ready else ['disabled'])
        self.opening_button.state(['!disabled'] if ready and self.candidates else ['disabled'])
        held=bool(self.cartesian.scene.get('object_attached'))
        contacts=bool(self.cartesian.scene.get('grasp_contacts_ready'))
        for index,button in enumerate(self.buttons):
            allowed=held and contacts if index==2 else not held and not contacts
            button.state(['!disabled'] if ready and self.candidates and allowed else ['disabled'])
        place=(ready and time.monotonic()-self.catalog_received<.8
               and bool(self.catalog.get('supports')) and self.cartesian.scene.get('object_attached')
               and self.cartesian.scene.get('grasp_contacts_ready'))
        for button in self.place_buttons:button.state(['!disabled'] if place else ['disabled'])
        if not self.cartesian.owned():
            self.object=None;self.candidates=[]
