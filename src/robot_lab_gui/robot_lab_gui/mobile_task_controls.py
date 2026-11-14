"""Build source-bound navigate/articulate tasks in the native control column."""
import copy
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, ttk
import uuid

from robot_lab_utils.mobile_task import SCHEMA, validate_task


class MobileTaskControls(ttk.LabelFrame):
    def __init__(self, parent, control):
        super().__init__(parent, text='Mobile task: navigate + arm / hand (experimental)', padding=6)
        self.control, self.app = control, control.app
        self.node, self.state, self.received = None, {}, -math.inf
        self.task_id, self.key, self.steps, self.buffers = '', None, [], {}
        self.status = tk.StringVar(value='Add a base destination and a taught arm/hand sequence.')
        self.pose = [tk.DoubleVar(value=0.) for _ in range(3)]
        for row, (label, variable) in enumerate(zip(('Map X (m)', 'Map Y (m)', 'Heading (deg)'), self.pose)):
            ttk.Label(self, text=label).grid(row=row, column=0, sticky='w')
            ttk.Entry(self, textvariable=variable, width=9).grid(row=row, column=1, sticky='ew')
        # Two columns throughout keep the frame inside the control column.
        self.buttons = {}
        for row, column, name, title, callback in (
                (3, 0, 'current', 'Use measured base', self.current),
                (3, 1, 'navigate', 'Add navigation step', self.add_navigation),
                (4, 0, 'articulate', 'Add taught sequence', self.add_articulation)):
            button = ttk.Button(self, text=title, command=callback)
            button.grid(row=row, column=column, sticky='ew', padx=2, pady=2)
            self.buttons[name] = button
        self.listing = tk.Listbox(self, height=5, width=30, exportselection=False)
        self.listing.grid(row=5, column=0, columnspan=2, sticky='ew', pady=4)
        for row, column, name, title, callback in (
                (6, 0, 'remove', 'Remove step', self.remove),
                (6, 1, 'up', 'Move step up', self.move_up),
                (7, 0, 'save', 'Save task', self.save),
                (7, 1, 'load', 'Load task', self.load),
                (8, 0, 'run', 'Run task', self.run),
                (8, 1, 'stop', 'Stop task', self.stop)):
            button = ttk.Button(self, text=title, command=callback)
            button.grid(row=row, column=column, sticky='ew', padx=2, pady=3)
            self.buttons[name] = button
        self.columnconfigure((0, 1), weight=1)
        ttk.Label(self, textvariable=self.status, wraplength=305).grid(row=9, column=0, columnspan=2, sticky='w', pady=4)
        ttk.Label(self, text='Teach configurations above, then add their sequence here. '
            'Base navigation needs Native workflow nav and a retracted arm. '
            'Tasks alternate Nav2 and stopped-base articulation. Editing/loading does not move the robot. '
            'Drive/Space, manual arm input, Stop or GUI loss interrupts the task. '
            'Grasp/object outcomes require separate inspection.',
            wraplength=305).grid(row=10, column=0, columnspan=2, sticky='w', pady=4)

        self.object_fields, self.object_choices = {}, {}
        for row, (key, label) in enumerate((('object', 'Free object body'), ('left_body', 'First finger body'),
                                            ('right_body', 'Opposing finger body')), 11):
            variable = tk.StringVar()
            self.object_fields[key] = variable
            ttk.Label(self, text=label).grid(row=row, column=0, sticky='w')
            choice = ttk.Combobox(self, textvariable=variable, width=18, state='readonly')
            choice.grid(row=row, column=1, sticky='ew')
            self.object_choices[key] = choice
        for column, condition in enumerate(('grasped', 'released')):
            button = ttk.Button(self, text='Check '+condition,
                                command=lambda value=condition: self.add_object_check(value))
            button.grid(row=14, column=column, sticky='ew', padx=2, pady=2)
            self.buttons['check_'+condition] = button
        ttk.Label(self, text='Choose the free object and two opposing finger bodies from the running model. Grasp needs both fingers and no external support; '
                  'release needs external support and low speed for 1 simulated second. '
                  'A grasp checkpoint enables contact/slip monitoring and adds the object collision bounds to the navigation footprint.',
                  wraplength=305).grid(row=15, column=0, columnspan=2, sticky='w')

        self.reach_offset = [tk.DoubleVar(value=0.) for _ in range(3)]
        self.place_target = [tk.DoubleVar(value=v) for v in (0.,0.,.5)]
        for row, (label, values) in enumerate((('Object offset XYZ (m)',self.reach_offset),
                                               ('Place XYZ · native world',self.place_target)),16):
            line = ttk.Frame(self)
            line.grid(row=row,column=0,columnspan=2,sticky='ew')
            ttk.Label(line,text=label).pack(anchor='w')
            fields = ttk.Frame(line);fields.pack(fill='x')
            for variable in values:
                ttk.Entry(fields,textvariable=variable,width=8).pack(side='left',expand=True,fill='x')
        self.orientation_mode=tk.StringVar(value='Position only')
        ttk.Combobox(self,textvariable=self.orientation_mode,state='readonly',
            values=('Position only','Keep measured pad','World pad RPY')).grid(row=18,column=0,columnspan=2,sticky='ew')
        self.orientation_rpy=[tk.DoubleVar(value=0.) for _ in range(3)]
        angles=ttk.Frame(self);angles.grid(row=19,column=0,columnspan=2,sticky='ew')
        ttk.Label(angles,text='World pad roll / pitch / yaw (deg)').pack(anchor='w')
        entries=ttk.Frame(angles);entries.pack(fill='x')
        for variable in self.orientation_rpy:
            ttk.Entry(entries,textvariable=variable,width=8).pack(side='left',expand=True,fill='x')
        for row,column,name,title,callback in (
                (20,0,'reach','Add object reach',self.add_reach),
                (20,1,'stow','Add arm retraction',lambda:self.add_native_step('stow_arm')),
                (21,0,'open','Add gripper open',lambda:self.add_native_step('grip',operation='open')),
                (21,1,'close','Add contact close',lambda:self.add_native_step('grip',operation='close')),
                (22,0,'pick','Add local pick recipe',self.add_pick),
                (22,1,'place','Add local place recipe',self.add_place)):
            button=ttk.Button(self,text=title,command=callback)
            button.grid(row=row,column=column,sticky='ew',padx=2,pady=2)
            self.buttons['object_'+name]=button
        ttk.Label(self,text='Recipes add steps only; Run task starts motion. Source IK tries up to six starts; bounded configuration '
                  'detours inspect sampled native motor motion. Place XYZ and pad RPY use native_world axes. '
                  'Keep measured pad holds wrist controls during each reach; explicit RPY constrains its endpoint. '
                  'Grasp outcomes require measured checkpoints.',
                  wraplength=305).grid(row=23,column=0,columnspan=2,sticky='w')

    def add_object_check(self, condition):
        if not self.editable() or len(self.steps) >= 32:
            return
        step = dict(kind='object_check', condition=condition, name='Check object '+condition,
                    **{key: variable.get().strip() for key, variable in self.object_fields.items()})
        try:
            document = self.document()
            document['steps'] = [step]
            validate_task(document, self.state['source_sha256'], self.state['scene_id'],
                          self.control.state['channels'], self.state.get('navigation_available', False))
            self.steps.append(step)
            self.refresh()
        except (ValueError, TypeError, KeyError) as exc:
            self.status.set(str(exc))

    def object_step(self, kind, **values):
        if kind=='reach_object':
            mode=self.orientation_mode.get()
            if mode=='Keep measured pad':values['keep_orientation']=True
            elif mode=='World pad RPY':
                from scipy.spatial.transform import Rotation
                rpy=[float(v.get()) for v in self.orientation_rpy]
                if not all(math.isfinite(v) and abs(v)<=360 for v in rpy):
                    raise ValueError('Use finite pad roll/pitch/yaw within ±360 degrees')
                values['orientation_wxyz']=Rotation.from_euler('xyz',rpy,degrees=True).as_quat()[[3,0,1,2]].tolist()
            elif mode!='Position only':raise ValueError('Select a reviewed reach orientation mode')
        return dict(kind=kind,name=kind.replace('_',' '),
                    **{key:var.get().strip() for key,var in self.object_fields.items()},**values)

    def add_native_steps(self, steps):
        if not self.editable():
            return
        try:
            document=self.document()
            document['steps']=[*document['steps'],*steps]
            validate_task(document,self.state['source_sha256'],self.state['scene_id'],
                          self.control.state['channels'],self.state.get('navigation_available',False))
            self.steps.extend(steps)
            self.refresh()
            self.status.set('Added local recipe steps; inspect destinations, then Run task.')
        except (ValueError,TypeError,KeyError,tk.TclError) as exc:
            self.status.set(str(exc))

    def add_native_step(self, kind, **values):
        self.add_native_steps([self.object_step(kind,**values)])

    def add_reach(self):
        try:
            offset=[float(v.get()) for v in self.reach_offset]
            self.add_native_step('reach_object',offset=offset)
        except (ValueError,tk.TclError) as exc:
            self.status.set(str(exc))

    def add_pick(self):
        try:
            self.add_native_steps([
                self.object_step('grip',operation='open'),
                self.object_step('reach_object',offset=[0.,0.,.08]),
                self.object_step('reach_object',offset=[0.,0.,0.]),
                self.object_step('grip',operation='close'),
                self.object_step('reach_object',offset=[0.,0.,.10]),
                self.object_step('object_check',condition='grasped'),
                self.object_step('stow_arm')])
        except (ValueError,tk.TclError) as exc:self.status.set(str(exc))

    def add_place(self):
        try:
            xyz=[float(v.get()) for v in self.place_target]
            self.add_native_steps([
                self.object_step('reach_object',target_xyz=[xyz[0],xyz[1],xyz[2]+.08]),
                self.object_step('reach_object',target_xyz=xyz),
                self.object_step('grip',operation='open'),
                self.object_step('object_check',condition='released'),
                self.object_step('reach_object',target_xyz=[xyz[0],xyz[1],xyz[2]+.10]),
                self.object_step('stow_arm')])
        except (ValueError,tk.TclError) as exc:
            self.status.set(str(exc))

    def selected(self):
        return bool(self.control.selected() and self.app._robot_config().get('mobile_control_config'))

    def connect(self):
        node = self.control.node
        if node and self.node is not node:
            from std_msgs.msg import String
            self.node = node
            self.command = node.create_publisher(String, '/mobile_task/command', 1)
            self.heartbeat = node.create_publisher(String, '/mobile_task/heartbeat', 1)
            node.create_subscription(String, '/mobile_task/state', self.receive, 10)

    def receive(self, message):
        try:
            state = json.loads(message.data)
            if (self.control.ready() and state.get('plant_id') == self.control.state['plant_id']
                    and state.get('source_sha256') == self.control.state['source_sha256']):
                self.state, self.received = state, time.monotonic()
        except (ValueError, TypeError, KeyError):
            pass

    def ready(self):
        return bool(self.selected() and self.control.ready() and time.monotonic()-self.received < .5
            and self.state.get('scene_id') and self.state.get('plant_id') == self.control.state['plant_id'])

    def editable(self):
        return bool(self.ready() and not self.state.get('active') and not self.state.get('nav_cancel_pending')
            and self.control.sequences.idle())

    def document(self):
        return dict(schema=SCHEMA, source_sha256=self.state['source_sha256'], scene_id=self.state['scene_id'],
            robot=self.app.robot_var.get(), world=self.app.map_var.get(), steps=copy.deepcopy(self.steps))

    def refresh(self):
        self.listing.delete(0, 'end')
        for index, step in enumerate(self.steps):
            self.listing.insert('end', str(index+1)+'. '+step['name'])

    def current(self):
        if self.ready():
            x, y, yaw = self.state['physical_pose']
            for variable, value in zip(self.pose, (x, y, math.degrees(yaw))):
                variable.set(round(value, 3))

    def add_navigation(self):
        if not self.editable() or not self.state.get('navigation_available'):
            return
        try:
            pose = [float(variable.get()) for variable in self.pose]
            pose[2] = math.radians(pose[2])
            if not all(map(math.isfinite, pose)) or len(self.steps) >= 32:
                raise ValueError('Use finite coordinates and at most 32 steps.')
            self.steps.append(dict(kind='navigate', name='Navigate %.2f, %.2f / %.1f°' %
                (pose[0], pose[1], math.degrees(pose[2])), pose=pose))
            self.refresh()
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def add_articulation(self):
        if self.editable() and self.control.sequences.points and len(self.steps) < 32:
            self.steps.append(dict(kind='articulate', name='Taught arm/hand sequence ('+
                str(len(self.control.sequences.points))+' points)',
                waypoints=copy.deepcopy(self.control.sequences.points)))
            self.refresh()

    def remove(self):
        if self.editable() and self.listing.curselection():
            del self.steps[self.listing.curselection()[0]]
            self.refresh()

    def move_up(self):
        if self.editable() and self.listing.curselection():
            index = self.listing.curselection()[0]
            if index:
                self.steps[index-1], self.steps[index] = self.steps[index], self.steps[index-1]
                self.refresh()
                self.listing.selection_set(index-1)

    def run(self):
        if not self.editable() or not self.steps:
            return
        try:
            from std_msgs.msg import String
            document = self.document()
            validate_task(document, self.state['source_sha256'], self.state['scene_id'],
                self.control.state['channels'], self.state.get('navigation_available', False))
            self.task_id = uuid.uuid4().hex
            self.command.publish(String(data=json.dumps(dict(operation='run',
                plant_id=self.state['plant_id'], task_id=self.task_id, task=document), allow_nan=False)))
            self.status.set('Task requested; waiting for measured controller feedback.')
        except (ValueError, TypeError, KeyError) as exc:
            self.status.set(str(exc))

    def stop(self):
        if self.ready():
            from std_msgs.msg import String
            self.command.publish(String(data=json.dumps(dict(operation='cancel', plant_id=self.state['plant_id']))))

    def save(self):
        if not self.editable() or not self.steps:
            return
        filename = filedialog.asksaveasfilename(parent=self,
            initialdir=self.control.sequences.initial_directory(), initialfile='mobile-task.json',
            defaultextension='.json', filetypes=[('Mobile task', '*.json')])
        if filename:
            try:
                path = Path(filename)
                if path.parent.stat().st_dev == Path('/').stat().st_dev:
                    raise ValueError('Save tasks on the mounted workspace SSD.')
                temporary = path.with_suffix(path.suffix+'.new')
                temporary.write_text(json.dumps(self.document(), indent=2, allow_nan=False)+'\n')
                temporary.replace(path)
                self.status.set('Saved task; no movement requested.')
            except (OSError, ValueError) as exc:
                self.status.set(str(exc))

    def load(self):
        if not self.editable():
            return
        filename = filedialog.askopenfilename(parent=self,
            initialdir=self.control.sequences.initial_directory(), filetypes=[('Mobile task', '*.json')])
        if filename:
            try:
                path = Path(filename)
                if path.stat().st_size > 2*1024*1024:
                    raise ValueError('Task exceeds the 2 MB input bound.')
                steps = validate_task(json.loads(path.read_text()), self.state['source_sha256'],
                    self.state['scene_id'], self.control.state['channels'], self.state.get('navigation_available', False))
                self.steps[:] = steps
                self.refresh()
                self.status.set('Loaded source-matched task; no movement until Run task.')
            except (ValueError, TypeError, KeyError, OSError) as exc:
                self.status.set(str(exc))

    def poll(self):
        if not self.selected():
            self.grid_remove()
            self.state = {}
            return
        self.grid()
        self.connect()
        if self.ready():
            for name, choice in self.object_choices.items():
                values = self.state.get('object_candidates' if name=='object' else 'finger_candidates', [])
                choice.configure(values=values)
                if self.object_fields[name].get() not in values:
                    default = {'left_body':'rubber_tip_left','right_body':'rubber_tip_right'}.get(name)
                    self.object_fields[name].set(default if default in values else
                        (values[0] if name=='object' and values else ''))
            key = (self.state['source_sha256'], self.state['scene_id'])
            if key != self.key:
                self.key = key
                self.steps = self.buffers.setdefault(key, [])
                self.refresh()
            if self.state.get('active') and self.state.get('task_id') == self.task_id:
                from std_msgs.msg import String
                self.heartbeat.publish(String(data=self.task_id))
            if self.state['state'] not in ('idle',):
                self.status.set(self.state['state']+' · '+str(self.state.get('step', 0))+'/'+
                    str(self.state.get('total', 0))+' · '+self.state.get('reason', '')+
                    (' · payload: '+self.state['payload_name'] if self.state.get('payload_name') else ''))
        for name, button in self.buttons.items():
            enabled = self.ready() if name in ('stop', 'current') else self.editable()
            if name.startswith(('check_','object_')):
                enabled = enabled and bool(self.state.get('object_candidates'))
            if name == 'navigate':
                enabled = enabled and self.state.get('navigation_available')
            if name == 'run':
                enabled = enabled and bool(self.steps)
            button.state(['!disabled'] if enabled else ['disabled'])
