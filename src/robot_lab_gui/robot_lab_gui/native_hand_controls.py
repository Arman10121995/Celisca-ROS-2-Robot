"""Coordinated native hand targets from owned, fresh measured state."""
import json
import math
import tkinter as tk
from tkinter import ttk


class NativeHandControls(ttk.Frame):
    def __init__(self, parent, articulation):
        super().__init__(parent)
        self.owner = articulation
        self.pose = tk.StringVar(value='Curl')
        self.finger = tk.StringVar(value='All')
        self.closure = tk.DoubleVar(value=.6)
        self.spread = tk.DoubleVar(value=0.)
        self.effort = tk.DoubleVar(value=.6)
        self.contact = tk.DoubleVar(value=1.)
        self.guarded = tk.BooleanVar(value=True)
        self.feedback = tk.StringVar(value='Enable native joint controls and Run to receive the hand contract.')
        self.note = tk.StringVar(value='Experimental native hand controls')
        self.columnconfigure((0, 1), weight=1)
        ttk.Label(self, text='Coordinated hand poses', style='Heading.TLabel').grid(
            row=0, column=0, columnspan=2, sticky='w')
        ttk.Label(self, textvariable=self.note, wraplength=305).grid(row=1, column=0, columnspan=2, sticky='w')
        self.pose_selector = ttk.Combobox(self, textvariable=self.pose, state='readonly', width=22)
        self.pose_selector.grid(row=2, column=0, columnspan=2, sticky='ew', pady=4)
        ttk.Label(self, text='Finger group').grid(row=3, column=0, sticky='w')
        self.finger_selector = ttk.Combobox(self, textvariable=self.finger, state='readonly', width=16)
        self.finger_selector.grid(row=3, column=1, sticky='ew')
        for row, label, var, low, high, increment in (
                (4, 'Closure fraction', self.closure, 0., 1., .05),
                (5, 'Spread fraction', self.spread, -1., 1., .1),
                (7, 'Effort stop fraction', self.effort, .05, 1., .05),
                (8, 'Contact stop (N / group)', self.contact, .2, 5., .2)):
            ttk.Label(self, text=label).grid(row=row, column=0, sticky='w', pady=3)
            ttk.Spinbox(self, textvariable=var, from_=low, to=high, increment=increment, width=8).grid(
                row=row, column=1, sticky='w')
        ttk.Checkbutton(self, text='Guard closure with measured effort/contact', variable=self.guarded).grid(
            row=6, column=0, columnspan=2, sticky='w', pady=4)
        self.apply_button = ttk.Button(self, text='Apply hand pose', command=self.apply)
        self.open_button = ttk.Button(self, text='Open selected fingers', command=lambda: self.apply('Open'))
        self.apply_button.grid(row=9, column=0, sticky='ew', padx=2, pady=5)
        self.grasp_object_button = ttk.Button(self, text='Grasp object', command=self.grasp_object)
        self.release_object_button = ttk.Button(self, text='Release object', command=self.release_object)
        self.grasp_object_button.grid(row=9, column=2, sticky='ew', padx=2, pady=5)
        self.release_object_button.grid(row=10, column=2, sticky='ew', padx=2, pady=5)
        self.stop_button = ttk.Button(self, text='Stop hand / hold', command=self.owner.stop)
        self.open_button.grid(row=9, column=1, sticky='ew', padx=2, pady=5)
        self.stop_button.grid(row=10, column=0, columnspan=2, sticky='ew', pady=3)
        ttk.Label(self, textvariable=self.feedback, wraplength=305).grid(row=11, column=0, columnspan=2, sticky='w', pady=5)
        ttk.Label(self, text='Editing a pose or limit causes no motion. Apply sends source-bounded '
            'targets. Each guarded finger group stops advancing at a measured threshold; '
            'this does not impose a hard contact-force limit or confirm a grasp. '
            'UMI has no source tendon force clamp: its effort threshold uses 2 N × fraction. '
            'Use Open or Release object to open; Stop and heartbeat loss hold measured coordinates.',
            wraplength=305).grid(row=12, column=0, columnspan=2, sticky='w', pady=5)
        self.enable_button = ttk.Button(self, text='Enable hand controls for next Run', command=self.enable)
        self.enable_button.grid(row=13, column=0, columnspan=2, sticky='ew', pady=5)

    def enable(self):
        if self.selected() and not self.owner.app._launch_running:
            self.owner.app.native_articulation_var.set(True)
            self.owner.app._update_from_selection()

    def selected(self):
        return self.owner.selected() and bool(self.owner.app._robot_config().get('native_hand'))

    def poll(self):
        if not self.selected():
            self.grid_remove()
            return
        self.grid()
        self.enable_button.state(['disabled'] if self.owner.app._launch_running else ['!disabled'])
        hand = (self.owner.state or {}).get('hand', {})
        ready = bool(self.owner.ready() and hand.get('contract_sha256') and hand.get('groups'))
        for button in (self.apply_button, self.open_button, self.stop_button):
            button.state(['!disabled'] if ready else ['disabled'])
        if not ready:
            self.feedback.set('Waiting for fresh source hand feedback from the selected Run.')
            return
        poses = hand['poses']
        fingers = ['All']+[g['name'] for g in hand['groups']]
        self.pose_selector.configure(values=poses)
        self.finger_selector.configure(values=fingers)
        if self.pose.get() not in poses:
            self.pose.set(poses[0])
        if self.finger.get() not in fingers:
            self.finger.set('All')
        self.note.set(hand['adapter']+' · '+hand['actuation']+' · experimental')
        details = ['%s: %.3f N (%d contacts)' % (name, entry['normal_force_n'], entry['contact_count'])
            for name, entry in hand.get('contacts', {}).items()]
        blocked = ', '.join(hand.get('blocked_groups', {}))
        self.feedback.set(hand['motion']['state']+'\n'
            + '\n'.join(details)
            + ('\nStopped groups: '+blocked if blocked else '')
            + ('\nDocumented object present: grasp/release enabled' if self.owner.object_body >= 0 else ''))

    def apply(self, pose=None):
        if not self.owner.ready():
            return
        state = self.owner.state
        hand = state.get('hand')
        if not hand:
            return
        try:
            closure, spread = float(self.closure.get()), float(self.spread.get())
            effort, contact = float(self.effort.get()), float(self.contact.get())
            if (not all(math.isfinite(v) for v in (closure, spread, effort, contact))
                    or not 0 <= closure <= 1 or not -1 <= spread <= 1
                    or not .05 <= effort <= 1 or not .2 <= contact <= 5):
                raise ValueError('Choose finite closure/spread and effort/contact thresholds inside the displayed bounds.')
            chosen = pose or self.pose.get()
            if chosen not in [*hand['poses'], 'Open']:
                raise ValueError('Choose an available hand pose')
            from std_msgs.msg import Empty, String
            self.owner.heartbeat.publish(Empty())
            self.owner.pub.publish(String(data=json.dumps(dict(operation='hand_pose',
                plant_id=state['plant_id'], source_sha256=state['source_sha256'],
                contract_sha256=hand['contract_sha256'], pose=chosen,
                finger=self.finger.get(), closure=closure, spread=spread,
                guarded=bool(self.guarded.get()), effort_fraction=effort, contact_limit_n=contact))))
            self.feedback.set('Requested '+chosen+'; waiting for measured feedback')
        except (ValueError, TypeError, tk.TclError) as exc:
            self.feedback.set(str(exc))

    def grasp_object(self):
        """Close on the documented free object; measured two-sided hold only."""
        if not self.owner.ready():
            self.feedback.set('Run first, then Grasp object.')
            return
        state = self.owner.state
        hand = state.get('hand')
        if self.owner.object_body < 0:
            self.feedback.set('Enable the object fixture before Grasp object.')
            return
        if not hand:
            self.feedback.set('No hand contract; enable native hand controls and Run.')
            return
        try:
            from std_msgs.msg import String
            self.owner.pub.publish(String(data=json.dumps(dict(operation='hand_grasp',
                plant_id=state['plant_id'], source_sha256=state['source_sha256'],
                contract_sha256=hand['contract_sha256'], closure=1., spread=0.,
                guarded=bool(self.guarded.get()), effort_fraction=float(self.effort.get()),
                contact_limit_n=float(self.contact.get())))))
            self.feedback.set('Closing on the documented object; measured contact hold pending.')
        except (ValueError, TypeError, KeyError, IndexError, AttributeError) as exc:
            self.feedback.set('Grasp rejected: '+str(exc))

    def release_object(self):
        """Open the fingers and let the physics engine own the object again."""
        if not self.owner.ready():
            self.feedback.set('Run first, then Release object.')
            return
        state = self.owner.state
        hand = state.get('hand')
        if self.owner.object_body < 0:
            self.feedback.set('Enable the object fixture before Release object.')
            return
        if not hand:
            self.feedback.set('No hand contract; enable native hand controls and Run.')
            return
        try:
            from std_msgs.msg import String
            self.owner.pub.publish(String(data=json.dumps(dict(operation='hand_release',
                plant_id=state['plant_id'], source_sha256=state['source_sha256'],
                contract_sha256=hand['contract_sha256']))))
            self.feedback.set('Released on the documented object; physics now owns its motion.')
        except (ValueError, TypeError, KeyError, IndexError, AttributeError) as exc:
            self.feedback.set('Release rejected: '+str(exc))
