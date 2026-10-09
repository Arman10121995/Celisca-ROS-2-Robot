"""Explicit press-and-hold Cartesian jogging for native actuator controllers."""
import json
import tkinter as tk
from tkinter import ttk


class ServoControls(ttk.LabelFrame):
    def __init__(self, parent, controller, topic):
        super().__init__(parent, text='Cartesian jog (experimental)', padding=6)
        self.controller, self.topic = controller, topic
        self.node = None
        self.enabled = tk.BooleanVar(value=False)
        self.site = tk.StringVar()
        self.status = tk.StringVar(value='Select an authored tool frame; hold a direction to move.')
        self.linear = tk.DoubleVar(value=.02)
        self.angular = tk.DoubleVar(value=.1)
        self.velocity = [0.]*6
        self.job = None
        ttk.Checkbutton(self, text='Enable Cartesian jogging', variable=self.enabled,
                        command=self.stop).grid(row=0, column=0, columnspan=4, sticky='w')
        self.combo = ttk.Combobox(self, textvariable=self.site, state='readonly', width=28)
        self.combo.grid(row=1, column=0, columnspan=4, sticky='ew', pady=4)
        self.combo.bind('<<ComboboxSelected>>', lambda _: self.stop())
        ttk.Label(self, text='Translation m/s').grid(row=2, column=0, sticky='w')
        ttk.Spinbox(self, textvariable=self.linear, from_=.005, to=.04, increment=.005, width=7).grid(row=2, column=1)
        ttk.Label(self, text='Rotation rad/s').grid(row=2, column=2, sticky='w')
        ttk.Spinbox(self, textvariable=self.angular, from_=.02, to=.2, increment=.02, width=7).grid(row=2, column=3)
        self.buttons = []
        for axis, label in enumerate(('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')):
            row, column = 3+axis//2, (axis % 2)*2
            for offset, sign, suffix in ((0, -1, '−'), (1, 1, '+')):
                button = ttk.Button(self, text=label+suffix, width=7)
                button.grid(row=row, column=column+offset, sticky='ew', padx=2, pady=2)
                button.bind('<ButtonPress-1>', lambda _, a=axis, s=sign: self.start(a, s))
                button.bind('<ButtonRelease-1>', lambda _: self.stop())
                button.bind('<Leave>', lambda _: self.stop())
                self.buttons.append(button)
        ttk.Button(self, text='Stop Cartesian jog', command=self.stop).grid(row=6, column=0, columnspan=4, sticky='ew', pady=4)
        ttk.Label(self, textvariable=self.status, wraplength=305).grid(row=7, column=0, columnspan=4, sticky='w')
        ttk.Label(self, text='Axes use native_world. Native Jacobian and predicted contacts bound '
            'each target. Release stops the jog; global obstacle planning remains Plan/Execute. '
            'Validation of this new servo is pending.', wraplength=305).grid(row=8, column=0, columnspan=4, sticky='w', pady=4)

    def poll(self):
        servo = (self.controller.state or {}).get('servo') or {}
        sites = servo.get('sites', [])
        self.combo.configure(values=sites)
        if self.site.get() not in sites:
            self.site.set(sites[0] if sites else '')
        if servo.get('error'):
            self.status.set(servo['error'])
        ready = self.controller.ready() and self.enabled.get() and bool(self.site.get())
        for button in self.buttons:
            button.state(['!disabled'] if ready else ['disabled'])
        if not ready:
            self.stop()

    def start(self, axis, sign):
        if not self.controller.ready() or not self.enabled.get() or not self.site.get():
            return 'break'
        try:
            speed = float(self.linear.get() if axis < 3 else self.angular.get())
            if not 0 < speed <= (.04 if axis < 3 else .2):
                raise ValueError('Choose a speed inside the displayed limits.')
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return 'break'
        if hasattr(self.controller, 'cartesian'):
            self.controller.cartesian.invalidate()
        self.stop()
        self.velocity[axis] = sign*speed
        self.repeat()
        return 'break'

    def send(self):
        node = self.controller.node
        if not node or not self.controller.owned():
            return
        from std_msgs.msg import String
        if self.node is not node:
            self.node = node
            self.publisher = node.create_publisher(String, self.topic, 1)
        message = dict(site=self.site.get(), frame='native_world',
            linear=self.velocity[:3], angular=self.velocity[3:])
        plant_id = (self.controller.state or {}).get('plant_id')
        if plant_id:
            message['plant_id'] = plant_id
        self.publisher.publish(String(data=json.dumps(message)))

    def repeat(self):
        self.job = None
        if not self.controller.ready() or not self.enabled.get():
            self.stop()
            return
        self.send()
        self.job = self.after(100, self.repeat)

    def stop(self):
        if self.job is not None:
            self.after_cancel(self.job)
            self.job = None
        moving = any(self.velocity)
        self.velocity = [0.]*6
        if moving:
            self.send()
