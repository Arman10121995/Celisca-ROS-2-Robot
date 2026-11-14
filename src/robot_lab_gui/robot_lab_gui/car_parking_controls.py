"""Explicit map target and parking controls for the owned car Nav2 session."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk


class CarParkingControls(ttk.LabelFrame):
    def __init__(self,parent,app):
        super().__init__(parent,text='Park at a map pose · experimental',padding=8)
        self.app=app;self.node=None;self.state={};self.received=-math.inf
        self.was_owned=False;self.closed=False;self.requested=False
        self.target=[tk.DoubleVar(value=0.) for _ in range(3)]
        self.timeout=tk.DoubleVar(value=180.)
        self.status=tk.StringVar(value='Select an Ackermann car in Navigation. Connect reads state only.')
        self.connect_button=ttk.Button(self,text='Connect parking state',command=self.connect)
        self.connect_button.grid(row=0,column=0,columnspan=2,sticky='ew')
        for row,(label,variable) in enumerate(zip(('X · map m','Y · map m','Heading · deg'),self.target),1):
            ttk.Label(self,text=label).grid(row=row,column=0,sticky='w')
            ttk.Entry(self,textvariable=variable,width=10).grid(row=row,column=1,sticky='ew')
        self.measured_button=ttk.Button(self,text='Use estimated pose',command=self.use_pose)
        self.measured_button.grid(row=4,column=0,columnspan=2,sticky='ew',pady=3)
        ttk.Label(self,text='Whole task budget · s').grid(row=5,column=0,sticky='w')
        ttk.Spinbox(self,textvariable=self.timeout,from_=5,to=600,increment=15,width=8).grid(row=5,column=1,sticky='ew')
        self.start_button=ttk.Button(self,text='Park at target',command=self.start)
        self.start_button.grid(row=6,column=0,sticky='ew',pady=5)
        self.cancel_button=ttk.Button(self,text='Cancel parking',command=self.cancel)
        self.cancel_button.grid(row=6,column=1,sticky='ew',pady=5)
        ttk.Label(self,textvariable=self.status,wraplength=300).grid(row=7,column=0,columnspan=2,sticky='w')
        ttk.Label(self,text='Uses Nav2, then forward/reverse heading correction and a straight trim. '
            'Editing targets sends no motion. Fresh localization, costmap and operator heartbeat '
            'are required. Errors shown here use localization estimates.',wraplength=300).grid(
                row=8,column=0,columnspan=2,sticky='w',pady=5)
        self.columnconfigure(1,weight=1)
        self.job=self.after(200,self.poll)

    def selected(self):
        return (self.app.robot_var.get() in ('ackermann_car','rear_steer_car','anti_ackermann_car')
            and self.app.mode_var.get()=='nav' and self.app.launch_kind_var.get()=='simulation')

    def owned(self):
        process=self.app.process
        return bool(self.selected() and self.app._launch_running and process and process.poll() is None
            and 'robot_model:='+self.app.robot_var.get() in process.args
            and 'simulator:='+self.app.simulator_var.get() in process.args and 'mode:=nav' in process.args)

    def ready(self):
        return bool(self.owned() and time.monotonic()-self.received<.8 and self.state.get('instance_id')
            and self.state.get('robot_model')==self.app.robot_var.get())

    def receive(self,message):
        try:
            if len(message.data)>32768:return
            data=json.loads(message.data)
            if not isinstance(data,dict) or not isinstance(data.get('instance_id'),str):return
            if data.get('robot_model')!=self.app.robot_var.get():return
            if self.state.get('instance_id')!=data['instance_id']:self.requested=False
            self.state=data;self.received=time.monotonic()
            if data.get('phase') in ('parked','failed','canceled') or data.get('rejected'):self.requested=False
        except (ValueError,TypeError):pass

    def connect(self):
        if not self.owned():return False
        if self.node is not None:return True
        try:
            import rclpy
            from std_msgs.msg import String,Empty
            if not rclpy.ok():rclpy.init(args=None)
            self.node=rclpy.create_node('robot_lab_gui_parking')
            self.publisher=self.node.create_publisher(String,'/car_parking/command',10)
            self.heartbeat=self.node.create_publisher(Empty,'/car_parking/heartbeat',10)
            self.node.create_subscription(String,'/car_parking/state',self.receive,10)
            self.status.set('Waiting for this launch parking controller…')
            return True
        except Exception as exc:
            if self.node is not None:self.node.destroy_node();self.node=None
            self.status.set('Parking connection: '+str(exc));return False

    def use_pose(self):
        if not self.ready():return
        try:
            x,y,yaw=map(float,self.state['pose'])
            if not all(map(math.isfinite,(x,y,yaw))):raise ValueError()
            for variable,value in zip(self.target,(x,y,math.degrees(yaw))):variable.set(round(value,4))
            self.status.set('Copied the current localization estimate; edit the target, then Park.')
        except (ValueError,TypeError,KeyError):self.status.set('No fresh localization pose.')

    def start(self):
        if not self.ready() or self.active() or not self.state.get('navigation_ready'):return
        try:
            x,y,degrees=[float(variable.get()) for variable in self.target];timeout=float(self.timeout.get())
            if not all(map(math.isfinite,(x,y,degrees,timeout))) or not 5<=timeout<=600:
                raise ValueError('Finite map target and 5–600 second budget required')
            self.app._stop_drive()
            from std_msgs.msg import String,Empty
            self.heartbeat.publish(Empty())
            self.publisher.publish(String(data=json.dumps(dict(op='park',instance_id=self.state['instance_id'],
                x=x,y=y,yaw=math.radians(degrees),timeout=timeout))))
            self.requested=True;self.status.set('Parking requested; waiting for controller state.')
        except (ValueError,tk.TclError) as exc:self.status.set(str(exc))

    def active(self):
        return bool(self.requested or self.state.get('phase') in ('navigating','aligning','trimming'))

    def cancel(self):
        if self.node is not None and self.state.get('instance_id') and self.active():
            from std_msgs.msg import String
            self.publisher.publish(String(data=json.dumps(dict(op='cancel',instance_id=self.state['instance_id']))))
        self.requested=False

    def poll(self):
        if self.closed:return
        self.grid() if self.selected() else self.grid_remove()
        owned=self.owned()
        if self.was_owned and not owned:self.cancel()
        self.was_owned=owned
        if self.node is not None:
            import rclpy
            try:
                for _ in range(3):rclpy.spin_once(self.node,timeout_sec=0.)
                if self.ready():
                    from std_msgs.msg import Empty
                    self.heartbeat.publish(Empty())
                    phase=self.state.get('phase','waiting')
                    text=phase if self.state.get('navigation_ready') else phase+' · waiting for active Nav2'
                    if 'position_error_m' in self.state:
                        text+=' · estimate %.3f m / %.2f°'%(self.state['position_error_m'],self.state['heading_error_deg'])
                    if self.state.get('reason') or self.state.get('rejected'):
                        text+=' · '+str(self.state.get('reason') or self.state.get('rejected'))
                    self.status.set(text)
                elif owned:self.status.set('Parking feedback missing/stale; reconnect if the launch changed.')
            except Exception as exc:self.status.set('Parking feedback: '+str(exc))
        self.connect_button.state(['!disabled'] if owned else ['disabled'])
        self.start_button.state(['!disabled'] if self.ready() and self.state.get('navigation_ready')
            and not self.active() else ['disabled'])
        self.measured_button.state(['!disabled'] if self.ready() else ['disabled'])
        self.cancel_button.state(['!disabled'] if owned and self.active() else ['disabled'])
        self.job=self.after(200,self.poll)

    def close(self):
        self.cancel();self.closed=True;self.after_cancel(self.job)
        if self.node is not None:self.node.destroy_node();self.node=None
