"""Explicit flipper targets; actual native joints remain the feedback source."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
except ImportError:rclpy=None


class TrackedControls(ttk.Frame):
    def __init__(self,notebook,app):
        super().__init__(notebook,padding=8);notebook.add(self,text='Tracks');self.app=app
        self.node=self.executor=self.runtime=None;self.runtime_at=-math.inf;self.closed=False
        self.status=tk.StringVar(value='Select the articulated-flipper variant in the tracked family.')
        self.feedback=tk.StringVar(value='Waiting for actual native joint positions')
        self.columnconfigure(0,weight=1)
        ttk.Label(self,text='Tracked vehicle · four flippers',font=('',11,'bold')).grid(row=0,column=0,columnspan=2,sticky='w')
        ttk.Label(self,textvariable=self.status,wraplength=310).grid(row=1,column=0,columnspan=2,sticky='ew',pady=5)
        ttk.Button(self,text='Connect · no target change',command=self.connect).grid(row=2,column=0,columnspan=2,sticky='ew')
        self.values=[tk.DoubleVar(value=0.) for _ in range(4)]
        for i,(label,value) in enumerate(zip(('Front left','Rear left','Front right','Rear right'),self.values)):
            ttk.Label(self,text=label+' °').grid(row=3+i,column=0,sticky='w')
            ttk.Spinbox(self,textvariable=value,from_=-45,to=45,increment=5,width=9).grid(row=3+i,column=1,sticky='ew',pady=3)
        for row,label,callback in [(7,'Use measured angles',self.use_measured),(8,'Apply bounded targets',self.apply),
                (9,'Raise ends',self.raise_ends),(10,'Hold flippers / stop Drive',self.stop),
                (11,'Open latched Drive controls',lambda:app.control_notebook.select(app.drive_page))]:
            ttk.Button(self,text=label,command=callback).grid(row=row,column=0,columnspan=2,sticky='ew',pady=2)
        ttk.Label(self,textvariable=self.feedback,wraplength=310).grid(row=12,column=0,columnspan=2,sticky='ew',pady=5)
        ttk.Label(self,text='Targets slew at 0.2 rad/s; native force PID is capped at 4 Nm. '
            'Hold preserves the current motor target. The body stays free in physics. '
            'The original source variant keeps its fixed flippers.',wraplength=310).grid(row=13,column=0,columnspan=2,sticky='ew')
        self.job=self.after(50,self.tick)

    def connect(self):
        if self.node:return True
        if rclpy is None:self.status.set('Source the ROS workspace first');return False
        if not rclpy.ok():rclpy.init(args=None)
        self.node=rclpy.create_node('tracked_flipper_gui_'+str(time.time_ns()))
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.publisher=self.node.create_publisher(String,'/tracked/flipper_command',10)
        self.node.create_subscription(String,'/tracked/state',self.observe,10)
        self.status.set('Connected; no angle or Drive command sent');return True

    def observe(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('robot_id')!=self.app.robot_var.get():return
            self.runtime=data;self.runtime_at=time.monotonic();flippers=data['flippers']
            if flippers['positions'] is not None:
                angles=[math.degrees(v) for v in flippers['positions']]
                self.feedback.set('Measured ° · FL {:.1f}, RL {:.1f}, FR {:.1f}, RR {:.1f}'.format(*angles))
            self.status.set('Articulated flippers connected' if flippers['enabled'] else 'Original source has fixed flippers; choose the articulated variant')
        except (ValueError,KeyError,TypeError):return

    def send(self,operation,positions=None):
        state=self.runtime
        if (not self.node or not state or time.monotonic()-self.runtime_at>.6
                or (operation!='hold' and state.get('robot_id')!=self.app.robot_var.get()) or not state['flippers']['enabled']
                or not state['flippers']['feedback_fresh']):
            self.status.set('Connect to fresh feedback from the selected articulated variant');return False
        message=dict(session_id=state['session_id'],robot_id=state['robot_id'],op=operation)
        if positions is not None:message['positions']=positions
        self.publisher.publish(String(data=json.dumps(message,allow_nan=False)));return True

    def use_measured(self):
        if self.runtime and self.runtime['flippers']['positions'] is not None:
            for variable,value in zip(self.values,self.runtime['flippers']['positions']):variable.set(math.degrees(value))

    def apply(self):
        try:
            angles=[math.radians(variable.get()) for variable in self.values]
            if any(not math.isfinite(value) or abs(value)>.8 for value in angles):raise ValueError('Flipper angles must be within ±45 degrees')
            self.send('target',angles)
        except (ValueError,tk.TclError) as exc:self.status.set(str(exc))

    def raise_ends(self):
        for variable,value in zip(self.values,(-25,25,-25,25)):variable.set(value)
        self.apply()

    def stop(self):
        if self.node and self.runtime:self.send('hold')
        self.app._stop_drive(keep_input_enabled=True)

    def tick(self):
        if self.closed:return
        if self.executor and rclpy.ok():self.executor.spin_once(timeout_sec=0.)
        self.job=self.after(50,self.tick)

    def close(self):
        if self.closed:return
        self.stop();self.closed=True;self.after_cancel(self.job)
        if self.executor:self.executor.shutdown(timeout_sec=.1)
        if self.node:self.node.destroy_node()
        self.node=self.executor=None
