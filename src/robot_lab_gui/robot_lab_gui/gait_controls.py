"""Native source-gait ownership and actual feedback; reuse the Drive pad."""
import json
import math
import time
import uuid
from pathlib import Path
import os
import sqlite3
import threading
import tkinter as tk
from tkinter import ttk
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
except ImportError:rclpy=None


class GaitControls(ttk.Frame):
    def __init__(self,notebook,app):
        super().__init__(notebook,padding=10);notebook.add(self,text='Gait');self.app=app
        self.node=self.executor=None;self.runtime=None;self.runtime_at=-math.inf;self.closed=False
        self.operator='gait-gui-'+uuid.uuid4().hex;self.last_hb=0.
        self.status=tk.StringVar(value='Select a six/eight/many-legged robot, MuJoCo and Gait mode.')
        self.feedback=tk.StringVar(value='Awaiting actual body, joints and foot forces')
        self.phase=tk.DoubleVar(value=0.0)
        self.schedule=tk.StringVar(value='wave');self.columnconfigure(0,weight=1)
        self.spine_phase=tk.DoubleVar(value=0.0);self.spine_freq=tk.DoubleVar(value=1.0)
        ttk.Label(self,text='Source gait · joint physics',font=('',11,'bold')).grid(row=0,column=0,sticky='w')
        ttk.Label(self,textvariable=self.status,wraplength=310).grid(row=1,column=0,sticky='ew',pady=7)
        for row,label,command in [(2,'Connect (neutral)',self.connect),(3,'Arm walking',lambda:self.operation('arm')),
                                  (4,'Stop / hold current targets',lambda:app._stop_drive(keep_input_enabled=True)),(5,'Return to source stance',lambda:self.operation('stand')),
                                  (6,'Open latched Drive / WASD controls',lambda:app.control_notebook.select(app.drive_page))]:
            ttk.Button(self,text=label,command=command).grid(row=row,column=0,sticky='ew',pady=2)
        combo=ttk.Frame(self);combo.grid(row=7,column=0,sticky='ew',pady=6)
        ttk.Label(combo,text='Schedule').pack(side='left')
        self.combo=ttk.Combobox(combo,textvariable=self.schedule,state='readonly',values=['wave','ripple','tripod'],width=12)
        self.combo.pack(side='left',padx=5);ttk.Button(combo,text='Apply',command=lambda:self.operation('gait')).pack(side='left');ttk.Label(combo,text='Phase').pack(side='left',padx=(10,0));ttk.Scale(combo,from_=0.0,to=1.0,variable=self.phase,orient='horizontal',length=50).pack(side='left',fill='x',expand=True,padx=2)
        ttk.Label(self,textvariable=self.feedback,wraplength=310,justify='left').grid(row=8,column=0,sticky='ew',pady=6)
        ttk.Label(self,text='Connect and Arm start without travel. Use Drive for latching, ramps, WASD/joystick and Space.\n'
                  'Stop/loss halts gait phases and holds bounded joint targets; bodies remain free in physics.\n'
                  'Measured contacts are separate from planned stance. Experimental contact odometry and mapping/navigation can be selected below; terrain and backend qualification remain pending.',
                  wraplength=310).grid(row=9,column=0,sticky='ew')
        tasks=ttk.LabelFrame(self,text='Experimental sensor task · applied on next Run',padding=5)
        tasks.grid(row=10,column=0,sticky='ew',pady=5)
        task_combo=ttk.Combobox(tasks,textvariable=app.gait_task_var,values=('gait','loc','slam','3d_slam','nav'),state='readonly')
        task_combo.grid(row=0,column=0,sticky='ew');tasks.columnconfigure(0,weight=1)
        task_combo.bind('<<ComboboxSelected>>',lambda _e:app._update_validation_and_command())
        ttk.Button(tasks,text='Save acquired map',command=self.save_map).grid(row=1,column=0,sticky='ew',pady=3)
        self.odometry_feedback=tk.StringVar(value='Sensors/SLAM/Nav2 are experimental candidates, not qualified higher modes.')
        ttk.Label(self,textvariable=self.odometry_feedback,wraplength=310).grid(row=11,column=0,sticky='ew')
        app.bind('<FocusOut>',lambda _e:self.after_idle(self.focus_check),add='+');self.job=self.after(50,self.tick)

    def selected(self):return self.app._gait_selectable()

    def connect(self):
        if self.node:return True
        if rclpy is None:self.status.set('Source the ROS workspace before starting GUI');return False
        if not rclpy.ok():rclpy.init(args=None)
        self.node=rclpy.create_node('gait_gui_'+self.operator[-12:]);self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.command_pub=self.node.create_publisher(String,'/gait/command',10);self.hb_pub=self.node.create_publisher(String,'/gait/operator_heartbeat',10)
        self.node.create_subscription(String,'/gait/state',self.observe,10)
        self.node.create_subscription(String,'/gait/odometry_state',self.observe_odometry,10)
        self.status.set('Connected neutral; wait for actual support, then Arm walking');return True

    def observe(self,msg):
        try:
            if len(msg.data)>65536:return
            data=json.loads(msg.data)
            if data.get('schema_version')!=1:return
            if not self.runtime or data['session_id']!=self.runtime['session_id']:
                self.schedule.set(data['gait'])
                if self.runtime:self.app._stop_drive(keep_input_enabled=True)
            interrupted=bool(self.runtime and self.runtime.get('enabled') and not data['enabled'])
            self.runtime,self.runtime_at=data,time.monotonic()
            if interrupted:self.app._stop_drive(keep_input_enabled=True)
            default={'hexapod_ros2':'tripod','lab_octopod':'tetrapod','lab_centipede':'wave'}.get(data['robot_id'],'wave')
            self.spine_phase=tk.DoubleVar(value=data.get('spine_phase',0.0))
            self.spine_freq=tk.DoubleVar(value=data.get('spine_freq',1.0))
            self.combo.configure(values=list(dict.fromkeys([default,'ripple','wave'])))
            p=data.get('position');forces=(data.get('actual_contacts') or {}).get('normal_forces_N',{})
            text=f"{data['robot_id']} · {'ARMED' if data['enabled'] else 'HELD'} · {data['reason']}\n"
            text+=f"{data['physical_legs']} physical legs / {data['actuated_joints']} joints · {data['gait']}\n"
            if p:text+=f"Actual body ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f}) m\n"
            if data.get('tilt_rad') is not None:text+=f"Tilt {data['tilt_rad']:.3f} rad · IK error {data['max_ik_error_m']*1000:.2f} mm\n"
            self.feedback.set(text+'Measured normal forces N:\n'+', '.join(f'{k}: {v:.1f}' for k,v in forces.items()))
        except (ValueError,KeyError,TypeError):return

    def send(self,op,**extra):
        if not self.node or not self.runtime or time.monotonic()-self.runtime_at>1.:
            self.status.set('Connect to fresh gait state first');return False
        if op!='stop' and (not self.selected() or self.runtime['robot_id']!=self.app.robot_var.get() or not self.runtime.get('control_allowed')):
            self.status.set('Select the running source robot in MuJoCo Gait mode');return False
        self.command_pub.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'],operator_id=self.operator,op=op,**extra))))
        self.status.set('Requested '+op+'; inspect actual state below');return True

    def operation(self,op):
        if not self.node:self.connect();return
        self.app._stop_drive(keep_input_enabled=True)
        extra={'gait':self.schedule.get(),'spine_phase':self.spine_phase.get(),'spine_freq':self.spine_freq.get()} if op=='gait' else {}
        self.send(op,**extra)

    def stop(self):
        if self.node and self.runtime:self.send('stop')

    def observe_odometry(self,msg):
        try:
            data=json.loads(msg.data)
            self.odometry_feedback.set(f"Contact odometry: {data['samples']} samples · XY {data['position']}\n"
                f"Support spread {data['velocity_scatter_m_s']:.3f} m/s · {data['source']}")
        except (ValueError,KeyError,TypeError):pass

    def save_map(self):
        if not self.runtime:return
        task=self.runtime.get('task')
        directory=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'maps'
        directory.mkdir(exist_ok=True)
        target=directory/('gait-'+self.runtime['robot_id']+'-'+time.strftime('%Y%m%d-%H%M%S'))
        if task=='slam':
            request=json.dumps(dict(map_topic='/gait/map',map_url=str(target),image_format='pgm',map_mode='trinary',free_thresh=.25,occupied_thresh=.65))
            command=['timeout','15','ros2','service','call','/gait/map_saver/save_map','nav2_msgs/srv/SaveMap',request]
            threading.Thread(target=self.app._run_aux_command,args=(command,'Gait map save'),daemon=True).start()
        elif task=='3d_slam' and self.runtime.get('directory'):
            source=Path(self.runtime['directory'])/'gait-rtabmap.db'
            if not source.is_file():self.status.set('No acquired RTAB-Map database yet');return
            try:
                with sqlite3.connect('file:'+str(source)+'?mode=ro',uri=True,timeout=2.) as old:
                    with sqlite3.connect(str(target)+'.db') as new:old.backup(new)
                self.status.set('Saved actual RTAB-Map database: '+str(target)+'.db')
            except sqlite3.Error as exc:self.status.set('Map backup failed: '+str(exc))
        else:self.status.set('Run experimental slam or 3d_slam and acquire measurements first')

    def focus_check(self):
        if not self.closed and self.focus_get() is None:self.app._stop_drive(keep_input_enabled=True)

    def tick(self):
        if self.closed:return
        if self.executor and rclpy.ok():self.executor.spin_once(timeout_sec=0.)
        now=time.monotonic()
        if self.runtime and now-self.runtime_at<1.:
            if self.selected() and self.runtime['robot_id']==self.app.robot_var.get():
                if now-self.last_hb>.2:
                    self.last_hb=now;self.hb_pub.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'],operator_id=self.operator))))
            elif self.runtime['enabled']:self.stop()
        self.job=self.after(50,self.tick)

    def close(self):
        self.stop();self.closed=True;self.after_cancel(self.job)
        if self.executor:self.executor.shutdown(timeout_sec=.1)
        if self.node:self.node.destroy_node()
        self.node=self.executor=None
