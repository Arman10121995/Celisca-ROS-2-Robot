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
from tkinter import filedialog, ttk
from .workspace_ui import ScrollPanel
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
except ImportError:rclpy=None


class GaitControls(ScrollPanel):
    def __init__(self,notebook,app):
        super().__init__(notebook,width=320);notebook.add(self,text='Gait');self.app=app
        body=self.body
        self.node=self.executor=None;self.runtime=None;self.runtime_at=-math.inf;self.closed=False
        self.operator='gait-gui-'+uuid.uuid4().hex;self.last_hb=0.
        self.estimated_pose=None;self.estimated_stamp=None
        self.goal_handle=None;self.goal_generation=0;self.goal_sending=False
        self.status=tk.StringVar(value='Select a six/eight/many-legged robot, MuJoCo and Gait mode.')
        self.feedback=tk.StringVar(value='Awaiting actual body, joints and foot forces')
        self.phase=tk.DoubleVar(value=0.0)
        self.schedule=tk.StringVar(value='wave');body.columnconfigure(0,weight=1)
        self.spine_phase=tk.DoubleVar(value=0.0);self.spine_freq=tk.DoubleVar(value=1.0)
        ttk.Label(body,text='Source gait · joint physics',font=('',11,'bold')).grid(row=0,column=0,sticky='w')
        ttk.Label(body,textvariable=self.status,wraplength=310).grid(row=1,column=0,sticky='ew',pady=7)
        for row,label,command in [(2,'Connect (neutral)',self.connect),(3,'Arm walking',lambda:self.operation('arm')),
                                  (4,'Stop / hold current targets',lambda:app._stop_drive(keep_input_enabled=True)),(5,'Return to source stance',lambda:self.operation('stand')),
                                  (6,'Open latched Drive / WASD controls',lambda:app.control_notebook.select(app.drive_page))]:
            ttk.Button(body,text=label,command=command).grid(row=row,column=0,sticky='ew',pady=2)
        combo=ttk.Frame(self);combo.grid(row=7,column=0,sticky='ew',pady=6)
        ttk.Label(combo,text='Schedule').pack(side='left')
        self.combo=ttk.Combobox(combo,textvariable=self.schedule,state='readonly',values=['wave','ripple','tripod'],width=12)
        self.combo.pack(side='left',padx=5);ttk.Button(combo,text='Apply',command=lambda:self.operation('gait')).pack(side='left');ttk.Label(combo,text='Phase').pack(side='left',padx=(10,0));ttk.Scale(combo,from_=0.0,to=1.0,variable=self.phase,orient='horizontal',length=50).pack(side='left',fill='x',expand=True,padx=2)
        ttk.Label(body,textvariable=self.feedback,wraplength=310,justify='left').grid(row=8,column=0,sticky='ew',pady=6)
        ttk.Label(body,text='Connect and Arm start without travel. Use Drive for latching, ramps, WASD/joystick and Space.\n'
                  'Stop/loss halts gait phases and holds bounded joint targets; bodies remain free in physics.\n'
                  'Measured contacts are separate from planned stance. Experimental contact odometry and mapping/navigation can be selected below; terrain and backend qualification remain pending.',
                  wraplength=310).grid(row=9,column=0,sticky='ew')
        tasks=ttk.LabelFrame(body,text='Experimental sensor task · applied on next Run',padding=5)
        tasks.grid(row=10,column=0,sticky='ew',pady=5)
        task_combo=ttk.Combobox(tasks,textvariable=app.gait_task_var,values=('gait','loc','slam','3d_slam','nav'),state='readonly')
        task_combo.grid(row=0,column=0,sticky='ew');tasks.columnconfigure(0,weight=1)
        task_combo.bind('<<ComboboxSelected>>',lambda _e:app._update_validation_and_command())
        ttk.Button(tasks,text='Save acquired map',command=self.save_map).grid(row=1,column=0,sticky='ew',pady=3)
        ttk.Label(tasks,text='Saved 2D map for loc/nav').grid(row=2,column=0,sticky='w')
        map_entry=ttk.Entry(tasks,textvariable=app.gait_map_yaml_var);map_entry.grid(row=3,column=0,sticky='ew')
        map_entry.bind('<FocusOut>',lambda _:app._update_validation_and_command())
        ttk.Button(tasks,text='Choose YAML',command=lambda:self.choose_saved(False)).grid(row=3,column=1)
        ttk.Label(tasks,text='Saved 3D database').grid(row=4,column=0,sticky='w')
        db_entry=ttk.Entry(tasks,textvariable=app.gait_database_var);db_entry.grid(row=5,column=0,sticky='ew')
        db_entry.bind('<FocusOut>',lambda _:app._update_validation_and_command())
        ttk.Button(tasks,text='Choose DB',command=lambda:self.choose_saved(True)).grid(row=5,column=1)
        ttk.Checkbutton(tasks,text='3D localization in saved database',variable=app.gait_localization_var,
            command=app._update_validation_and_command).grid(row=6,column=0,columnspan=2,sticky='w')
        self.odometry_feedback=tk.StringVar(value='Sensors/SLAM/Nav2 are experimental candidates, not qualified higher modes.')
        ttk.Label(body,textvariable=self.odometry_feedback,wraplength=310).grid(row=11,column=0,sticky='ew')
        navigation=ttk.LabelFrame(body,text='Navigation goal · map coordinates',padding=5)
        navigation.grid(row=12,column=0,sticky='ew',pady=5)
        self.goal_values={key:tk.DoubleVar(value=0.) for key in ('x','y','yaw')}
        for col,(key,var) in enumerate(self.goal_values.items()):
            ttk.Label(navigation,text=key.upper()).grid(row=0,column=col)
            ttk.Entry(navigation,textvariable=var,width=7).grid(row=1,column=col,sticky='ew')
            navigation.columnconfigure(col,weight=1)
        ttk.Button(navigation,text='Use estimated pose',command=self.use_estimated_pose).grid(row=2,column=0,columnspan=3,sticky='ew',pady=3)
        ttk.Button(navigation,text='Send goal · Nav2',command=self.send_goal).grid(row=3,column=0,columnspan=2,sticky='ew')
        ttk.Button(navigation,text='Cancel',command=self.cancel_goal).grid(row=3,column=2,sticky='ew')
        ttk.Label(navigation,text='Uses contact odometry + AMCL, A* and bounded regulated pure pursuit. '
            'Connect and Arm walking after launching the nav task. Actual goal results remain separate from body/terrain qualification.',
            wraplength=310).grid(row=4,column=0,columnspan=3,sticky='w')
        spines=ttk.LabelFrame(body,text='Segmented body wave · Apply schedule to use',padding=5)
        spines.grid(row=13,column=0,sticky='ew')
        for col,(label,var) in enumerate([('Phase cycles',self.spine_phase),('Frequency multiplier',self.spine_freq)]):
            ttk.Label(spines,text=label).grid(row=0,column=col)
            ttk.Entry(spines,textvariable=var,width=10).grid(row=1,column=col,sticky='ew')
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
        from nav_msgs.msg import Odometry
        from std_srvs.srv import Trigger
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from tf2_ros import Buffer,TransformListener
        self.tf=Buffer(node=self.node);self.listener=TransformListener(self.tf,self.node)
        self.node.create_subscription(Odometry,'/gait/odometry',self.observe_pose,10)
        self.node.create_subscription(String,'/gait/mapping/state',self.observe_archive,10)
        self.map_client=self.node.create_client(Trigger,'/gait/mapping/save')
        self.navigation_client=ActionClient(self.node,NavigateToPose,'/gait/navigate_to_pose')
        self.status.set('Connected neutral; wait for actual support, then Arm walking');return True

    def observe(self,msg):
        try:
            if len(msg.data)>65536:return
            data=json.loads(msg.data)
            if data.get('schema_version')!=1:return
            if not self.runtime or data['session_id']!=self.runtime['session_id']:
                self.schedule.set(data['gait'])
                self.spine_phase.set(data.get('spine_phase',0.))
                self.spine_freq.set(data.get('spine_freq',1.))
                self.goal_generation+=1
                if self.runtime:self.app._stop_drive(keep_input_enabled=True)
            interrupted=bool(self.runtime and self.runtime.get('enabled') and not data['enabled'])
            self.runtime,self.runtime_at=data,time.monotonic()
            if interrupted:self.app._stop_drive(keep_input_enabled=True)
            default={'hexapod_ros2':'tripod','lab_octopod':'tetrapod','lab_centipede':'wave'}.get(data['robot_id'],'wave')
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
        if op=='arm' and (self.goal_sending or self.goal_handle is not None):
            self.stop();self.status.set('Waiting for actual Nav2 cancellation before re-arming');return
        self.app._stop_drive(keep_input_enabled=True)
        extra={'gait':self.schedule.get(),'phase':self.phase.get(),'spine_phase':self.spine_phase.get(),'spine_freq':self.spine_freq.get()} if op=='gait' else {}
        self.send(op,**extra)

    def stop(self):
        self.cancel_goal()
        if self.node and self.runtime:self.send('stop')

    def observe_odometry(self,msg):
        try:
            data=json.loads(msg.data)
            self.odometry_feedback.set(f"Contact odometry: {data['samples']} samples · XY {data['position']}\n"
                f"Support spread {data['velocity_scatter_m_s']:.3f} m/s · {data['source']}")
        except (ValueError,KeyError,TypeError):pass

    def save_map(self):
        from std_srvs.srv import Trigger
        if not self.node or not self.runtime or self.runtime.get('task') not in ('slam','3d_slam'):
            self.status.set('Run slam/3d_slam, connect, and acquire actual measurements first');return
        if not self.map_client.service_is_ready():
            self.status.set('Map archive service is not ready');return
        future=self.map_client.call_async(Trigger.Request())
        def done(future):
            try:
                result=future.result();self.status.set(result.message)
            except Exception as exc:self.status.set('Map save failed: '+str(exc))
        future.add_done_callback(done)

    def choose_saved(self,three_d):
        path=filedialog.askopenfilename(parent=self,title='Choose acquired gait map',
            filetypes=[('RTAB-Map database','*.db')] if three_d else [('Occupancy YAML','*.yaml')])
        if path:
            (self.app.gait_database_var if three_d else self.app.gait_map_yaml_var).set(path)
            if three_d:self.app.gait_localization_var.set(True)
            self.app._update_validation_and_command()

    def observe_archive(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('status')=='saved' and data.get('result'):
                result=data['result']
                if result.get('robot') and result['robot']!=self.app.robot_var.get():return
                if result.get('map_yaml'):self.app.gait_map_yaml_var.set(result['map_yaml'])
                if result.get('database'):self.app.gait_database_var.set(result['database'])
                self.app._update_validation_and_command()
            self.odometry_feedback.set(data.get('reason') or 'Acquired map saved; path filled for the next Run')
        except (ValueError,KeyError,TypeError):pass

    def observe_pose(self,msg):
        from tf2_ros import TransformException
        from scipy.spatial.transform import Rotation
        import numpy as np
        try:
            transform=self.tf.lookup_transform('map',msg.header.frame_id,rclpy.time.Time())
            t,q=transform.transform.translation,transform.transform.rotation
            r=Rotation.from_quat([q.x,q.y,q.z,q.w]);p=msg.pose.pose.position;o=msg.pose.pose.orientation
            xyz=r.apply([p.x,p.y,p.z])+[t.x,t.y,t.z]
            yaw=(r*Rotation.from_quat([o.x,o.y,o.z,o.w])).as_euler('xyz')[2]
            if np.isfinite([*xyz,yaw]).all():
                self.estimated_pose=(float(xyz[0]),float(xyz[1]),float(yaw))
                self.estimated_stamp=msg.header.stamp
        except (TransformException,ValueError):pass

    def use_estimated_pose(self):
        if self.estimated_pose is None:self.status.set('No estimated map pose yet');return
        for key,value in zip(('x','y','yaw'),self.estimated_pose):self.goal_values[key].set(value)

    def send_goal(self):
        if (not self.node or not self.runtime or time.monotonic()-self.runtime_at>1. or self.runtime.get('task')!='nav'
                or not self.runtime.get('enabled') or self.runtime.get('operator_id')!=self.operator):
            self.status.set('Launch nav, Connect, then Arm walking before dispatch');return
        if self.goal_sending or self.goal_handle is not None or not self.navigation_client.server_is_ready():
            self.status.set('Wait for Nav2 readiness or cancel the active goal first');return
        from nav2_msgs.action import NavigateToPose
        try:
            x,y,yaw=[self.goal_values[k].get() for k in ('x','y','yaw')]
            if not all(math.isfinite(v) for v in (x,y,yaw)):raise ValueError('Finite map coordinates required')
            request=NavigateToPose.Goal();request.pose.header.frame_id='map'
            if self.estimated_stamp is not None:request.pose.header.stamp=self.estimated_stamp
            request.pose.pose.position.x,request.pose.pose.position.y=x,y
            request.pose.pose.orientation.z,request.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            generation=self.goal_generation;self.goal_sending=True
            def accepted(future):
                self.goal_sending=False
                try:
                    handle=future.result()
                    if not handle.accepted:self.status.set('Nav2 rejected the actual gait goal');return
                    self.goal_handle=handle
                    def finished(future):
                        if self.goal_handle is handle:self.goal_handle=None
                        if generation==self.goal_generation:
                            try:self.status.set('Actual Nav2 result status='+str(future.result().status)+'; inspect estimated/body settling separately')
                            except Exception as exc:self.status.set('Nav2 result unavailable: '+str(exc))
                    handle.get_result_async().add_done_callback(finished)
                    if generation!=self.goal_generation:handle.cancel_goal_async();return
                except Exception as exc:self.status.set('Navigation action failed: '+str(exc))
            self.navigation_client.send_goal_async(request).add_done_callback(accepted)
        except (ValueError,tk.TclError) as exc:self.status.set(str(exc))

    def cancel_goal(self):
        self.goal_generation+=1
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        if self.node and self.runtime and self.runtime.get('enabled') and self.runtime.get('input_owner')=='navigation':
            self.send('stop')

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
