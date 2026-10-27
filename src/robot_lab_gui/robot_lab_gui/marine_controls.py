"""Native ROV controls in the Launch control column, with neutral input enable."""
import json
import math
import time
import uuid
import tkinter as tk
from tkinter import ttk,filedialog
from .drive_control import LinuxJoystick,RampDrive

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist,Vector3
    from std_msgs.msg import String
except ImportError:
    rclpy=None


class MarineControls(ttk.Frame):
    def __init__(self,notebook,app):
        super().__init__(notebook,padding=8)
        self.app=app
        notebook.add(self,text='Marine')
        self.columnconfigure(0,weight=1)
        self.node=self.executor=self.command_pub=self.hb_pub=self.drive_pub=None
        self.mission_pub=None
        self.runtime=None;self.runtime_at=-math.inf
        self.operator='marine-gui-'+uuid.uuid4().hex
        self.last_hb=self.last_drive=0.
        self.latches=set();self.keys=set();self.key_jobs={}
        self.ramp=RampDrive(max_linear=.35,min_linear=-.35,max_angular=.35,min_angular=-.35)
        self.joystick=LinuxJoystick();self.driving=False;self.closed=False
        self.keyboard=tk.BooleanVar(value=False)
        self.linear_delta=tk.DoubleVar(value=.01);self.angular_delta=tk.DoubleVar(value=.025)
        self.depth=tk.DoubleVar(value=1.)
        self.status=tk.StringVar(value='Choose ROV or AUV + marine world + Marine mode. Experimental native water forces.')
        self.feedback=tk.StringVar(value='Awaiting actual body/force feedback')
        ttk.Label(self,text='Marine · physical propulsion',font=('',11,'bold')).grid(row=0,column=0,sticky='w')
        ttk.Label(self,textvariable=self.status,wraplength=300).grid(row=1,column=0,sticky='ew',pady=5)
        connect=ttk.Frame(self);connect.grid(row=2,column=0,sticky='ew')
        ttk.Button(connect,text='Connect',command=self.connect).pack(side='left',expand=True,fill='x')
        self.arm_label=tk.StringVar(value='Arm depth/heading hold')
        ttk.Button(connect,textvariable=self.arm_label,command=lambda:self.operation('arm')).pack(side='left',expand=True,fill='x')
        pad=ttk.Frame(self);pad.grid(row=3,column=0,sticky='ew',pady=5)
        self.buttons={}
        for label,direction,row,col in [('↖',(1,1,0),0,0),('↑',(1,0,0),0,1),('↗',(1,-1,0),0,2),
                                       ('↶',(0,1,0),1,0),('↓',(-1,0,0),1,1),('↷',(0,-1,0),1,2),
                                       ('Strafe ←',(0,0,1),2,0),('Strafe →',(0,0,-1),2,2)]:
            button=ttk.Button(pad,text=label,style='LatchedDrive.TButton',command=lambda d=direction:self.latch(d))
            button.grid(row=row,column=col,sticky='ew',padx=2,pady=2)
            self.buttons[direction]=button
            for event in ('<ButtonRelease-1>','<Leave>','<Enter>','<FocusOut>'):
                button.bind(event,lambda _e:self.after_idle(self.sync),add='+')
        ttk.Button(pad,text='STOP',command=self.stop).grid(row=2,column=1,sticky='ew',padx=2)
        for col in range(3):pad.columnconfigure(col,weight=1)
        ttk.Checkbutton(self,text='WASD + joystick · Space stops',variable=self.keyboard,command=self.clear).grid(row=4,column=0,sticky='w')
        increments=ttk.Frame(self);increments.grid(row=5,column=0,sticky='ew')
        ttk.Label(increments,text='Δ linear / yaw per 0.1 s').pack(side='left')
        for var in (self.linear_delta,self.angular_delta):ttk.Entry(increments,textvariable=var,width=6).pack(side='left',padx=3)
        depth=ttk.LabelFrame(self,text='Depth below water surface (m)',padding=5);depth.grid(row=6,column=0,sticky='ew',pady=7)
        for col,(label,op) in enumerate([('Depth −','depth_up'),('Hold','hold'),('Depth +','depth_down')]):
            ttk.Button(depth,text=label,command=lambda o=op:self.operation(o)).grid(row=0,column=col,sticky='ew',padx=2)
            depth.columnconfigure(col,weight=1)
        ttk.Entry(depth,textvariable=self.depth,width=8).grid(row=1,column=0,sticky='ew',pady=4)
        ttk.Button(depth,text='Set depth',command=lambda:self.operation('set_depth')).grid(row=1,column=1,sticky='ew')
        ttk.Button(depth,text='Near surface',command=lambda:self.operation('surface')).grid(row=1,column=2,sticky='ew')
        ttk.Label(self,textvariable=self.feedback,wraplength=300,justify='left').grid(row=7,column=0,sticky='ew',pady=5)
        ttk.Label(self,text='Click to latch; click again to brake. Connect/keyboard starts neutral.\n'
                  'Stop removes propulsion; physical drift/coasting can continue.\n'
                  'AUV fins need forward flow; no hover, strafe or pivot.\n'
                  'Manual inner control uses simulator poses. Marine missions adds noisy pressure/DVL odometry and optional real RGB-D SLAM; physical qualification remains pending.',
                  wraplength=300).grid(row=8,column=0,sticky='ew')
        app.bind('<KeyPress>',self.key_press,add='+');app.bind('<KeyRelease>',self.key_release,add='+')
        app.bind('<FocusOut>',lambda _e:self.after_idle(self._focus_check),add='+')
        self.job=self.after(50,self.tick)
        self.missions=MarineMissions(notebook,app,self)

    def selected(self):
        return self.app._marine_selectable()

    def connect(self):
        if self.node:return True
        if rclpy is None:self.status.set('Start the GUI from a sourced ROS workspace');return False
        if not rclpy.ok():rclpy.init(args=None)
        self.node=rclpy.create_node('marine_gui_'+self.operator[-12:])
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.command_pub=self.node.create_publisher(String,'/marine/command',10)
        self.hb_pub=self.node.create_publisher(String,'/marine/operator_heartbeat',10)
        self.drive_pub=self.node.create_publisher(Twist,'/marine/key_vel',10)
        self.mission_pub=self.node.create_publisher(String,'/marine/mission/command',10)
        self.node.create_subscription(String,'/marine/state',self.observe,10)
        self.node.create_subscription(String,'/marine/mission/state',self.missions.observe,10)
        self.status.set('Connected; no movement input sent. Wait for actual state, then Arm.')
        return True

    def observe(self,msg):
        try:
            if len(msg.data)>65536:return
            data=json.loads(msg.data)
            if data.get('schema_version')!=1 or not self.app.robot_profiles.get(data.get('robot_id'),{}).get('marine_model'):return
            if data['robot_id']!=self.app.robot_var.get():return
            if self.runtime and self.runtime['session_id']!=data['session_id']:self.clear()
            self.runtime,self.runtime_at=data,time.monotonic()
            p=data.get('position');v=data.get('velocity_world')
            text=f"{'ARMED' if data['enabled'] else 'DISARMED'} · {data['reason']}\n"
            if p:
                target=data.get('target_depth')
                text+=f"Depth {data['depth']:.2f} m · target {target if target is not None else 'none'}\n"
                text+=f"Position ({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f}) m\n"
            if v:text+=f"Measured world velocity ({v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}) m/s\n"
            text+='Thrust N: '+', '.join(f'{f:.1f}' for f in data['commanded_thruster_forces_N'])
            if data.get('propulsion_kind')=='propeller_fins':
                text+='\nFin targets rad: '+', '.join(f'{v:.3f}' for v in data['fin_targets_rad'])
                text+='\nMeasured fins rad: '+', '.join(f"{k}: {v:.3f}" for k,v in data.get('actual_joints_rad',{}).items() if k!='propeller_joint')
                text+='\nCruise guidance '+('active' if data['cruise_guidance_active'] else 'requires forward flow')
            text+=f"\nActual body samples: {data['body_samples']}"
            self.feedback.set(text)
            if not data['enabled']:self.clear()
        except (KeyError,ValueError,TypeError):return

    def send(self,op,**extra):
        if not self.node or not self.runtime or time.monotonic()-self.runtime_at>1.:
            self.status.set('Connect to actual marine telemetry first');return False
        if op!='stop' and (not self.selected() or self.runtime['robot_id']!=self.app.robot_var.get() or not self.runtime.get('control_allowed')):
            self.status.set('Select Marine mode in a compatible Gazebo water world');return False
        self.command_pub.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'],operator_id=self.operator,op=op,**extra),allow_nan=False)))
        self.status.set('Requested '+op+'; actual feedback below')
        return True

    def operation(self,op):
        if not self.node:self.connect();return
        try:
            if op=='hold':self.clear()
            self.send(op,**({'depth':self.depth.get()} if op=='set_depth' else {}))
        except (ValueError,tk.TclError) as exc:self.status.set(str(exc))

    def clear(self):
        self.latches.clear();self.keys.clear();self.ramp.linear=self.ramp.angular=0.
        self.driving=False;self.joystick.close();self.sync()

    def sync(self):
        if not self.closed:
            for direction,button in self.buttons.items():button.state(['selected','pressed'] if direction in self.latches else ['!selected','!pressed'])

    def stop(self):
        self.clear()
        if self.node and self.runtime:self.send('stop')
        if self.mission_pub and self.runtime:self.missions.send('cancel')

    def latch(self,direction):
        if not self.runtime or not self.runtime.get('enabled') or not self.selected():
            self.status.set('Arm Marine control first');return
        self.latches.remove(direction) if direction in self.latches else self.latches.add(direction)
        self.sync()

    def _focus_check(self):
        if not self.closed and self.focus_get() is None:self.stop()

    def active_keys(self):
        return (self.keyboard.get() and self.selected() and self.app.notebook.select()==str(self.app.launch_tab)
                and self.app.control_notebook.select()==str(self)
                and not isinstance(self.focus_get(),(tk.Entry,ttk.Entry,tk.Text,ttk.Combobox)))

    def key_press(self,event):
        key=event.keysym.lower()
        if not self.active_keys():return
        if key=='space':self.stop();return 'break'
        if key in ('w','s','a','d'):
            job=self.key_jobs.pop(key,None)
            if job:self.after_cancel(job)
            self.keys.add(key);return 'break'

    def key_release(self,event):
        key=event.keysym.lower()
        if key in self.keys:
            self.key_jobs[key]=self.after(35,lambda:self.keys.discard(key));return 'break'

    def tick(self):
        if self.closed:return
        now=time.monotonic()
        auv=self.app._robot_config().get('marine_propulsion')=='propeller_fins'
        self.arm_label.set('Arm AUV cruise guidance' if auv else 'Arm depth/heading hold')
        for direction,button in self.buttons.items():
            if direction[2]:button.state(['disabled'] if auv else ['!disabled'])
        if self.executor and rclpy.ok():self.executor.spin_once(timeout_sec=0.)
        state=self.runtime
        if state and now-self.runtime_at<1. and self.selected() and state['robot_id']==self.app.robot_var.get():
            if now-self.last_hb>.2:
                self.last_hb=now
                self.hb_pub.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id=self.operator))))
            if state['enabled'] and state['ready'] and state.get('operator_id')==self.operator and now-self.last_drive>.1:
                self.last_drive=now
                linear=sum(d[0] for d in self.latches);angular=sum(d[1] for d in self.latches);lateral=sum(d[2] for d in self.latches)
                if self.active_keys():
                    jl,ja=self.joystick.poll()
                    linear+=int('w' in self.keys)-int('s' in self.keys)+jl
                    angular+=int('a' in self.keys)-int('d' in self.keys)+ja
                try:
                    dl,da=self.linear_delta.get(),self.angular_delta.get()
                    if not 0<dl<=.04 or not 0<da<=.05:raise ValueError('Marine increments: linear (0,.04], yaw (0,.05]')
                    limits=state.get('limits',{})
                    self.ramp.max_linear=float(limits.get('surge',.35));self.ramp.min_linear=-self.ramp.max_linear
                    self.ramp.max_angular=float(limits.get('yaw',.35));self.ramp.min_angular=-self.ramp.max_angular
                    vx,wz=self.ramp.step(linear,angular,dl,da)
                    sway=float(limits.get('sway',.25));vy=max(-sway,min(sway,lateral*.15))
                    if linear or angular or lateral or self.driving or vx or wz:
                        message=Twist();message.linear.x=vx;message.linear.y=vy;message.angular.z=wz
                        self.drive_pub.publish(message)
                        self.driving=bool(linear or angular or lateral or vx or wz)
                except (ValueError,tk.TclError) as exc:self.stop();self.status.set(str(exc))
        elif state and state['enabled']:
            self.stop()
        self.job=self.after(50,self.tick)

    def close(self):
        self.stop();self.closed=True;self.after_cancel(self.job)
        if self.executor:self.executor.shutdown(timeout_sec=.1)
        if self.node:self.node.destroy_node()
        self.node=self.executor=None


class MarineMissions(ttk.Frame):
    """Native mission/map page sharing the Marine operator and Stop lease."""
    def __init__(self,notebook,app,marine):
        super().__init__(notebook,padding=8)
        self.app,self.marine=app,marine
        notebook.add(self,text='Marine missions')
        self.columnconfigure(0,weight=1)
        self.runtime=None
        self.mission_runtime_at=-math.inf
        self.last_saved_database=None
        self.goal={k:tk.DoubleVar(value=v) for k,v in [('x',1.),('y',0.),('z',-1.)]}
        self.ahead_distance=tk.DoubleVar(value=3.)
        self.feedback=tk.StringVar(value='Sensor odometry starts with the marine launch. Connect Marine to see estimates.')
        ttk.Label(self,text='Estimated-pose 3D missions',font=('',11,'bold')).grid(row=0,column=0,sticky='w')
        ttk.Button(self,text='Connect Marine telemetry',command=marine.connect).grid(row=1,column=0,sticky='ew',pady=5)
        coords=ttk.LabelFrame(self,text='World target (metres; underwater Z is negative)',padding=5)
        coords.grid(row=2,column=0,sticky='ew')
        for col,(key,var) in enumerate(self.goal.items()):
            ttk.Label(coords,text=key.upper()).grid(row=0,column=col)
            ttk.Entry(coords,textvariable=var,width=8).grid(row=1,column=col,sticky='ew',padx=2)
            coords.columnconfigure(col,weight=1)
        ttk.Button(coords,text='Use estimated position',command=self.use_position).grid(row=2,column=0,columnspan=3,sticky='ew',pady=4)
        ttk.Button(coords,text='Waypoint ahead · metres',command=self.ahead).grid(row=3,column=0,columnspan=2,sticky='ew')
        ttk.Entry(coords,textvariable=self.ahead_distance,width=6).grid(row=3,column=2,sticky='ew',padx=2)
        buttons=ttk.Frame(self);buttons.grid(row=3,column=0,sticky='ew',pady=6)
        for label,op in [('Plan','plan'),('Execute','execute'),('Cancel / Stop','cancel')]:
            ttk.Button(buttons,text=label,command=lambda operation=op:self.send(operation)).pack(side='left',expand=True,fill='x')
        mapping=ttk.LabelFrame(self,text='RGB-D mapping · ROV / authored AUV kit',padding=5)
        mapping.grid(row=4,column=0,sticky='ew')
        ttk.Checkbutton(mapping,text='Enable RTAB-Map on next Run',variable=app.marine_slam_var,
                        command=app._update_validation_and_command).grid(row=0,column=0,columnspan=2,sticky='w')
        ttk.Checkbutton(mapping,text='Localize in selected saved database',variable=app.marine_localization_var,
                        command=app._update_validation_and_command).grid(row=1,column=0,columnspan=2,sticky='w')
        ttk.Entry(mapping,textvariable=app.marine_database_var,width=26).grid(row=2,column=0,sticky='ew')
        ttk.Button(mapping,text='Choose DB',command=self.choose_database).grid(row=2,column=1)
        ttk.Button(mapping,text='Save acquired 3D map',command=lambda:self.send('save_map')).grid(row=3,column=0,columnspan=2,sticky='ew',pady=4)
        mapping.columnconfigure(0,weight=1)
        ttk.Label(self,textvariable=self.feedback,wraplength=310,justify='left').grid(row=5,column=0,sticky='ew',pady=8)
        ttk.Label(self,text='Pressure and Janus-DVL are explicit noisy simulator sensor models; native IMU supplies attitude.\n'
                  'ROV can follow checked 3D routes. Source AUV supports forward-aligned cruise transects with a clear coasting corridor.\n'
                  'Arm on Marine before Execute. Manual input interrupts missions. Stop/heartbeat or sensor loss closes the actuator gate.\n'
                  'Saved SLAM, sensor accuracy and navigation remain experimental until physical missions are recorded.',
                  wraplength=310).grid(row=6,column=0,sticky='ew')

    def choose_database(self):
        path=filedialog.askopenfilename(parent=self,title='Select saved marine RTAB-Map database',filetypes=[('RTAB-Map database','*.db'),('All files','*')])
        if path:
            self.app.marine_database_var.set(path)
            self.app.marine_slam_var.set(True)
            self.app._update_validation_and_command()

    def use_position(self):
        if self.runtime and self.runtime.get('position'):
            for key,value in zip('xyz',self.runtime['position']):self.goal[key].set(value)

    def ahead(self):
        try:
            if (not self.runtime or time.monotonic()-self.mission_runtime_at>1.
                    or not self.runtime.get('ready') or self.runtime.get('estimated_yaw') is None):
                raise ValueError('Wait for a fresh estimated marine pose')
            distance=float(self.ahead_distance.get())
            auv=self.runtime['robot_id']=='tethys_auv'
            if not math.isfinite(distance) or not (2. if auv else .1)<=distance<=20.:
                raise ValueError('Ahead distance must be 2–20 m for the cruise AUV, or 0.1–20 m for the ROV')
            heading=self.runtime['estimated_yaw']+(math.pi if auv else 0.)
            p=self.runtime['position']
            if not all(math.isfinite(v) for v in [*p,heading]):raise ValueError('Finite estimated marine position/heading required')
            for key,value in zip('xyz',[p[0]+distance*math.cos(heading),p[1]+distance*math.sin(heading),p[2]]):
                self.goal[key].set(value)
            self.feedback.set('Ahead waypoint filled from estimated heading. Plan checks source/observed clearance; Execute remains explicit.')
        except (ValueError,TypeError,tk.TclError) as exc:self.feedback.set(str(exc))

    def observe(self,msg):
        try:
            data=json.loads(msg.data)
            if data['robot_id']!=self.app.robot_var.get():return
            self.runtime=data
            self.mission_runtime_at=time.monotonic()
            saved=data.get('saved_map') or {}
            if data.get('mapping_status')=='saved' and saved.get('database') and saved['database']!=self.last_saved_database:
                self.last_saved_database=saved['database']
                self.app.marine_database_var.set(saved['database'])
                self.app._update_validation_and_command()
            text=data['status']+' · '+data['reason']
            if data.get('position'):
                text+='\nEstimated XYZ: '+', '.join(f'{v:.3f}' for v in data['position'])
            text+=f"\nObserved occupied cells: {data.get('observed_cells',0)}\nArtifacts: {data.get('directory','')}"
            self.feedback.set(text)
        except (ValueError,KeyError,TypeError):return

    def send(self,op):
        parent=self.marine
        if not parent.node:
            parent.connect();return
        if not parent.runtime or time.monotonic()-parent.runtime_at>1.:
            self.feedback.set('Wait for fresh Marine telemetry');return
        try:
            extra={key:var.get() for key,var in self.goal.items()} if op=='plan' else {}
            if op in ('plan','execute'):parent.clear()
            parent.mission_pub.publish(String(data=json.dumps(dict(session_id=parent.runtime['session_id'],
                operator_id=parent.operator,op=op,**extra),allow_nan=False)))
            self.feedback.set('Requested '+op+'; waiting for actual mission status')
        except (ValueError,tk.TclError) as exc:self.feedback.set(str(exc))
