"""Native Fleet subpage for actual facility joints and modeled energy."""
import json
import time
import tkinter as tk
from tkinter import ttk
from .workspace_ui import ScrollPanel


class InfrastructureControls(ScrollPanel):
    def __init__(self,notebook,fleet):
        super().__init__(notebook,width=380);notebook.add(self,text='Facility');self.fleet=fleet
        self.runtime=None;self.at=0.;self.publisher=None
        self.door=tk.StringVar(value='main');self.floor=tk.StringVar(value='L0');self.dock=tk.StringVar(value='west_charger')
        self.status=tk.StringVar(value='Choose fleet_facility, Run, then Connect. No infrastructure command is sent on connect.')
        self.feedback=tk.StringVar(value='Waiting for physical joint feedback; battery energy is unknown without verified motor power.')
        self.body.columnconfigure(0,weight=1)
        ttk.Label(self.body,textvariable=self.status,wraplength=420).grid(row=0,column=0,sticky='ew')
        self.doors=ttk.Combobox(self.body,textvariable=self.door,values=['main','lift'],state='readonly');self.doors.grid(row=1,column=0,sticky='ew',pady=4)
        controls=ttk.Frame(self.body);controls.grid(row=2,column=0,sticky='ew')
        for label,opened in [('Open door',True),('Close if clear',False)]:
            ttk.Button(controls,text=label,command=lambda o=opened:self.send('door',door=self.door.get(),open=o)).pack(side='left',expand=True,fill='x')
        self.floors=ttk.Combobox(self.body,textvariable=self.floor,values=['L0','L1'],state='readonly');self.floors.grid(row=3,column=0,sticky='ew',pady=4)
        lift_controls=ttk.Frame(self.body);lift_controls.grid(row=4,column=0,sticky='ew')
        ttk.Button(lift_controls,text='Request lift only',command=lambda:self.send('lift',floor=self.floor.get())).pack(side='left',expand=True,fill='x')
        ttk.Button(lift_controls,text='Transport selected robot',command=self.transport_member).pack(side='left',expand=True,fill='x')
        queue_controls=ttk.Frame(self.body);queue_controls.grid(row=5,column=0,sticky='ew',pady=4)
        ttk.Button(queue_controls,text='Queue selected members',command=self.queue_members).pack(side='left',expand=True,fill='x')
        ttk.Button(queue_controls,text='Queue round trip',command=lambda:self.queue_members(True)).pack(side='left',expand=True,fill='x')
        queue_actions=ttk.Frame(self.body);queue_actions.grid(row=6,column=0,sticky='ew')
        ttk.Button(queue_actions,text='Cancel transport queue',command=lambda:self.send('cancel_transports')).pack(side='left',expand=True,fill='x')
        self.recover_button=ttk.Button(queue_actions,text='Recover held lift to floor',command=lambda:self.send('recover_lift',floor=self.floor.get()),state='disabled')
        self.recover_button.pack(side='left',expand=True,fill='x')
        self.queue_text=tk.StringVar(value='No queued floor missions. Recovery requires an explicit request after Stop/loss.')
        self.auto_park=tk.BooleanVar(value=True)
        ttk.Checkbutton(self.body,text='Park after exit · keep landing clear',variable=self.auto_park).grid(row=12,column=0,sticky='w',pady=4)
        ttk.Label(self.body,textvariable=self.queue_text,wraplength=420,justify='left').grid(row=7,column=0,sticky='ew',pady=4)
        self.docks=ttk.Combobox(self.body,textvariable=self.dock,values=['west_charger'],state='readonly');self.docks.grid(row=8,column=0,sticky='ew',pady=4)
        ttk.Button(self.body,text='Navigate selected member to dock',command=self.dock_member).grid(row=9,column=0,sticky='ew')
        ttk.Label(self.body,textvariable=self.feedback,wraplength=420,justify='left').grid(row=10,column=0,sticky='ew',pady=5)
        ttk.Label(self.body,text='Door/lift states come from native joint positions. Close waits for fresh, clear fleet-body sweeps. '
            'Queued transport approaches, boards, rides, changes maps and exits one member at a time; Stop/loss cancels dependent stops. '
            'A parked peer may block a landing: move it clear before dispatch. Recovery closes clear gates and returns a held car to a floor; '
            'use Drive to exit afterward. Docking requires an independently observed settled body. Energy is an authored model.',wraplength=420).grid(row=11,column=0,sticky='ew')
        def reflow(event):
            for widget in self.body.winfo_children():
                if isinstance(widget,ttk.Label):widget.configure(wraplength=max(240,event.width-35))
            for frame in (controls,lift_controls,queue_controls,queue_actions):
                for button in frame.winfo_children():button.pack_configure(side='top' if event.width<430 else 'left')
        self.canvas.bind('<Configure>',reflow,add='+')

    def attach(self,node):
        from std_msgs.msg import String
        self.publisher=node.create_publisher(String,'/fleet/infrastructure/command',10)
        node.create_subscription(String,'/fleet/infrastructure/state',self.observe,10)

    def observe(self,msg):
        try:
            value=json.loads(msg.data)
            if not self.fleet.runtime or value['session_id']!=self.fleet.runtime['session_id']:return
            self.runtime=value;self.at=time.monotonic();self.doors.configure(values=list(value['doors']));self.floors.configure(values=value['lift']['floors']);self.docks.configure(values=list(value['docks']))
            self.recover_button.configure(state='normal' if value.get('ready') and value.get('operator_hold') else 'disabled')
            queue=value.get('transport_queue',{})
            rows=([queue['active']] if queue.get('active') else [])+queue.get('pending',[])
            history=queue.get('history',[])
            self.queue_text.set('\n'.join(f"{i+1}. {row['robot']} → {row['floor']}: {row['phase']} — {row['reason']}" for i,row in enumerate(rows[:8]))
                +(f'\n{len(rows)-8} more stops' if len(rows)>8 else '')
                if rows else ('Last transport: '+history[-1]['phase']+' — '+history[-1]['reason'] if history else 'No queued floor missions'))
            self.status.set('Facility request rejected: '+value['last_rejection'] if value.get('last_rejection') else 'Physical infrastructure ready' if value['ready'] else 'Infrastructure feedback stale; no arrival claim')
            lines=[f"Door {n}: {v['position_m']:.2f}/{v['command_m']:.2f} m, mode {v['mode']}"
                   for n,v in value['doors'].items()]
            lines.append(f"Lift: {value['lift']} (operator hold: {value.get('operator_hold', False)})")
            if value.get('rmf_protocol'):
                lines.append('SDK facility lease: '+('ready' if value.get('rmf_control_ready') else
                    'inactive; explicitly give RMF control for SDK tasks'))
            for name,energy in value['energy'].items():
                lines.append(name+' modeled battery: '+('unknown: '+energy.get('reason', 'motor feedback unqualified') if energy['fraction'] is None else f"{100*energy['fraction']:.1f}%"))
                if energy.get('power_W') is not None:
                    lines.append(f"  Observed load {energy['power_W']:.2f} W; authored charging {energy.get('charging_W', 0.):.1f} W")
            active_maps=value.get('active_robot_maps',{})
            floor_maps=value.get('robot_floor_maps',{})
            statuses=value.get('map_status',{})
            loading=set(value.get('map_loading',[]))
            for name,floor in value.get('robot_floors',{}).items():
                desired=floor_maps.get(name,'').rsplit('/',1)[-1] or 'unconfigured'
                active=active_maps.get(name,'').rsplit('/',1)[-1] or 'unconfigured'
                status='held/loading' if name in loading else statuses.get(name,'ready')
                lines.append(f'{name} floor {floor}: map {desired}; active {active} ({status})')
            lines.extend(name+' map error: '+error for name,error in value.get('map_errors',{}).items())
            lines += [name+' planar localization: '+('live scan' if enabled else 'paused for lift/map transition') for name,enabled in value.get('localization_scan_enabled',{}).items()]
            lines += [name+' lift transport: '+task['phase']+' — '+task['reason'] for name,task in value.get('transports',{}).items()]
            if value.get('lift_recovery'):
                recovery=value['lift_recovery']
                lines.append('Lift recovery: '+recovery['phase']+' — '+recovery['reason'])
            lines += [n+' dock: '+('physically settled' if v['docked'] else 'not settled') for n,v in value['docking'].items()]
            self.feedback.set('\n'.join(lines))
        except (ValueError,KeyError,TypeError):return

    def send(self,op,**extra):
        from std_msgs.msg import String
        if not self.publisher or not self.runtime or time.monotonic()-self.at>1.:
            self.status.set('Run a facility session and Connect to fresh feedback first');return
        state=self.fleet.runtime
        if (not state or state.get('operator_id')!=self.fleet.operator_id or not state.get('operator_fresh')
                or self.runtime.get('operator_id')!=self.fleet.operator_id or not self.runtime.get('operator_fresh')):
            self.status.set('Wait for the GUI operator lease; disable RMF ownership for manual facility requests');return
        self.publisher.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'],operator_id=self.fleet.operator_id,op=op,**extra))))

    def dock_member(self):
        ids=self.fleet._ids()
        if len(ids)!=1:self.status.set('Select one member to dock');return
        self.send('dock',robot=ids[0],dock=self.dock.get())

    def transport_member(self):
        ids=self.fleet._ids()
        if len(ids)!=1:self.status.set('Select one idle Nav2 member for transport');return
        self.send('transport',robot=ids[0],floor=self.floor.get())

    def queue_members(self,round_trip=False):
        ids=self.fleet._ids()
        if not ids:
            self.status.set('Select one or more idle Nav2 members for the itinerary');return
        floors=(self.runtime or {}).get('robot_floors',{})
        target=self.floor.get()
        requests=[]
        for name in ids:
            origin=floors.get(name)
            if origin is None or origin==target:
                self.status.set('Choose a different destination floor for every selected member');return
            requests.append(dict(robot=name,floor=target,**({'parking':'auto'} if self.auto_park.get() else {})))
        if round_trip:
            requests.extend(dict(robot=name,floor=floors[name],**({'parking':'auto'} if self.auto_park.get() else {})) for name in ids)
        self.send('transport_queue',requests=requests)
