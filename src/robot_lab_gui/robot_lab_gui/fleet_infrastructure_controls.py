"""Native Fleet subpage for actual facility joints and modeled energy."""
import json
import time
import tkinter as tk
from tkinter import ttk


class InfrastructureControls(ttk.Frame):
    def __init__(self,notebook,fleet):
        super().__init__(notebook,padding=5);notebook.add(self,text='Facility');self.fleet=fleet
        self.runtime=None;self.at=0.;self.publisher=None
        self.door=tk.StringVar(value='main');self.floor=tk.StringVar(value='L0');self.dock=tk.StringVar(value='west_charger')
        self.status=tk.StringVar(value='Choose fleet_facility, Run, then Connect. No infrastructure command is sent on connect.')
        self.feedback=tk.StringVar(value='Waiting for physical joint feedback; battery energy is unknown without verified motor power.')
        self.columnconfigure(0,weight=1)
        ttk.Label(self,textvariable=self.status,wraplength=420).grid(row=0,column=0,sticky='ew')
        self.doors=ttk.Combobox(self,textvariable=self.door,values=['main','lift'],state='readonly');self.doors.grid(row=1,column=0,sticky='ew',pady=4)
        controls=ttk.Frame(self);controls.grid(row=2,column=0,sticky='ew')
        for label,opened in [('Open door',True),('Close if clear',False)]:
            ttk.Button(controls,text=label,command=lambda o=opened:self.send('door',door=self.door.get(),open=o)).pack(side='left',expand=True,fill='x')
        self.floors=ttk.Combobox(self,textvariable=self.floor,values=['L0','L1'],state='readonly');self.floors.grid(row=3,column=0,sticky='ew',pady=4)
        ttk.Button(self,text='Request lift floor',command=lambda:self.send('lift',floor=self.floor.get())).grid(row=4,column=0,sticky='ew')
        self.docks=ttk.Combobox(self,textvariable=self.dock,values=['west_charger'],state='readonly');self.docks.grid(row=5,column=0,sticky='ew',pady=4)
        ttk.Button(self,text='Navigate selected member to dock',command=self.dock_member).grid(row=6,column=0,sticky='ew')
        ttk.Label(self,textvariable=self.feedback,wraplength=420,justify='left').grid(row=7,column=0,sticky='ew',pady=5)
        ttk.Label(self,text='Door/lift states come from native joint positions. Close waits for fresh, clear fleet-body sweeps. '
            'Lift floor motion closes the car door first; transport and multi-floor navigation remain separate. '
            'Docking requires an independently observed settled body. Energy is an authored model, never a fabricated hardware battery reading.',wraplength=420).grid(row=8,column=0,sticky='ew')

    def attach(self,node):
        from std_msgs.msg import String
        self.publisher=node.create_publisher(String,'/fleet/infrastructure/command',10)
        node.create_subscription(String,'/fleet/infrastructure/state',self.observe,10)

    def observe(self,msg):
        try:
            value=json.loads(msg.data)
            if not self.fleet.runtime or value['session_id']!=self.fleet.runtime['session_id']:return
            self.runtime=value;self.at=time.monotonic();self.doors.configure(values=list(value['doors']));self.floors.configure(values=value['lift']['floors']);self.docks.configure(values=list(value['docks']))
            self.status.set('Physical infrastructure ready' if value['ready'] else 'Infrastructure feedback stale; no arrival claim')
            lines=[f"Door {n}: {v['position_m']:.2f} m, mode {v['mode']}" for n,v in value['doors'].items()]
            lines.append('Lift: '+str(value['lift']))
            for name,energy in value['energy'].items():
                lines.append(name+' modeled battery: '+('unknown' if energy['fraction'] is None else f"{100*energy['fraction']:.1f}%"))
            lines += [n+' dock: '+('physically settled' if v['docked'] else 'not settled') for n,v in value['docking'].items()]
            self.feedback.set('\n'.join(lines))
        except (ValueError,KeyError,TypeError):return

    def send(self,op,**extra):
        from std_msgs.msg import String
        if not self.publisher or not self.runtime or time.monotonic()-self.at>1.:
            self.status.set('Run a facility session and Connect to fresh feedback first');return
        state=self.fleet.runtime
        if not state or state.get('operator_id')!=self.fleet.operator_id or not state.get('operator_fresh'):
            self.status.set('Wait for the GUI operator lease; disable RMF ownership for manual facility requests');return
        self.publisher.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'],operator_id=self.fleet.operator_id,op=op,**extra))))

    def dock_member(self):
        ids=self.fleet._ids()
        if len(ids)!=1:self.status.set('Select one member to dock');return
        self.send('dock',robot=ids[0],dock=self.dock.get())
