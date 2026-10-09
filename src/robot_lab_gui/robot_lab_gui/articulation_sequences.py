"""Teach measured native articulation and replay explicit bounded waypoints."""
import json
import math
import os
from pathlib import Path
from tkinter import filedialog, ttk
import tkinter as tk


class ArticulationSequences(ttk.LabelFrame):
    def __init__(self, parent, controller):
        super().__init__(parent, text='Teach / replay articulation (experimental)', padding=6)
        self.control, self.buffers, self.source = controller, {}, None
        self.status = tk.StringVar(value='Jog to a configuration, then Teach current.')
        self.points = []
        self.listing = tk.Listbox(self, height=4, exportselection=False)
        self.listing.grid(row=0, column=0, columnspan=3, sticky='ew')
        self.buttons = []
        for row, column, label, callback in (
                (1, 0, 'Teach current', self.teach), (1, 1, 'Remove point', self.remove),
                (1, 2, 'Replay', self.replay), (2, 0, 'Save sequence', self.save),
                (2, 1, 'Load sequence', self.load), (2, 2, 'Stop replay', controller.stop)):
            button = ttk.Button(self, text=label, command=callback)
            button.grid(row=row, column=column, sticky='ew', padx=2, pady=3)
            self.buttons.append(button)
        ttk.Label(self, textvariable=self.status, wraplength=300).grid(row=3, column=0, columnspan=3, sticky='w')
        ttk.Label(self, text='Replay follows measured actuator configurations. '
            'It does not plan a collision-free path or assess grasp success. '
            'Each step must settle; Stop/input loss interrupts replay.',
            wraplength=300).grid(row=4, column=0, columnspan=3, sticky='w', pady=4)

    def sync_source(self):
        source = (self.control.state or {}).get('source_sha256')
        if source != self.source:
            self.source = source
            self.points = self.buffers.setdefault(source, []) if source else []
            self.refresh()

    def refresh(self):
        self.listing.delete(0, 'end')
        for index, point in enumerate(self.points):
            self.listing.insert('end', str(index+1)+'. '+point['name'])

    def idle(self):
        state = self.control.state or {}
        channels = state.get('channels', [])
        return bool(self.control.ready() and not state.get('mobile_state', {}).get('moving')
            and state.get('sequence', {}).get('state') != 'executing'
            and not (state.get('servo') or {}).get('active') and channels and all(
                abs(channel.get('measured_rate', math.inf)) < .05*channel['max_rate'] for channel in channels))

    def teach(self):
        self.sync_source()
        if not self.idle():
            self.status.set('Wait for measured articulation and base to stop.')
            return
        if len(self.points) >= 64:
            self.status.set('Maximum 64 taught configurations per sequence.')
            return
        self.points.append(dict(name='Configuration '+str(len(self.points)+1), settle_s=.5,
            targets={channel['name']: channel['measured'] for channel in self.control.state['channels']}))
        self.refresh()
        self.status.set('Captured measured configuration; teaching commands no movement.')

    def remove(self):
        if self.idle() and self.listing.curselection():
            del self.points[self.listing.curselection()[0]]
            self.refresh()

    def replay(self):
        self.sync_source()
        if not self.idle() or not self.points:
            return
        from std_msgs.msg import String
        self.control.pub.publish(String(data=json.dumps(dict(operation='sequence',
            plant_id=self.control.state['plant_id'], source_sha256=self.source, waypoints=self.points))))
        self.status.set('Requested replay; waiting for actual controller feedback.')

    def initial_directory(self):
        return str(Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')))

    def save(self):
        if not self.points or not self.source:
            return
        filename = filedialog.asksaveasfilename(parent=self, initialdir=self.initial_directory(),
            initialfile='articulation-sequence.json', defaultextension='.json', filetypes=[('JSON sequence', '*.json')])
        if filename:
            try:
                path = Path(filename)
                if path.parent.stat().st_dev == Path('/').stat().st_dev:
                    raise ValueError('Keep sequence output on the mounted workspace SSD.')
                payload = dict(schema='robot_lab.native_articulation_sequence.v1', source_sha256=self.source,
                    robot=self.control.app.robot_var.get(), waypoints=self.points)
                temporary = path.with_suffix(path.suffix+'.new')
                temporary.write_text(json.dumps(payload, indent=2)+'\n')
                temporary.replace(path)
                self.status.set('Saved taught configurations: '+str(path))
            except (OSError, ValueError) as exc:
                self.status.set(str(exc))

    def load(self):
        self.sync_source()
        if not self.idle():
            return
        filename = filedialog.askopenfilename(parent=self, initialdir=self.initial_directory(),
            filetypes=[('JSON sequence', '*.json')])
        if not filename:
            return
        try:
            path = Path(filename)
            if path.stat().st_size > 1024*1024:
                raise ValueError('Sequence file exceeds the 1 MB input bound.')
            payload = json.loads(path.read_text())
            if (not isinstance(payload, dict) or payload.get('schema') != 'robot_lab.native_articulation_sequence.v1'
                    or payload.get('source_sha256') != self.source):
                raise ValueError('Sequence must match the executed native model source.')
            points = payload['waypoints']
            if not isinstance(points, list) or not 1 <= len(points) <= 64:
                raise ValueError('Sequence must contain 1–64 waypoints.')
            channels = {channel['name']: channel for channel in self.control.state['channels']}
            for point in points:
                if not isinstance(point, dict) or not isinstance(point.get('targets'), dict):
                    raise ValueError('Waypoints must supply named actuator targets.')
                if set(point['targets']) != set(channels):
                    raise ValueError('Saved actuator channels differ from the running plant.')
                dwell = float(point.get('settle_s', .5))
                if not math.isfinite(dwell) or not .2 <= dwell <= 5.:
                    raise ValueError('Settle duration must be 0.2–5 simulation seconds.')
                for name, target in point['targets'].items():
                    value = float(target)
                    if not math.isfinite(value) or not channels[name]['limits'][0] <= value <= channels[name]['limits'][1]:
                        raise ValueError('Saved target exceeds source channel bounds.')
                point['name'] = str(point.get('name', 'Configuration'))[:80]
            self.points[:] = points
            self.refresh()
            self.status.set('Loaded configurations; no motion until Replay.')
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.status.set(str(exc))

    def poll(self):
        self.sync_source()
        for button in self.buttons[:5]:
            button.state(['!disabled'] if self.idle() else ['disabled'])
        self.buttons[-1].state(['!disabled'] if self.control.ready() else ['disabled'])
        sequence = (self.control.state or {}).get('sequence', {})
        if sequence.get('state') not in (None, 'idle'):
            self.status.set('Replay: '+sequence['state']+' · '+str(sequence.get('index', 0))+
                '/'+str(sequence.get('total', 0))+' · '+sequence.get('reason', ''))
