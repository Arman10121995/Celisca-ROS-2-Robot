"""Queue exact Launch selections for isolated, bounded subprocess execution."""
import json
import math
import queue
import shlex
import threading
import time
import tkinter as tk
from tkinter import ttk
from pathlib import Path


class ExperimentQueue(ttk.LabelFrame):
    def __init__(self, parent, app, output):
        super().__init__(parent, text='Concurrent experiments — real launches', padding=8)
        self.app, self.output = app, output
        self.specifications, self.runner = [], None
        self.events = queue.Queue()
        self.parallel = tk.IntVar(value=1)
        self.duration = tk.DoubleVar(value=30.)
        self.rss = tk.DoubleVar(value=4096.)
        self.cpu = tk.DoubleVar(value=800.)
        self.log_mb = tk.DoubleVar(value=256.)
        self.task = tk.StringVar()
        self.workload = tk.StringVar(value='Capture only')
        self.goals = tk.StringVar(value='')
        self.seed = tk.IntVar(value=42)
        self.status = tk.StringVar(value='Queue the current Launch command. Mission validation remains separate.')
        self.tree = ttk.Treeview(self, columns=('robot', 'map', 'backend', 'workload', 'state'), show='headings', height=4)
        for name in self.tree['columns']:
            self.tree.heading(name, text=name.title())
            self.tree.column(name, width=95)
        self.tree.grid(row=0, column=0, columnspan=6, sticky='ew')
        for column, (label, variable, start, end) in enumerate((
                ('Parallel', self.parallel, 1, 4), ('Capture s', self.duration, 5, 3600),
                ('RSS MB', self.rss, 256, 32768), ('CPU %', self.cpu, 100, 2400),
                ('Artifacts MB', self.log_mb, 16, 8192))):
            ttk.Label(self, text=label).grid(row=1, column=column, sticky='w', pady=(6, 0))
            ttk.Spinbox(self, textvariable=variable, from_=start, to=end, width=9).grid(row=2, column=column, sticky='w')
        ttk.Label(self, text='Requested seed').grid(row=1, column=5, sticky='w', pady=(6, 0))
        ttk.Spinbox(self, textvariable=self.seed, from_=0, to=2147483647, width=10).grid(row=2, column=5, sticky='w')
        self.workload_combo = ttk.Combobox(self, textvariable=self.workload, state='readonly',
            values=('Capture only', 'Nav2 point goals', 'External task'), width=20)
        self.workload_combo.grid(row=3, column=0, columnspan=2, sticky='w', pady=6)
        self.workload_combo.bind('<<ComboboxSelected>>', self.workload_changed)
        ttk.Entry(self, textvariable=self.goals).grid(row=3, column=2, columnspan=4, sticky='ew')
        ttk.Label(self, text='Nav2 goals: X Y heading_deg; X Y heading_deg. Independent truth must use selected-world coordinates. '
            'Seeds are requested; simulator application and contacts remain unassessed.', wraplength=800).grid(
            row=4, column=0, columnspan=6, sticky='w')
        ttk.Label(self, text='External task command (literal arguments; runs after clock/FCU readiness)').grid(
            row=5, column=0, columnspan=6, sticky='w', pady=(6, 0))
        ttk.Entry(self, textvariable=self.task).grid(row=6, column=0, columnspan=6, sticky='ew')
        self.add_button = ttk.Button(self, text='Queue Launch selection', command=self.add)
        self.run_button = ttk.Button(self, text='Run queue', command=self.run)
        self.clear_button = ttk.Button(self, text='Clear queue', command=self.clear)
        self.cancel_button = ttk.Button(self, text='Cancel owned runs', command=self.cancel)
        self.reset_button = ttk.Button(self, text='Reset selected run', command=self.reset)
        for column, button in enumerate((self.add_button, self.run_button, self.clear_button, self.cancel_button, self.reset_button)):
            button.grid(row=7, column=column, sticky='ew', padx=2, pady=6)
        ttk.Label(self, textvariable=self.status, wraplength=800).grid(row=8, column=0, columnspan=6, sticky='w')
        self.after(200, self.poll)
        self.bind('<Destroy>', lambda event: self.cancel() if event.widget is self else None, add='+')

    def workload_changed(self, _event=None):
        if self.workload.get() == 'Nav2 point goals':
            try:
                self.duration.set(max(600., float(self.duration.get())))
            except (ValueError, tk.TclError):
                self.duration.set(600.)
            self.status.set('Choose Navigation in Launch and enter selected-map goals. Capture budget raised to 600 s for stack startup and motion.')

    def goal_workload(self):
        from robot_lab_benchmark.workload_contract import WORKLOAD_SCHEMA, validate_workload
        goals = []
        for group in self.goals.get().split(';'):
            if not group.strip():
                continue
            values = [float(value) for value in group.replace(',', ' ').split()]
            if len(values) != 3:
                raise ValueError('Each Nav2 goal needs X Y heading_deg; separate multiple goals with semicolons.')
            values[2] = math.radians(values[2])
            goals.append(values)
        return validate_workload(dict(schema=WORKLOAD_SCHEMA, goals=goals, truth_coordinates='selected_world'))

    def add(self):
        try:
            from robot_lab_benchmark.concurrent_runner import RunSpec
            command = shlex.split(self.app.command_var.get())
            if not command or command[:2] != ['ros2', 'launch']:
                raise ValueError('Select a simulation in Launch and generate its run command first.')
            # Each queued plant owns its graph. Interactive GUI/RViz remain
            # with the main Launch workflow rather than sharing that graph.
            for flag in ('gui', 'start_rviz', 'enable_joystick'):
                command = [arg for arg in command if not arg.startswith(flag+':=')]
                command.append(flag+':=false')
            selection = dict(robot=self.app.robot_var.get(), map=self.app.map_var.get(),
                backend=self.app.simulator_var.get(), mode=self.app.mode_var.get(), workload=self.workload.get())
            workload = self.goal_workload() if self.workload.get() == 'Nav2 point goals' else {}
            specification = RunSpec(command=command, duration_s=float(self.duration.get()),
                max_rss_mb=float(self.rss.get()), max_cpu_percent=float(self.cpu.get()),
                max_log_mb=float(self.log_mb.get()), seed=int(self.seed.get()), workload=workload,
                task_command=shlex.split(self.task.get()) if self.workload.get() == 'External task' else [], selection=selection,
                readiness_topic='/px4/odometry' if selection['robot'] == 'px4_x500' else '/clock')
            specification.validate()
            self.specifications.append(specification)
            self.tree.insert('', 'end', values=(selection['robot'], selection['map'], selection['backend'], selection['workload'], 'queued'))
            self.status.set('Queued exact command: '+shlex.join(command))
        except Exception as exc:
            self.status.set(str(exc))

    def clear(self):
        if self.runner is None:
            self.specifications.clear()
            for item in self.tree.get_children():
                self.tree.delete(item)

    def run(self):
        if self.runner is not None or not self.specifications:
            return
        try:
            from robot_lab_benchmark.concurrent_runner import ConcurrentRunner
            directory = Path(self.output.get())/('concurrent-'+str(time.time_ns()))
            self.runner = ConcurrentRunner(directory, int(self.parallel.get()), self.events.put)
            specifications = list(self.specifications)
            self.tree.delete(*self.tree.get_children())
            self.status.set('Launching '+str(len(specifications))+' jobs; artifacts: '+str(directory))
            for button in (self.add_button, self.run_button, self.clear_button):
                button.state(['disabled'])
            threading.Thread(target=self.worker, args=(self.runner, specifications), daemon=False).start()
        except Exception as exc:
            self.runner = None
            self.status.set(str(exc))

    def worker(self, runner, specifications):
        try:
            results = runner.run(specifications)
            self.events.put(dict(queue_finished=True, results=results))
        except Exception as exc:
            self.events.put(dict(queue_finished=True, error=str(exc)))

    def cancel(self):
        if self.runner:
            self.runner.cancel()
            self.status.set('Cancel requested; the runner cleans up only its owned processes.')

    def reset(self):
        items = self.tree.selection()
        if self.runner and items:
            runner, identifier = self.runner, items[0]
            def work():
                try:
                    self.events.put(dict(reset=runner.reset(identifier)))
                except Exception as exc:
                    self.events.put(dict(reset=dict(error=str(exc))))
            threading.Thread(target=work, daemon=True).start()

    def poll(self):
        try:
            while True:
                result = self.events.get_nowait()
                if result.get('queue_finished'):
                    self.runner = None
                    self.specifications.clear()
                    for button in (self.add_button, self.run_button, self.clear_button):
                        button.state(['!disabled'])
                    results = result.get('results', [])
                    measured = sum(bool((item.get('mission_result') or {}).get('success')) for item in results)
                    failed = sum(item.get('mission_result') is not None and not item['mission_result'].get('success') for item in results)
                    self.status.set(result.get('error', 'Queue finished: '+str(measured)+' measured goal results, '+
                        str(failed)+' failed workloads, '+str(len(results)-measured-failed)+' unassessed. '
                        'Inspect raw artifacts; contacts and full mission qualification remain separate.'))
                elif 'reset' in result:
                    self.status.set('Reset response: '+json.dumps(result['reset']))
                else:
                    selection = result.get('selection', {})
                    state = result['state']
                    if result.get('mission_result') is not None:
                        state += ' · '+result['mission_result']['state']
                    values = (selection.get('robot'), selection.get('map'), selection.get('backend'), selection.get('workload'), state)
                    if self.tree.exists(result['id']):
                        self.tree.item(result['id'], values=values)
                    else:
                        self.tree.insert('', 'end', iid=result['id'], values=values)
                    self.app.log('[concurrent] '+json.dumps(result, default=str)+'\n')
        except queue.Empty:
            pass
        if self.winfo_exists():
            self.after(200, self.poll)
