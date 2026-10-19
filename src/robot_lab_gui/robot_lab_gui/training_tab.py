"""Training workspace: train, watch, export, evaluate and use legged policies.

The tab starts ``legged-train`` (robot_lab_adapter.legged_train) as its own
process group, follows the run's ``progress.jsonl`` and plots it, lists the
runs under ``$ROBOT_LAB_RUNTIME_ROOT/training`` and hands an exported
policy directory to the Launch page (``legged_policy:=<directory>``).
Stopping writes a ``STOP`` file and signals the trainer, which saves a
checkpoint and exports the policy before it exits.  A trainer keeps running
when the window closes; reopening the tab finds it through ``run.json``.
"""
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .launcher import subprocess_env

# Mirrors robot_lab_adapter.legged_training (TRAINABLE, TASKS); the tab reads
# the module when it is importable and falls back to this copy.
TRAINABLE_ROBOTS = ('unitree_go2', 'unitree_a1', 'unitree_b2')
TASKS = {
    'walk': 'Track commanded forward, lateral and yaw velocity.',
    'turn_in_place': 'Turn on the spot at the commanded yaw rate.',
    'stand': 'Stand still and upright in the default pose.',
    'push_recovery': 'Stand, and recover from random pushes.',
}


def training_root():
    return Path(subprocess_env().get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'training'


def read_progress(run_dir, limit=None):
    """Iteration lines (and the finish line) of a run's progress.jsonl."""
    path = Path(run_dir) / 'progress.jsonl'
    lines = []
    if not path.exists():
        return lines
    for raw in path.read_text().splitlines():
        try:
            lines.append(json.loads(raw))
        except ValueError:
            continue
    return lines[-limit:] if limit else lines


def run_summary(run_dir):
    """One row for the runs list."""
    run_dir = Path(run_dir)
    try:
        run = json.loads((run_dir / 'run.json').read_text())
    except (OSError, ValueError):
        return None
    lines = read_progress(run_dir)
    iterations = [line for line in lines if 'iteration' in line and 'event' not in line]
    last = iterations[-1] if iterations else {}
    finished = any(line.get('event') == 'finished' for line in lines)
    return dict(name=run_dir.name, robot=run.get('robot', ''), task=run.get('task', ''),
                iteration=last.get('iteration', 0), planned=run.get('iterations', 0),
                mean_return=last.get('mean_return'), finished=finished,
                policy=(run_dir / 'policy' / 'policy.onnx').exists(), path=str(run_dir))


def evaluation_summary(path):
    """One line per evaluated command from ``legged-train evaluate --out``."""
    report = json.loads(Path(path).read_text())
    parts = []
    for phase in report['phases']:
        command = ', '.join('%g' % v for v in phase['command'])
        measured = ', '.join('%.2f' % v for v in phase['measured'])
        parts.append('(%s) → (%s)%s' % (command, measured, ' FELL' if phase['fallen'] else ''))
    return 'Evaluation, commanded → measured vx, vy, wz: ' + '; '.join(parts)


def trainer_pid(run_dir):
    """PID of the live trainer writing ``run_dir`` (0 when none)."""
    try:
        pid = int(json.loads((Path(run_dir) / 'run.json').read_text()).get('pid', 0))
        cmdline = Path('/proc/%d/cmdline' % pid).read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except (OSError, ValueError, TypeError):
        return 0
    trainer = 'legged_train' in cmdline or 'legged-train' in cmdline
    return pid if trainer and str(Path(run_dir)) in cmdline else 0


def trainer_command(robot, task, envs, iterations, steps, learning_rate, seed, out, resume=''):
    command = [sys.executable, '-m', 'robot_lab_adapter.legged_train', 'train', '--robot', robot,
               '--task', task, '--envs', str(int(envs)), '--iterations', str(int(iterations)),
               '--steps', str(int(steps)), '--learning-rate', repr(float(learning_rate)),
               '--seed', str(int(seed)), '--out', str(out)]
    if resume:
        command += ['--resume', str(resume)]
    return command


class TrainingTab(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        self.app = app
        notebook.add(self, text='Training')
        self.process = None
        self.run_dir = None
        self.tool_results = []      # (exit code, message function) from worker threads
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        self._build()
        self.refresh_runs()
        for run in self.tree.get_children():
            if trainer_pid(run):
                self.run_dir = Path(run)
                self.tree.selection_set(run)
                break
        self.job = self.after(1000, self._poll)

    # --- layout ---------------------------------------------------------------
    def _build(self):
        robots, tasks = self._choices()
        setup = ttk.LabelFrame(self, text='Train a legged policy (MuJoCo, PPO)', padding=10)
        setup.grid(row=0, column=0, sticky='ew')
        for column in (1, 3):
            setup.columnconfigure(column, weight=1)
        self.robot_var = tk.StringVar(value='unitree_go2' if 'unitree_go2' in robots else (robots[0] if robots else ''))
        self.task_var = tk.StringVar(value='walk')
        self.envs_var = tk.IntVar(value=512)
        self.iterations_var = tk.IntVar(value=1500)
        self.steps_var = tk.IntVar(value=24)
        self.lr_var = tk.DoubleVar(value=1e-3)
        self.seed_var = tk.IntVar(value=1)
        self.resume_var = tk.StringVar(value='')
        self.out_var = tk.StringVar(value='')
        ttk.Label(setup, text='Robot').grid(row=0, column=0, sticky='w')
        ttk.Combobox(setup, textvariable=self.robot_var, values=robots, state='readonly',
                     width=22).grid(row=0, column=1, sticky='ew', pady=2)
        ttk.Label(setup, text='Task').grid(row=0, column=2, sticky='w', padx=(12, 4))
        task_box = ttk.Combobox(setup, textvariable=self.task_var, values=list(tasks), state='readonly', width=22)
        task_box.grid(row=0, column=3, sticky='ew', pady=2)
        self.task_note = tk.StringVar(value=tasks.get('walk', ''))
        task_box.bind('<<ComboboxSelected>>', lambda _e: self.task_note.set(tasks.get(self.task_var.get(), '')))
        ttk.Label(setup, textvariable=self.task_note, style='Muted.TLabel').grid(
            row=1, column=0, columnspan=4, sticky='w', pady=(0, 4))
        fields = (('Parallel environments', self.envs_var, 16, 4096, 16),
                  ('Iterations', self.iterations_var, 1, 100000, 50),
                  ('Steps per iteration', self.steps_var, 8, 200, 4),
                  ('Seed', self.seed_var, 0, 99999, 1))
        for index, (label, variable, low, high, step) in enumerate(fields):
            row, column = 2 + index // 2, (index % 2) * 2
            ttk.Label(setup, text=label).grid(row=row, column=column, sticky='w', padx=(12 if column else 0, 4))
            ttk.Spinbox(setup, from_=low, to=high, increment=step, textvariable=variable,
                        width=10).grid(row=row, column=column + 1, sticky='w', pady=2)
        ttk.Label(setup, text='Learning rate').grid(row=4, column=0, sticky='w')
        ttk.Entry(setup, textvariable=self.lr_var, width=10).grid(row=4, column=1, sticky='w', pady=2)
        ttk.Label(setup, text='Resume from').grid(row=5, column=0, sticky='w')
        resume = ttk.Frame(setup)
        resume.grid(row=5, column=1, columnspan=3, sticky='ew', pady=2)
        resume.columnconfigure(0, weight=1)
        ttk.Entry(resume, textvariable=self.resume_var).grid(row=0, column=0, sticky='ew')
        ttk.Button(resume, text='Browse', command=self._browse_resume).grid(row=0, column=1, padx=(4, 0))
        ttk.Label(setup, text='Run folder').grid(row=6, column=0, sticky='w')
        ttk.Entry(setup, textvariable=self.out_var).grid(row=6, column=1, columnspan=3, sticky='ew', pady=2)
        ttk.Label(setup, text='Empty: a new folder under ' + str(training_root()) + '. Training runs on the '
                  'CPU (MuJoCo rollout threads); a walk needs hours, stand minutes.',
                  style='Muted.TLabel', wraplength=640, justify='left').grid(
            row=7, column=0, columnspan=4, sticky='w', pady=(2, 0))
        buttons = ttk.Frame(self)
        buttons.grid(row=1, column=0, sticky='ew', pady=8)
        self.start_button = ttk.Button(buttons, text='Start training', command=self.start)
        self.stop_button = ttk.Button(buttons, text='Stop and export', command=self.stop)
        self.export_button = ttk.Button(buttons, text='Export policy', command=self.export)
        self.evaluate_button = ttk.Button(buttons, text='Evaluate policy', command=self.evaluate)
        self.use_button = ttk.Button(buttons, text='Use policy in Launch', command=self.use_in_launch)
        for column, button in enumerate((self.start_button, self.stop_button, self.export_button,
                                         self.evaluate_button, self.use_button)):
            buttons.columnconfigure(column, weight=1)
            button.grid(row=0, column=column, sticky='ew', padx=2)
        # Export/evaluate results (the progress line is redrawn every poll).
        self.note = tk.StringVar(value='')
        ttk.Label(buttons, textvariable=self.note, style='Muted.TLabel', wraplength=900,
                  justify='left').grid(row=1, column=0, columnspan=5, sticky='w', pady=(4, 0))
        body = ttk.Panedwindow(self, orient='horizontal')
        body.grid(row=2, column=0, sticky='nsew')
        progress = ttk.LabelFrame(body, text='Progress', padding=6)
        progress.columnconfigure(0, weight=1)
        progress.rowconfigure(1, weight=1)
        self.status = tk.StringVar(value='No training run selected.')
        ttk.Label(progress, textvariable=self.status, wraplength=560, justify='left').grid(row=0, column=0, sticky='ew')
        self.plot = tk.Canvas(progress, height=220, background='#151a22', highlightthickness=0)
        self.plot.grid(row=1, column=0, sticky='nsew', pady=4)
        self.terms = tk.StringVar(value='')
        ttk.Label(progress, textvariable=self.terms, style='Muted.TLabel', wraplength=560,
                  justify='left').grid(row=2, column=0, sticky='ew')
        runs = ttk.LabelFrame(body, text='Runs', padding=6)
        runs.columnconfigure(0, weight=1)
        runs.rowconfigure(0, weight=1)
        columns = ('robot', 'task', 'iteration', 'return', 'policy')
        self.tree = ttk.Treeview(runs, columns=columns, show='tree headings', height=10)
        self.tree.heading('#0', text='Run')
        for column, title, width in zip(columns, ('Robot', 'Task', 'Iteration', 'Return', 'Policy'),
                                        (110, 90, 80, 70, 60)):
            self.tree.heading(column, text=title)
            self.tree.column(column, width=width, anchor='w')
        self.tree.column('#0', width=200)
        self.tree.grid(row=0, column=0, sticky='nsew')
        self.tree.bind('<<TreeviewSelect>>', self._select_run)
        ttk.Button(runs, text='Refresh', command=self.refresh_runs).grid(row=1, column=0, sticky='e', pady=(4, 0))
        body.add(progress, weight=3)
        body.add(runs, weight=2)
        self._update_buttons()

    def _choices(self):
        try:
            from robot_lab_adapter.legged_training import TASKS as tasks, TRAINABLE
            return sorted(TRAINABLE), {name: task.description for name, task in tasks.items()}
        except Exception:  # noqa: BLE001 - the GUI still lists the known robots
            return list(TRAINABLE_ROBOTS), dict(TASKS)

    # --- actions ----------------------------------------------------------------
    def running(self):
        if self.process is not None and self.process.poll() is None:
            return True
        return bool(self.run_dir is not None and trainer_pid(self.run_dir))

    def start(self):
        if self.running():
            messagebox.showinfo('Training running', 'Stop the current training run first.')
            return
        robot, task = self.robot_var.get(), self.task_var.get()
        if not robot or not task:
            return
        missing = [name for name in ('torch', 'mujoco') if importlib.util.find_spec(name) is None]
        if missing:
            messagebox.showerror('Training needs %s' % ' and '.join(missing),
                                 'Install it for this Python (for example: pip install %s).' % ' '.join(missing))
            return
        out = self.out_var.get().strip()
        if not out:
            out = str(training_root() / ('%s_%s_%s' % (robot, task, time.strftime('%Y%m%d_%H%M%S'))))
        out = Path(out)
        if (out / 'run.json').exists() and not self.resume_var.get().strip():
            messagebox.showinfo('Run exists', 'Choose a new run folder, or resume from a checkpoint.')
            return
        try:
            command = trainer_command(robot, task, self.envs_var.get(), self.iterations_var.get(),
                                      self.steps_var.get(), self.lr_var.get(), self.seed_var.get(), out,
                                      self.resume_var.get().strip())
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror('Training settings', str(exc))
            return
        out.mkdir(parents=True, exist_ok=True)
        (out / 'STOP').unlink(missing_ok=True)
        log = open(out / 'trainer.log', 'a')
        self.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=subprocess_env(),
                                        start_new_session=True)
        log.close()
        self.run_dir = out
        self.app.log('[training] %s\n' % ' '.join(command))
        self.app.set_status('Training %s / %s' % (robot, task))
        self._update_buttons()
        self.refresh_runs()

    def stop(self):
        if self.run_dir is not None:
            Path(self.run_dir, 'STOP').touch()
        if self.running():
            try:
                if self.process is not None and self.process.poll() is None:
                    os.killpg(self.process.pid, signal.SIGINT)
                else:
                    os.kill(trainer_pid(self.run_dir), signal.SIGINT)
            except OSError:
                pass
            self.status.set('Stopping: the trainer saves a checkpoint and exports the policy.')
        self._update_buttons()

    def export(self):
        run = self._selected_dir()
        checkpoint = run and Path(run, 'checkpoints', 'latest.pt')
        if not checkpoint or not checkpoint.exists():
            messagebox.showinfo('Export', 'Select a run with a checkpoint.')
            return
        self._run_tool(['export', '--checkpoint', str(checkpoint), '--out', str(Path(run, 'policy'))],
                       lambda: 'Exported %s' % Path(run, 'policy'))

    def evaluate(self):
        run = self._selected_dir()
        policy = run and Path(run, 'policy')
        if not policy or not (policy / 'policy.onnx').exists():
            messagebox.showinfo('Evaluate', 'Select a run with an exported policy.')
            return
        task = self._run_info(run).get('task', 'walk')
        commands = {'walk': '0.5,0,0;0,0.3,0;0,0,0.8;0,0,0', 'turn_in_place': '0,0,0.6;0,0,-0.6;0,0,1.0'}.get(
            task, '0,0,0')
        self.note.set('Evaluating %s ...' % policy)
        self._run_tool(['evaluate', '--policy-dir', str(policy), '--task', task, '--commands', commands,
                        '--out', str(Path(run, 'evaluation.json'))],
                       lambda: evaluation_summary(Path(run, 'evaluation.json')))

    def use_in_launch(self):
        run = self._selected_dir()
        policy = run and Path(run, 'policy')
        if not policy or not (policy / 'source.json').exists():
            messagebox.showinfo('Use policy', 'Select a run with an exported policy.')
            return
        robot = json.loads((policy / 'source.json').read_text()).get('profile', '')
        if robot and robot in getattr(self.app, 'robot_profiles', {robot: {}}):
            self.app.robot_var.set(robot)
        if self.app.mode_var.get() not in ('loc', 'slam', 'nav'):
            self.app.mode_var.set('loc')
        self.app.legged_policy_var.set(True)
        self.app.trained_policy_var.set(str(policy))
        self.app._update_from_selection()
        self.app.log('[training] Launch uses legged_policy:=%s for %s\n' % (policy, robot))
        self.app.set_status('Trained policy selected for ' + robot)
        self.app.show_tab('Launch')

    def _run_tool(self, arguments, done):
        """Run a legged-train subcommand off the Tk thread; _poll shows done() or the failure."""
        command = [sys.executable, '-m', 'robot_lab_adapter.legged_train', *arguments]
        self.app.log('[training] %s\n' % ' '.join(command))

        def worker():
            result = subprocess.run(command, capture_output=True, text=True, env=subprocess_env())
            text = (result.stdout or '') + (result.stderr or '')
            self.app.log(text + ('\n' if text and not text.endswith('\n') else ''))
            self.tool_results.append((result.returncode, done))
        threading.Thread(target=worker, daemon=True).start()

    def _browse_resume(self):
        path = filedialog.askopenfilename(initialdir=str(training_root()), filetypes=[('Checkpoint', '*.pt')])
        if path:
            self.resume_var.set(path)

    # --- runs and progress -----------------------------------------------------------
    def refresh_runs(self):
        root = training_root()
        selected = self._selected_dir()
        self.tree.delete(*self.tree.get_children())
        rows = []
        if root.exists():
            for path in root.iterdir():
                summary = run_summary(path) if path.is_dir() else None
                if summary:
                    rows.append(summary)
        for row in sorted(rows, key=lambda r: r['name'], reverse=True):
            value = '' if row['mean_return'] is None else '%.2f' % row['mean_return']
            iteration = '%s/%s' % (row['iteration'], row['planned'])
            self.tree.insert('', 'end', iid=row['path'], text=row['name'],
                             values=(row['robot'], row['task'], iteration, value, 'yes' if row['policy'] else ''))
        target = str(self.run_dir) if self.run_dir else selected
        if target and self.tree.exists(target):
            self.tree.selection_set(target)

    def _select_run(self, _event=None):
        run = self._selected_dir()
        if run and not (self.process is not None and self.process.poll() is None):
            self.run_dir = Path(run)
        self._draw()
        self._update_buttons()

    def _selected_dir(self):
        selection = self.tree.selection()
        if selection:
            return selection[0]
        return str(self.run_dir) if self.run_dir else None

    def _run_info(self, run):
        try:
            return json.loads(Path(run, 'run.json').read_text())
        except (OSError, ValueError):
            return {}

    def _poll(self):
        while self.tool_results:
            code, done = self.tool_results.pop(0)
            try:
                self.note.set(done() if code == 0 else 'Failed (exit %d): see the console.' % code)
            except (OSError, ValueError, KeyError) as exc:
                self.note.set('Finished, but the result could not be read: %s' % exc)
            self.refresh_runs()
        if self.process is not None and self.process.poll() is not None:
            code = self.process.returncode
            self.process = None
            self.app.set_status('Training finished' if code == 0 else 'Training stopped (exit %s)' % code)
            self.refresh_runs()
        self.polls = getattr(self, 'polls', 0) + 1
        if self.running() and self.polls % 5 == 0:
            self.refresh_runs()
        if self.run_dir is not None:
            self._draw()
        self._update_buttons()
        self.job = self.after(2000, self._poll)

    def _draw(self):
        run = self._selected_dir()
        if not run:
            return
        lines = [line for line in read_progress(run) if 'iteration' in line and 'event' not in line]
        info = self._run_info(run)
        canvas = self.plot
        canvas.delete('all')
        width, height = max(200, canvas.winfo_width()), max(120, canvas.winfo_height())
        if not lines:
            self.status.set('%s: waiting for the first iteration.' % Path(run).name)
            return
        last = lines[-1]
        planned = info.get('iterations', 0)
        done = last['iteration'] - (lines[0]['iteration'] - 1)
        rate = done / max(1e-6, last.get('wall_s', 0.0))
        remaining = max(0, planned - done) / rate if rate > 0 else math.nan
        self.status.set('%s · %s / %s · iteration %d of %s · %.2f M steps · %s steps/s · %.0f s elapsed%s' % (
            Path(run).name, info.get('robot', ''), info.get('task', ''), last['iteration'],
            planned or '?', last['environment_steps'] / 1e6, last.get('steps_per_s', '?'), last.get('wall_s', 0),
            '' if not self.running() or math.isnan(remaining) else ' · about %.0f min left' % (remaining / 60)))
        terms = last.get('terms', {})
        self.terms.set('Mean return %s · episode %s s · action std %.2f · KL %.4f · lr %.1e\n%s' % (
            '-' if last.get('mean_return') is None else '%.2f' % last['mean_return'],
            '-' if last.get('mean_episode_s') is None else '%.1f' % last['mean_episode_s'],
            last.get('action_std', 0.0), last.get('kl', 0.0), last.get('learning_rate', 0.0),
            '  '.join('%s %.4f' % (k, v) for k, v in sorted(terms.items()))))
        for key, colour in (('mean_return', '#5fb3ff'), ('mean_episode_s', '#8fd694')):
            points = [(line['iteration'], line[key]) for line in lines if line.get(key) is not None]
            if len(points) < 2:
                continue
            xs, ys = [p[0] for p in points], [p[1] for p in points]
            x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
            span_x, span_y = max(1e-9, x1 - x0), max(1e-9, y1 - y0)
            coords = []
            for x, y in points:
                coords += [8 + (width - 16) * (x - x0) / span_x, height - 18 - (height - 36) * (y - y0) / span_y]
            canvas.create_line(*coords, fill=colour, width=2)
            canvas.create_text(width - 10, height - 8, anchor='e', fill='#8a94a6',
                               text='iteration %d … %d' % (x0, x1))
            canvas.create_text(10, 10 if key == 'mean_return' else 24, anchor='w', fill=colour,
                               text='%s %.2f … %.2f' % ('return' if key == 'mean_return' else 'episode s', y0, y1))

    def _update_buttons(self):
        running = self.running()
        self.start_button.state(['disabled'] if running else ['!disabled'])
        self.stop_button.state(['!disabled'] if running else ['disabled'])
        run = self._selected_dir()
        has_checkpoint = bool(run and Path(run, 'checkpoints', 'latest.pt').exists())
        has_policy = bool(run and Path(run, 'policy', 'policy.onnx').exists())
        self.export_button.state(['!disabled'] if has_checkpoint and not running else ['disabled'])
        for button in (self.evaluate_button, self.use_button):
            button.state(['!disabled'] if has_policy else ['disabled'])

    def close(self):
        """The trainer outlives the window; Stop it from the tab."""
        self.after_cancel(self.job)
