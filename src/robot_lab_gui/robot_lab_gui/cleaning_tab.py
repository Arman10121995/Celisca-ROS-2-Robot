"""Vacuum workspace: the robot-vacuum cleaning app.

A live map (rooms, cleaned area, the robot, the plan) with editing tools for
room selection, zones, no-go areas, virtual walls, spot and dock, and pages
for cleaning, rooms, schedule, history, maintenance and setup.  It talks to
the robot_lab_vacuum_cleaning node only through its topics:
/vacuum/command (JSON) in; /vacuum/status, /vacuum/layout, /vacuum/history,
/vacuum/plan, /vacuum/events, /vacuum/rooms, /vacuum/coverage and /map out.
"""

import json
import math
import time
import tkinter as tk
from tkinter import scrolledtext, simpledialog, ttk

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy ships with ROS
    np = None

from .lab_tabs import LabTab

MODES = [('all', 'Whole home'), ('rooms', 'Selected rooms'), ('zones', 'Zones'),
         ('spot', 'Spot'), ('edges', 'Edges only'), ('explore', 'Mapping run')]
TOOLS = [('select', 'Rooms', 'Click rooms to select them (click order = cleaning order)'),
         ('zone', 'Zone', 'Drag a rectangle to clean'),
         ('nogo', 'No-go', 'Drag a rectangle the robot must stay out of'),
         ('wall', 'Wall', 'Drag a virtual wall the robot will not cross'),
         ('carpet', 'Carpet', 'Drag a rectangle over a rug or carpet (Max suction there; a mopping job keeps off it)'),
         ('spot', 'Spot', 'Click a spot to clean intensively'),
         ('dock', 'Dock', 'Click the dock position, drag towards the way the robot faces when docked'),
         ('split', 'Split', 'Drag a line across a room to divide it'),
         ('erase', 'Erase', 'Click a zone, no-go area, carpet or wall to remove it')]
STATE_TEXT = {
    'idle': ('Idle', 'PillIdle.TLabel'), 'docked': ('Docked', 'PillIdle.TLabel'),
    'charging': ('Charging', 'PillRunning.TLabel'), 'cleaning': ('Cleaning', 'PillRunning.TLabel'),
    'mapping': ('Mapping', 'PillRunning.TLabel'), 'paused': ('Paused', 'PillWarn.TLabel'),
    'returning': ('Returning to dock', 'PillRunning.TLabel'), 'docking': ('Docking', 'PillRunning.TLabel'),
    'emptying': ('Emptying dustbin', 'PillRunning.TLabel'), 'error': ('Error', 'PillWarn.TLabel'),
    'refilling': ('Refilling water tank', 'PillRunning.TLabel'),
}
CLEAN_MODES = [('vacuum', 'Vacuum'), ('vacuum_mop', 'Vacuum + mop'), ('mop', 'Mop only')]
WATER_FLOWS = ['low', 'medium', 'high']
CARPET_COLOUR = '#c8a46e'
DAYS = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
ROOM_COLOURS = [(110, 170, 230), (240, 170, 110), (140, 210, 150), (210, 140, 210), (230, 210, 120),
                (120, 210, 210), (230, 130, 140), (170, 160, 240), (190, 220, 120), (240, 190, 200)]
SUPPORTED_MAPS = ('small_house', 'celisca_floor_1', 'celisca_floor_1_furniture', 'residential_demo',
                  'simple_office', 'nav_maze', 'nav_obstacle', 'nav_empty')


def fmt_duration(seconds):
    seconds = int(max(0, seconds or 0))
    if seconds >= 3600:
        return '%d h %02d min' % (seconds // 3600, seconds % 3600 // 60)
    return '%d min %02d s' % (seconds // 60, seconds % 60)


class VacuumTab(LabTab):
    """Robot-vacuum cleaning app for robots with supports_room_vacuum."""

    def __init__(self, notebook, app):
        super().__init__(notebook, app, "Vacuum")
        self.node = None
        self.executor = None
        self.command_pub = None
        self.ros_error = ''
        self.status = {}
        self.layout = {}
        self.history = []
        self.plan = {}
        self.map_info = None       # (width, height, resolution, ox, oy)
        self.map_data = None       # int16 [h, w]
        self.rooms_grid = None
        self.coverage_grid = None
        self.history_overlay = None
        self.dirty_image = False
        self.last_image = 0.0
        self.last_status_time = 0.0
        self.selected_rooms = []
        self.spot_point = None
        self.view = None           # (x0, y0, x1, y1) world window
        self.view_user = False
        self.drag = None
        self.photo = None
        self.locate_until = 0.0
        self.job = None
        self.closed = False
        self._build_ui()
        self.bind('<Map>', lambda _e: self._ensure_ros(), add='+')
        self.job = self.after(150, self._poll)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def _build_ui(self):
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=2, minsize=380)
        self.rowconfigure(1, weight=1)

        bar = ttk.Frame(self, style='Surface.TFrame', padding=(14, 10))
        bar.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 8))
        self.state_var = tk.StringVar(value='○  Not connected')
        self.state_label = ttk.Label(bar, textvariable=self.state_var, style='PillIdle.TLabel')
        self.state_label.grid(row=0, column=0, rowspan=2, sticky='w', padx=(0, 16))
        self.metric_vars = {}
        metrics = [('battery', 'Battery'), ('area', 'Cleaned'), ('coverage', 'Coverage'),
                   ('time', 'Time'), ('room', 'Room'), ('bin', 'Dustbin')]
        for col, (key, label) in enumerate(metrics, start=1):
            ttk.Label(bar, text=label, style='SurfaceField.TLabel').grid(row=0, column=col, sticky='w', padx=10)
            var = tk.StringVar(value='–')
            self.metric_vars[key] = var
            ttk.Label(bar, textvariable=var, style='SurfaceBody.TLabel').grid(row=1, column=col, sticky='w', padx=10)
            bar.columnconfigure(col, weight=1)
        self.battery_bar = ttk.Progressbar(bar, length=90, maximum=100)
        self.battery_bar.grid(row=2, column=1, sticky='w', padx=10, pady=(2, 0))
        self.message_var = tk.StringVar(value='Start the simulation and the cleaning system in Setup.')
        ttk.Label(bar, textvariable=self.message_var, style='SurfaceBody.TLabel', wraplength=900,
                  justify='left').grid(row=3, column=0, columnspan=8, sticky='w', pady=(6, 0))

        # Map
        mapf = ttk.Frame(self)
        mapf.grid(row=1, column=0, sticky='nsew', padx=(0, 8))
        mapf.columnconfigure(0, weight=1)
        mapf.rowconfigure(1, weight=1)
        tools = ttk.Frame(mapf)
        tools.grid(row=0, column=0, sticky='ew', pady=(0, 6))
        self.tool_var = tk.StringVar(value='select')
        self.tool_buttons = {}
        for i, (key, label, tip) in enumerate(TOOLS):
            b = ttk.Radiobutton(tools, text=label, value=key, variable=self.tool_var,
                                command=self._tool_changed, style='Toolbutton')
            b.grid(row=0, column=i, padx=(0, 2))
            self.tool_buttons[key] = b
        ttk.Button(tools, text='Fit', width=4, command=self._fit_view).grid(row=0, column=len(TOOLS), padx=(10, 0))
        self.tool_hint = tk.StringVar(value=TOOLS[0][2])
        ttk.Label(mapf, textvariable=self.tool_hint, style='Muted.TLabel').grid(row=2, column=0, sticky='w', pady=(4, 0))
        self.canvas = tk.Canvas(mapf, background='#0d1520', highlightthickness=0, width=640, height=460,
                                cursor='crosshair')
        self.canvas.grid(row=1, column=0, sticky='nsew')
        self.canvas.bind('<Configure>', lambda _e: self._mark_dirty())
        self.canvas.bind('<ButtonPress-1>', self._press)
        self.canvas.bind('<B1-Motion>', self._motion)
        self.canvas.bind('<ButtonRelease-1>', self._release)
        self.canvas.bind('<MouseWheel>', self._wheel)
        self.canvas.bind('<Button-4>', lambda e: self._zoom(e, 1.2))
        self.canvas.bind('<Button-5>', lambda e: self._zoom(e, 1 / 1.2))
        self.canvas.bind('<ButtonPress-3>', self._pan_start)
        self.canvas.bind('<B3-Motion>', self._pan_move)
        legend = ttk.Frame(mapf)
        legend.grid(row=3, column=0, sticky='w', pady=(4, 0))
        for i, (colour, text) in enumerate([('#4fbf76', 'cleaned'), ('#ff9f43', 'path'),
                                            ('#5aa9ff', 'zone'), ('#ff5c7a', 'no-go / wall'),
                                            (CARPET_COLOUR, 'carpet'), ('#ffe066', 'dock')]):
            sw = tk.Canvas(legend, width=12, height=12, highlightthickness=0, background=colour)
            sw.grid(row=0, column=2 * i, padx=(0 if i == 0 else 10, 4))
            ttk.Label(legend, text=text, style='Muted.TLabel').grid(row=0, column=2 * i + 1)

        # Pages
        pages = ttk.Notebook(self)
        pages.grid(row=1, column=1, sticky='nsew')
        self.pages = pages
        self._build_clean(self._page(pages, 'Clean'))
        self._build_rooms(self._page(pages, 'Rooms'))
        self._build_schedule(self._page(pages, 'Schedule'))
        self._build_history(self._page(pages, 'History'))
        self._build_care(self._page(pages, 'Care'))
        self._build_setup(self._page(pages, 'Setup'))

    @staticmethod
    def _page(notebook, title):
        frame = ttk.Frame(notebook, padding=10)
        frame.columnconfigure(0, weight=1)
        notebook.add(frame, text=title)
        return frame

    def _build_clean(self, page):
        ttk.Label(page, text='What to clean', style='Section.TLabel').grid(row=0, column=0, sticky='w')
        self.mode_var = tk.StringVar(value='all')
        modes = ttk.Frame(page)
        modes.grid(row=1, column=0, sticky='ew', pady=(4, 6))
        for i, (key, label) in enumerate(MODES):
            ttk.Radiobutton(modes, text=label, value=key, variable=self.mode_var,
                            command=self._update_selection_text).grid(row=i // 2, column=i % 2, sticky='w', padx=(0, 12))
        self.selection_var = tk.StringVar()
        ttk.Label(page, textvariable=self.selection_var, style='Muted.TLabel', wraplength=360,
                  justify='left').grid(row=2, column=0, sticky='w')

        opts = ttk.Frame(page)
        opts.grid(row=3, column=0, sticky='ew', pady=(10, 6))
        opts.columnconfigure(1, weight=1)
        ttk.Label(opts, text='Suction').grid(row=0, column=0, sticky='w')
        self.suction_var = tk.StringVar(value='standard')
        self.suction_combo = ttk.Combobox(opts, textvariable=self.suction_var, state='readonly',
                                          values=['eco', 'standard', 'turbo', 'max'], width=12)
        self.suction_combo.grid(row=0, column=1, sticky='w', padx=8)
        ttk.Label(opts, text='Passes').grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.passes_var = tk.IntVar(value=1)
        ttk.Spinbox(opts, from_=1, to=3, textvariable=self.passes_var, width=5).grid(
            row=1, column=1, sticky='w', padx=8, pady=(4, 0))
        self.edge_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(opts, text='Edge cleaning along walls first', variable=self.edge_var).grid(
            row=2, column=0, columnspan=2, sticky='w', pady=(4, 0))
        ttk.Label(opts, text='A second pass crosses the first at right angles.',
                  style='Muted.TLabel').grid(row=3, column=0, columnspan=2, sticky='w')
        ttk.Label(opts, text='Cleaning').grid(row=4, column=0, sticky='w', pady=(6, 0))
        self.clean_mode_var = tk.StringVar(value='Vacuum')
        self.clean_mode_combo = ttk.Combobox(opts, textvariable=self.clean_mode_var, state='readonly',
                                             values=[label for _, label in CLEAN_MODES], width=14)
        self.clean_mode_combo.grid(row=4, column=1, sticky='w', padx=8, pady=(6, 0))
        ttk.Label(opts, text='Water flow').grid(row=5, column=0, sticky='w', pady=(4, 0))
        self.water_flow_var = tk.StringVar(value='medium')
        self.water_flow_combo = ttk.Combobox(opts, textvariable=self.water_flow_var, state='readonly',
                                             values=WATER_FLOWS, width=10)
        self.water_flow_combo.grid(row=5, column=1, sticky='w', padx=8, pady=(4, 0))
        self.mop_hint_var = tk.StringVar(value='Mopping needs a robot with a mop module.')
        ttk.Label(opts, textvariable=self.mop_hint_var, style='Muted.TLabel', wraplength=340,
                  justify='left').grid(row=6, column=0, columnspan=2, sticky='w')

        buttons = ttk.Frame(page)
        buttons.grid(row=4, column=0, sticky='ew', pady=(8, 4))
        for c in range(3):
            buttons.columnconfigure(c, weight=1, uniform='b')
        self.start_button = ttk.Button(buttons, text='▶  Start', style='Accent.TButton', command=self._start)
        self.start_button.grid(row=0, column=0, sticky='ew', padx=2, pady=2)
        self.pause_button = ttk.Button(buttons, text='❚❚  Pause', command=lambda: self._send({'cmd': 'pause'}))
        self.pause_button.grid(row=0, column=1, sticky='ew', padx=2, pady=2)
        self.resume_button = ttk.Button(buttons, text='▶  Resume', command=lambda: self._send({'cmd': 'resume'}))
        self.resume_button.grid(row=0, column=2, sticky='ew', padx=2, pady=2)
        self.stop_button = ttk.Button(buttons, text='■  Stop', style='Danger.TButton',
                                      command=lambda: self._send({'cmd': 'stop'}))
        self.stop_button.grid(row=1, column=0, sticky='ew', padx=2, pady=2)
        self.dock_button = ttk.Button(buttons, text='⌂  Dock', command=lambda: self._send({'cmd': 'dock'}))
        self.dock_button.grid(row=1, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(buttons, text='◎  Locate', command=self._locate).grid(row=1, column=2, sticky='ew', padx=2, pady=2)
        self.estimate_var = tk.StringVar(value='')
        ttk.Label(page, textvariable=self.estimate_var, style='Muted.TLabel').grid(row=5, column=0, sticky='w')

        ttk.Label(page, text='Activity', style='Section.TLabel').grid(row=6, column=0, sticky='w', pady=(10, 2))
        self.event_log = scrolledtext.ScrolledText(page, height=8, width=44, wrap='word')
        self.event_log.grid(row=7, column=0, sticky='nsew')
        self.event_log.configure(state='disabled')
        page.rowconfigure(7, weight=1)
        self._update_selection_text()

    def _build_rooms(self, page):
        page.rowconfigure(1, weight=1)
        ttk.Label(page, text='Rooms are found automatically from the map. Click rooms on the map to select '
                  'them; the order you click is the cleaning order.', style='Muted.TLabel', wraplength=360,
                  justify='left').grid(row=0, column=0, sticky='w', pady=(0, 6))
        cols = ('order', 'name', 'area', 'suction', 'passes', 'edge')
        self.room_tree = ttk.Treeview(page, columns=cols, show='headings', height=8, selectmode='extended')
        for c, w in zip(cols, (44, 120, 60, 70, 50, 44)):
            self.room_tree.heading(c, text=c.title())
            self.room_tree.column(c, width=w, anchor='w')
        self.room_tree.grid(row=1, column=0, sticky='nsew')
        self.room_tree.bind('<<TreeviewSelect>>', self._room_tree_selected)
        row = ttk.Frame(page)
        row.grid(row=2, column=0, sticky='ew', pady=(6, 0))
        for i, (text, cmd) in enumerate([('↑', lambda: self._move_room(-1)), ('↓', lambda: self._move_room(1)),
                                         ('Rename…', self._rename_room), ('Merge', self._merge_rooms)]):
            ttk.Button(row, text=text, command=cmd, width=8 if len(text) > 1 else 3).grid(row=0, column=i, padx=(0, 4))
        per = ttk.LabelFrame(page, text='Settings for the selected room', padding=8)
        per.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        self.room_suction_var = tk.StringVar(value='')
        ttk.Label(per, text='Suction').grid(row=0, column=0, sticky='w')
        self.room_suction_combo = ttk.Combobox(per, textvariable=self.room_suction_var, state='readonly',
                                               values=['', 'eco', 'standard', 'turbo', 'max'], width=10)
        self.room_suction_combo.grid(row=0, column=1, sticky='w', padx=6)
        ttk.Label(per, text='Passes').grid(row=0, column=2, sticky='w')
        self.room_passes_var = tk.StringVar(value='')
        ttk.Combobox(per, textvariable=self.room_passes_var, state='readonly', values=['', '1', '2', '3'],
                     width=4).grid(row=0, column=3, sticky='w', padx=6)
        self.room_edge_var = tk.StringVar(value='')
        ttk.Label(per, text='Edges').grid(row=1, column=0, sticky='w', pady=(4, 0))
        ttk.Combobox(per, textvariable=self.room_edge_var, state='readonly', values=['', 'on', 'off'],
                     width=6).grid(row=1, column=1, sticky='w', padx=6, pady=(4, 0))
        ttk.Button(per, text='Apply', command=self._apply_room_settings).grid(row=1, column=3, sticky='e', pady=(4, 0))
        ttk.Label(per, text='Blank = use the job setting.', style='Muted.TLabel').grid(
            row=2, column=0, columnspan=4, sticky='w', pady=(4, 0))
        seg = ttk.Frame(page)
        seg.grid(row=4, column=0, sticky='ew', pady=(8, 0))
        ttk.Label(seg, text='Doorway width (m)').grid(row=0, column=0, sticky='w')
        self.door_var = tk.DoubleVar(value=0.9)
        ttk.Spinbox(seg, from_=0.5, to=2.0, increment=0.1, textvariable=self.door_var, width=5).grid(
            row=0, column=1, padx=6)
        ttk.Button(seg, text='Divide map again', command=self._resegment).grid(row=0, column=2, padx=(4, 0))
        ttk.Label(page, text='Use the Split tool on the map to divide a room.', style='Muted.TLabel').grid(
            row=5, column=0, sticky='w', pady=(4, 0))

    def _build_schedule(self, page):
        page.rowconfigure(1, weight=1)
        ttk.Label(page, text='Scheduled cleanings run on the robot (local time), also with this window closed.',
                  style='Muted.TLabel', wraplength=360, justify='left').grid(row=0, column=0, sticky='w')
        cols = ('on', 'time', 'days', 'what', 'suction')
        self.schedule_tree = ttk.Treeview(page, columns=cols, show='headings', height=6)
        for c, w in zip(cols, (34, 54, 130, 110, 70)):
            self.schedule_tree.heading(c, text=c.title())
            self.schedule_tree.column(c, width=w, anchor='w')
        self.schedule_tree.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        self.schedule_tree.bind('<<TreeviewSelect>>', self._schedule_selected)
        form = ttk.LabelFrame(page, text='Cleaning plan', padding=8)
        form.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        ttk.Label(form, text='Time').grid(row=0, column=0, sticky='w')
        self.sched_time_var = tk.StringVar(value='09:00')
        ttk.Entry(form, textvariable=self.sched_time_var, width=7).grid(row=0, column=1, sticky='w', padx=6)
        self.sched_enabled_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(form, text='Enabled', variable=self.sched_enabled_var).grid(row=0, column=2, columnspan=3, sticky='w')
        self.day_vars = []
        days = ttk.Frame(form)
        days.grid(row=1, column=0, columnspan=6, sticky='w', pady=(4, 0))
        for i, name in enumerate(DAYS):
            var = tk.BooleanVar(value=i < 5)
            self.day_vars.append(var)
            ttk.Checkbutton(days, text=name, variable=var).grid(row=0, column=i, padx=(0, 2))
        ttk.Label(form, text='Clean').grid(row=2, column=0, sticky='w', pady=(4, 0))
        self.sched_mode_var = tk.StringVar(value='all')
        ttk.Combobox(form, textvariable=self.sched_mode_var, state='readonly', values=['all', 'rooms', 'edges'],
                     width=8).grid(row=2, column=1, sticky='w', padx=6, pady=(4, 0))
        ttk.Label(form, text='Suction').grid(row=2, column=2, sticky='w', pady=(4, 0))
        self.sched_suction_var = tk.StringVar(value='standard')
        ttk.Combobox(form, textvariable=self.sched_suction_var, state='readonly',
                     values=['eco', 'standard', 'turbo', 'max'], width=9).grid(row=2, column=3, sticky='w', padx=6, pady=(4, 0))
        ttk.Label(form, text='"rooms" uses the rooms selected on the map.', style='Muted.TLabel').grid(
            row=3, column=0, columnspan=6, sticky='w', pady=(2, 0))
        row = ttk.Frame(form)
        row.grid(row=4, column=0, columnspan=6, sticky='w', pady=(6, 0))
        ttk.Button(row, text='Add', style='Accent.TButton', command=lambda: self._save_schedule_entry(new=True)).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(row, text='Update', command=lambda: self._save_schedule_entry(new=False)).grid(row=0, column=1, padx=4)
        ttk.Button(row, text='Delete', command=self._delete_schedule_entry).grid(row=0, column=2, padx=4)
        dnd = ttk.LabelFrame(page, text='Do not disturb', padding=8)
        dnd.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        self.dnd_var = tk.BooleanVar(value=False)
        self.dnd_start_var = tk.StringVar(value='22:00')
        self.dnd_end_var = tk.StringVar(value='08:00')
        ttk.Checkbutton(dnd, text='No automatic resume from', variable=self.dnd_var).grid(row=0, column=0, sticky='w')
        ttk.Entry(dnd, textvariable=self.dnd_start_var, width=6).grid(row=0, column=1, padx=4)
        ttk.Label(dnd, text='to').grid(row=0, column=2)
        ttk.Entry(dnd, textvariable=self.dnd_end_var, width=6).grid(row=0, column=3, padx=4)
        ttk.Button(dnd, text='Save', command=self._save_dnd).grid(row=0, column=4, padx=(6, 0))

    def _build_history(self, page):
        page.rowconfigure(0, weight=1)
        cols = ('when', 'what', 'result', 'area', 'cov', 'time')
        self.history_tree = ttk.Treeview(page, columns=cols, show='headings', height=12)
        for c, label, w in zip(cols, ('When', 'What', 'Result', 'm²', '%', 'Time'), (110, 90, 100, 50, 40, 70)):
            self.history_tree.heading(c, text=label)
            self.history_tree.column(c, width=w, anchor='w')
        self.history_tree.grid(row=0, column=0, sticky='nsew')
        self.history_tree.bind('<<TreeviewSelect>>', self._history_selected)
        row = ttk.Frame(page)
        row.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        ttk.Button(row, text='Show on map', command=self._show_history).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(row, text='Live map', command=self._clear_history_overlay).grid(row=0, column=1, padx=4)
        self.history_detail = tk.StringVar(value='')
        ttk.Label(page, textvariable=self.history_detail, style='Muted.TLabel', wraplength=360,
                  justify='left').grid(row=2, column=0, sticky='w', pady=(6, 0))
        self.totals_var = tk.StringVar(value='')
        ttk.Label(page, textvariable=self.totals_var, style='SurfaceBody.TLabel').grid(row=3, column=0, sticky='w', pady=(6, 0))

    def _build_care(self, page):
        ttk.Label(page, text='Maintenance', style='Section.TLabel').grid(row=0, column=0, sticky='w')
        self.part_widgets = {}
        parts = ttk.Frame(page)
        parts.grid(row=1, column=0, sticky='ew', pady=(4, 0))
        parts.columnconfigure(1, weight=1)
        for i, (key, label) in enumerate([('main_brush', 'Main brush'), ('side_brush', 'Side brush'),
                                          ('filter', 'Filter'), ('sensors', 'Sensors (clean)'),
                                          ('mop_pad', 'Mop pad')]):
            ttk.Label(parts, text=label).grid(row=i, column=0, sticky='w', pady=2)
            bar = ttk.Progressbar(parts, maximum=100, length=120)
            bar.grid(row=i, column=1, sticky='ew', padx=6)
            var = tk.StringVar(value='–')
            ttk.Label(parts, textvariable=var, style='Muted.TLabel', width=16).grid(row=i, column=2, sticky='w')
            ttk.Button(parts, text='Reset', width=6,
                       command=lambda k=key: self._send({'cmd': 'reset_consumable', 'name': k})).grid(row=i, column=3)
            self.part_widgets[key] = (bar, var)
        binf = ttk.Frame(page)
        binf.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        binf.columnconfigure(1, weight=1)
        ttk.Label(binf, text='Dustbin').grid(row=0, column=0, sticky='w')
        self.bin_bar = ttk.Progressbar(binf, maximum=100, length=120)
        self.bin_bar.grid(row=0, column=1, sticky='ew', padx=6)
        ttk.Button(binf, text='Emptied', width=8, command=lambda: self._send({'cmd': 'empty_bin'})).grid(row=0, column=2)
        ttk.Label(binf, text='Water tank').grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.water_bar = ttk.Progressbar(binf, maximum=100, length=120)
        self.water_bar.grid(row=1, column=1, sticky='ew', padx=6, pady=(4, 0))
        self.water_var = tk.StringVar(value='no mop module')
        ttk.Label(binf, textvariable=self.water_var, style='Muted.TLabel').grid(row=1, column=2, sticky='w', pady=(4, 0))

        ttk.Label(page, text='Behaviour', style='Section.TLabel').grid(row=3, column=0, sticky='w', pady=(12, 0))
        beh = ttk.Frame(page)
        beh.grid(row=4, column=0, sticky='ew')
        self.charge_resume_var = tk.BooleanVar(value=True)
        self.auto_empty_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(beh, text='Recharge and resume when the battery runs low',
                        variable=self.charge_resume_var).grid(row=0, column=0, columnspan=4, sticky='w')
        ttk.Checkbutton(beh, text='Auto-empty dock (empties the bin when full)',
                        variable=self.auto_empty_var).grid(row=1, column=0, columnspan=4, sticky='w')
        self.carpet_boost_var = tk.BooleanVar(value=True)
        self.auto_refill_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(beh, text='Carpet boost (Max suction on carpet areas)',
                        variable=self.carpet_boost_var).grid(row=2, column=0, columnspan=4, sticky='w')
        ttk.Checkbutton(beh, text='Refilling dock (refills the water tank and resumes)',
                        variable=self.auto_refill_var).grid(row=3, column=0, columnspan=4, sticky='w')
        ttk.Label(beh, text='Return at').grid(row=4, column=0, sticky='w', pady=(4, 0))
        self.low_var = tk.IntVar(value=20)
        ttk.Spinbox(beh, from_=5, to=50, textvariable=self.low_var, width=4).grid(row=4, column=1, sticky='w', pady=(4, 0))
        ttk.Label(beh, text='% · resume at').grid(row=4, column=2, sticky='w', pady=(4, 0))
        self.resume_var = tk.IntVar(value=80)
        ttk.Spinbox(beh, from_=30, to=100, textvariable=self.resume_var, width=4).grid(row=4, column=3, sticky='w', pady=(4, 0))
        ttk.Label(beh, text='Battery model speed').grid(row=5, column=0, sticky='w', pady=(4, 0))
        self.scale_var = tk.StringVar(value='1')
        ttk.Combobox(beh, textvariable=self.scale_var, values=['1', '10', '30', '60'], width=5).grid(
            row=5, column=1, sticky='w', pady=(4, 0))
        ttk.Label(beh, text='× (simulated battery)', style='Muted.TLabel').grid(row=5, column=2, columnspan=2, sticky='w', pady=(4, 0))
        ttk.Label(beh, text='Speed').grid(row=6, column=0, sticky='w', pady=(4, 0))
        self.speed_scale_var = tk.StringVar(value='1.0')
        ttk.Combobox(beh, textvariable=self.speed_scale_var, values=['0.6', '0.8', '1.0', '1.2'], width=5).grid(
            row=6, column=1, sticky='w', pady=(4, 0))
        ttk.Button(beh, text='Save', style='Accent.TButton', command=self._save_settings).grid(row=7, column=0, sticky='w', pady=(8, 0))
        tools = ttk.Frame(page)
        tools.grid(row=5, column=0, sticky='ew', pady=(12, 0))
        ttk.Button(tools, text='Forget obstacles found on runs',
                   command=lambda: self._send({'cmd': 'clear_map_edits'})).grid(row=0, column=0, sticky='w')
        ttk.Label(page, text='Battery, dustbin, water tank and part wear are models — the simulators have no '
                  'battery. Wear counts cleaning time; the mop pad wears only while mopping.', style='Muted.TLabel', wraplength=360, justify='left').grid(
            row=6, column=0, sticky='w', pady=(10, 0))

    def _build_setup(self, page):
        capable = sorted(rid for rid, cfg in self.app.robot_profiles.items()
                         if bool((cfg or {}).get('supports_room_vacuum', False)))
        default_robot = 'oomwoo' if 'oomwoo' in capable else (capable[0] if capable else '')
        ttk.Label(page, text='Simulation', style='Section.TLabel').grid(row=0, column=0, sticky='w')
        form = ttk.Frame(page)
        form.grid(row=1, column=0, sticky='ew', pady=(4, 0))
        form.columnconfigure(1, weight=1)
        self.vacuum_robot_var = tk.StringVar(value=default_robot)
        ttk.Label(form, text='Robot').grid(row=0, column=0, sticky='w')
        robot_combo = ttk.Combobox(form, textvariable=self.vacuum_robot_var, values=capable, state='readonly')
        robot_combo.grid(row=0, column=1, sticky='ew', padx=6, pady=2)
        robot_combo.bind('<<ComboboxSelected>>', self._on_robot_selected)
        maps = [m for m in SUPPORTED_MAPS if m in getattr(self.app, 'map_profiles', {}) or not getattr(self.app, 'map_profiles', None)]
        self.vacuum_map_var = tk.StringVar(value='small_house' if 'small_house' in maps else (maps[0] if maps else ''))
        ttk.Label(form, text='Home map').grid(row=1, column=0, sticky='w')
        ttk.Combobox(form, textvariable=self.vacuum_map_var, values=maps).grid(row=1, column=1, sticky='ew', padx=6, pady=2)
        self.vacuum_sim_var = tk.StringVar(value='gazebo')
        ttk.Label(form, text='Simulator').grid(row=2, column=0, sticky='w')
        ttk.Combobox(form, textvariable=self.vacuum_sim_var, state='readonly',
                     values=['gazebo', 'mujoco', 'pybullet', 'isaac']).grid(row=2, column=1, sticky='ew', padx=6, pady=2)
        self.vacuum_mode_var = tk.StringVar(value='nav')
        ttk.Label(form, text='Map source').grid(row=3, column=0, sticky='w')
        src = ttk.Frame(form)
        src.grid(row=3, column=1, sticky='w', padx=6)
        ttk.Radiobutton(src, text='Saved map', value='nav', variable=self.vacuum_mode_var).grid(row=0, column=0, padx=(0, 8))
        ttk.Radiobutton(src, text='Build a new map (SLAM)', value='slam', variable=self.vacuum_mode_var).grid(row=0, column=1)
        row = ttk.Frame(page)
        row.grid(row=2, column=0, sticky='ew', pady=(8, 0))
        ttk.Button(row, text='▶  Launch robot + cleaning', style='Accent.TButton',
                   command=self._launch).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(row, text='Stop simulation', command=self.app.stop_launch).grid(row=0, column=1, padx=4)
        ttk.Label(page, text='With "Build a new map", run a Mapping run first; rooms are divided when it ends.',
                  style='Muted.TLabel', wraplength=360, justify='left').grid(row=3, column=0, sticky='w', pady=(6, 0))
        ttk.Label(page, text='Already running a simulation?', style='Section.TLabel').grid(row=4, column=0, sticky='w', pady=(12, 0))
        row2 = ttk.Frame(page)
        row2.grid(row=5, column=0, sticky='ew', pady=(4, 0))
        ttk.Button(row2, text='Start cleaning system', command=self._start_node).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(row2, text='Stop it', command=lambda: self.app.stop_bg_process('vacuum_cleaner')).grid(row=0, column=1, padx=4)
        ttk.Label(page, text='Dock', style='Section.TLabel').grid(row=6, column=0, sticky='w', pady=(12, 0))
        ttk.Label(page, text='The first job sets the dock where the robot starts. Use the Dock tool on the map, '
                  'or:', style='Muted.TLabel', wraplength=360, justify='left').grid(row=7, column=0, sticky='w')
        ttk.Button(page, text='Dock is where the robot is now', command=lambda: self._send({'cmd': 'set_dock'})).grid(
            row=8, column=0, sticky='w', pady=(4, 0))
        ttk.Label(page, text='Drive it by hand with the Drive pad in Launch (keyboard and joystick override '
                  'the cleaning system).', style='Muted.TLabel', wraplength=360, justify='left').grid(
            row=9, column=0, sticky='w', pady=(12, 0))
        ttk.Button(page, text='Open the drive pad  →', command=lambda: self.app.show_tab('Drive')).grid(
            row=10, column=0, sticky='w', pady=(4, 0))

    # ------------------------------------------------------------------
    # Launch
    # ------------------------------------------------------------------
    def _on_robot_selected(self, _event=None):
        robot = self.vacuum_robot_var.get()
        try:
            self.app.robot_var.set(robot)
            self.app._update_from_selection()
        except (AttributeError, tk.TclError):
            pass
        self.app.set_status('Vacuum robot: %s' % robot)

    def launch_command(self):
        return ['ros2', 'launch', 'robot_lab_bringup', 'simulated_room_vacuum.launch.py',
                'robot_model:=%s' % self.vacuum_robot_var.get(),
                'map_name:=%s' % self.vacuum_map_var.get(),
                'simulator:=%s' % self.vacuum_sim_var.get(),
                'mode:=%s' % self.vacuum_mode_var.get(),
                'controller:=cleaning']

    def _launch(self, extra=()):
        # The header names the running session from the main selection.
        for name, var in (('robot_var', self.vacuum_robot_var), ('map_var', self.vacuum_map_var),
                          ('simulator_var', self.vacuum_sim_var)):
            if hasattr(self.app, name):
                getattr(self.app, name).set(var.get())
        self.app.start_launch_command(self.launch_command() + list(extra))
        self._ensure_ros()

    def node_command(self):
        return ['ros2', 'run', 'robot_lab_vacuum_cleaning', 'vacuum_cleaner', '--ros-args',
                '-p', 'robot_model:=%s' % self.vacuum_robot_var.get(),
                '-p', 'map_key:=%s' % self.vacuum_map_var.get(),
                '-p', 'use_sim_time:=true']

    def _start_node(self):
        self.app.start_bg_process(self.node_command(), 'vacuum_cleaner')
        self._ensure_ros()

    # ------------------------------------------------------------------
    # ROS
    # ------------------------------------------------------------------
    def _ensure_ros(self):
        if self.node is not None or self.closed:
            return self.node is not None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from nav_msgs.msg import OccupancyGrid
            from std_msgs.msg import String
        except ImportError as exc:
            self.ros_error = 'ROS 2 Python is not available (%s)' % exc
            return False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node('vacuum_app_%d' % (int(time.time() * 1000) % 100000))
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            sub = self.node.create_subscription
            sub(String, '/vacuum/status', lambda m: self._on_json('status', m), 10)
            sub(String, '/vacuum/layout', lambda m: self._on_json('layout', m), latched)
            sub(String, '/vacuum/history', lambda m: self._on_json('history', m), latched)
            sub(String, '/vacuum/plan', lambda m: self._on_json('plan', m), latched)
            sub(String, '/vacuum/events', self._on_event, 50)
            sub(OccupancyGrid, '/map', self._on_map, latched)
            sub(OccupancyGrid, '/vacuum/rooms', lambda m: self._on_grid('rooms', m), latched)
            sub(OccupancyGrid, '/vacuum/coverage', lambda m: self._on_grid('coverage', m), latched)
            self.command_pub = self.node.create_publisher(String, '/vacuum/command', 10)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
        except Exception as exc:  # rclpy raises several runtime errors
            self.ros_error = 'Could not start ROS: %s' % exc
            self.node = None
            return False
        return True

    def _on_json(self, kind, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if kind == 'status':
            self.status = data
            self.last_status_time = time.monotonic()
            self._apply_status()
        elif kind == 'layout':
            self.layout = data
            self._apply_layout()
        elif kind == 'history':
            self.history = data
            self._apply_history()
        elif kind == 'plan':
            self.plan = data
            self._draw_overlays()

    def _on_event(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        self.log_event(event.get('text', ''), event.get('level', 'info'), event.get('t'))

    def _on_map(self, msg):
        if np is None:
            return
        info = msg.info
        self.map_info = (info.width, info.height, info.resolution, info.origin.position.x, info.origin.position.y)
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self._mark_dirty()

    def _on_grid(self, kind, msg):
        if np is None:
            return
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        if kind == 'rooms':
            self.rooms_grid = data
        else:
            self.coverage_grid = data > 50
        self._mark_dirty()

    def _send(self, cmd):
        if self.command_pub is None and not self._ensure_ros():
            self.message_var.set(self.ros_error or 'ROS is not available')
            return False
        if not self.status:
            self.message_var.set('The cleaning system is not running yet — launch it in Setup.')
        from std_msgs.msg import String
        self.command_pub.publish(String(data=json.dumps(cmd)))
        return True

    def _poll(self):
        if self.closed:
            return
        if self.executor is not None:
            try:
                for _ in range(20):
                    self.executor.spin_once(timeout_sec=0.0)
            except Exception:
                pass
        if self.status and time.monotonic() - self.last_status_time > 3.0:
            self.state_var.set('○  Cleaning system not responding')
            self.state_label.configure(style='PillWarn.TLabel')
        if self.dirty_image and time.monotonic() - self.last_image > 0.8:
            self._render()
        if self.locate_until and time.monotonic() > self.locate_until:
            self.locate_until = 0.0
            self._draw_robot()
        self.job = self.after(100, self._poll)

    def close(self):
        self.closed = True
        if self.job is not None:
            try:
                self.after_cancel(self.job)
            except tk.TclError:
                pass
        try:
            if self.executor is not None:
                self.executor.shutdown()
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass
        self.node = None
        self.executor = None

    # ------------------------------------------------------------------
    # Applying node state
    # ------------------------------------------------------------------
    def _apply_status(self):
        s = self.status
        state = s.get('state', 'idle')
        text, style = STATE_TEXT.get(state, (state.title(), 'PillIdle.TLabel'))
        if state == 'error' and s.get('error'):
            text = 'Error'
        bullet = '●' if style == 'PillRunning.TLabel' else '▲' if style == 'PillWarn.TLabel' else '○'
        task = s.get('task')
        if state == 'cleaning' and task in ('edge', 'transit'):
            text += ' · ' + ('edges' if task == 'edge' else 'moving')
        self.state_var.set('%s  %s' % (bullet, text))
        self.state_label.configure(style=style)
        battery = s.get('battery_pct', 0.0)
        self.battery_bar['value'] = battery
        self.metric_vars['battery'].set('%.0f%% · %s left' % (battery, fmt_duration(60 * s.get('battery_runtime_min', 0))))
        self.metric_vars['area'].set('%.1f m²' % s.get('area_m2', 0.0))
        self.metric_vars['coverage'].set('%.0f%%' % s.get('coverage_pct', 0.0) if s.get('job') else '–')
        if s.get('job'):
            self.metric_vars['time'].set('%s · ~%s to go' % (fmt_duration(s.get('elapsed_s')), fmt_duration(s.get('remaining_s'))))
        else:
            self.metric_vars['time'].set('–')
        self.metric_vars['room'].set(s.get('room_name') or '–')
        self.metric_vars['bin'].set('%.0f%%' % s.get('dustbin_pct', 0.0))
        self.bin_bar['value'] = s.get('dustbin_pct', 0.0)
        if state == 'error':
            self.message_var.set(s.get('error') or 'Error')
        elif not s.get('map_ready'):
            self.message_var.set('Waiting for the map…')
        elif not s.get('localised'):
            self.message_var.set('Waiting for the robot position (map → base transform)…')
        elif s.get('resume_after_dock') == 'charge' and state in ('charging', 'returning', 'docking'):
            self.message_var.set('Recharging; the job continues at %d%%.' % int(self.resume_var.get()))
        elif state in ('idle', 'docked', 'charging'):
            self.message_var.set('Ready. Choose what to clean and press Start.')
        else:
            job = s.get('job') or {}
            how = dict(CLEAN_MODES).get(job.get('clean_mode') or 'vacuum', 'Vacuum')
            extra = ' · carpet boost' if s.get('carpet_boost_active') else ''
            if job.get('clean_mode') in ('vacuum_mop', 'mop'):
                extra += ' · %.1f m² mopped · water %.0f%%' % (s.get('mopped_m2', 0.0), s.get('water_pct') or 0.0)
            self.message_var.set('%s job (%s) · %d paths left · suction %s%s' % (
                dict(MODES).get(job.get('mode'), job.get('mode', '')), how, s.get('tasks_left', 0),
                s.get('suction_label', ''), extra))
        for key, (bar, var) in self.part_widgets.items():
            c = (s.get('consumables') or {}).get(key)
            if c:
                bar['value'] = c['left_pct']
                var.set('%.0f%% · %.1f/%d h' % (c['left_pct'], c['used_h'], c['life_h']))
        mop_module = bool(s.get('mop_module'))
        self.clean_mode_combo.state(['!disabled'] if mop_module else ['disabled'])
        self.water_flow_combo.state(['!disabled'] if mop_module else ['disabled'])
        if not mop_module:
            self.clean_mode_var.set('Vacuum')
        water = s.get('water_pct')
        self.water_bar['value'] = water or 0.0
        self.water_var.set('no mop module' if water is None else '%.0f%%' % water)
        self.mop_hint_var.set('Mopping keeps off carpet areas; edges are vacuumed only.' if mop_module else
                              'Mopping needs a robot with a mop module.')
        busy = state in ('cleaning', 'mapping', 'returning', 'docking', 'emptying', 'refilling')
        self.start_button.state(['disabled'] if busy else ['!disabled'])
        self.pause_button.state(['!disabled'] if busy else ['disabled'])
        self.resume_button.state(['!disabled'] if state in ('paused', 'error') or (
            state in ('docked', 'charging') and s.get('tasks_left')) else ['disabled'])
        self._draw_robot()

    def _apply_layout(self):
        lay = self.layout
        modes = list((lay.get('power_modes') or {}).keys())
        if modes:
            self.suction_combo.configure(values=modes)
            self.room_suction_combo.configure(values=[''] + modes)
        settings = lay.get('settings') or {}
        if settings:
            self.charge_resume_var.set(bool(settings.get('charge_resume', True)))
            self.auto_empty_var.set(bool(settings.get('auto_empty', True)))
            self.low_var.set(int(settings.get('low_battery_pct', 20)))
            self.resume_var.set(int(settings.get('resume_battery_pct', 80)))
            self.scale_var.set(str(settings.get('battery_time_scale', 1)).rstrip('0').rstrip('.') or '1')
            self.speed_scale_var.set(str(settings.get('speed_scale', 1.0)))
            self.carpet_boost_var.set(bool(settings.get('carpet_boost', True)))
            self.auto_refill_var.set(bool(settings.get('auto_refill', True)))
            dnd = settings.get('dnd') or {}
            self.dnd_var.set(bool(dnd.get('enabled')))
            self.dnd_start_var.set(dnd.get('start', '22:00'))
            self.dnd_end_var.set(dnd.get('end', '08:00'))
        rooms = lay.get('rooms') or {}
        self.selected_rooms = [r for r in self.selected_rooms if str(r) in rooms]
        selection = self.room_tree.selection()
        self.room_tree.delete(*self.room_tree.get_children())
        for i, rid in enumerate(lay.get('room_order') or []):
            meta = rooms.get(str(rid))
            if not meta:
                continue
            edge = meta.get('edge')
            self.room_tree.insert('', 'end', iid=str(rid), values=(
                i + 1, meta.get('name', 'Room %s' % rid), '%.1f' % meta.get('area_m2', 0.0),
                meta.get('suction', '') or '–', meta.get('passes', '') or '–',
                '–' if edge is None else ('on' if edge else 'off')))
        keep = [iid for iid in selection if self.room_tree.exists(iid)]
        if keep:
            self.room_tree.selection_set(keep)
        self.schedule_tree.delete(*self.schedule_tree.get_children())
        for entry in lay.get('schedule') or []:
            what = entry.get('mode', 'all')
            if what == 'rooms':
                what = ', '.join(rooms.get(str(r), {}).get('name', str(r)) for r in entry.get('rooms', [])) or 'rooms'
            self.schedule_tree.insert('', 'end', iid=entry['id'], values=(
                '✓' if entry.get('enabled', True) else '', entry.get('time', ''),
                ' '.join(DAYS[d] for d in entry.get('days', [])), what, entry.get('suction', '')))
        self._update_selection_text()
        self._mark_dirty()

    def _apply_history(self):
        self.history_tree.delete(*self.history_tree.get_children())
        total_area, total_time = 0.0, 0.0
        for rec in reversed(self.history):
            when = time.strftime('%d %b %H:%M', time.localtime(rec.get('started', 0)))
            what = dict(MODES).get(rec.get('mode'), rec.get('mode', ''))
            if rec.get('room_names'):
                what = ', '.join(rec['room_names'])
            self.history_tree.insert('', 'end', iid=rec['id'], values=(
                when, what, rec.get('result', ''), '%.1f' % rec.get('area_m2', 0.0),
                '%.0f' % rec.get('coverage_pct', 0.0), fmt_duration(rec.get('duration_s', 0))))
            total_area += rec.get('area_m2', 0.0)
            total_time += rec.get('duration_s', 0.0)
        self.totals_var.set('%d runs · %.0f m² · %s cleaning' % (len(self.history), total_area, fmt_duration(total_time))
                            if self.history else 'No runs yet.')

    def log_event(self, text, level='info', stamp=None):
        if not text:
            return
        mark = {'warn': '⚠ ', 'error': '✖ '}.get(level, '')
        line = '%s  %s%s\n' % (time.strftime('%H:%M:%S', time.localtime(stamp or time.time())), mark, text)
        self.event_log.configure(state='normal')
        self.event_log.insert('end', line)
        lines = int(self.event_log.index('end-1c').split('.')[0])
        if lines > 400:
            self.event_log.delete('1.0', '%d.0' % (lines - 400))
        self.event_log.see('end')
        self.event_log.configure(state='disabled')

    # ------------------------------------------------------------------
    # Clean page actions
    # ------------------------------------------------------------------
    def room_name(self, rid):
        return ((self.layout.get('rooms') or {}).get(str(rid)) or {}).get('name', 'Room %s' % rid)

    def _update_selection_text(self):
        mode = self.mode_var.get()
        if mode == 'rooms':
            text = ('Rooms in order: ' + ' → '.join(self.room_name(r) for r in self.selected_rooms)
                    if self.selected_rooms else 'Click rooms on the map (Rooms tool).')
        elif mode == 'zones':
            n = len(self.layout.get('zones') or [])
            text = '%d zone(s) drawn. Draw zones with the Zone tool.' % n
        elif mode == 'spot':
            text = ('Spot at (%.1f, %.1f), 1.6 m square, twice.' % tuple(self.spot_point)
                    if self.spot_point else 'Click a spot with the Spot tool (or clean around the robot).')
        elif mode == 'explore':
            text = 'Drives to the edges of the known map until it is complete, then divides rooms.'
        elif mode == 'edges':
            text = 'Along every wall and piece of furniture in all rooms.'
        else:
            text = 'All rooms in the order on the Rooms page.'
        self.selection_var.set(text)

    def start_request(self):
        mode = self.mode_var.get()
        req = {'cmd': 'start', 'mode': mode, 'suction': self.suction_var.get(),
               'passes': int(self.passes_var.get() or 1), 'edge': bool(self.edge_var.get())}
        clean_mode = dict((label, key) for key, label in CLEAN_MODES).get(self.clean_mode_var.get(), 'vacuum')
        if clean_mode != 'vacuum' and mode != 'explore':
            req['clean_mode'] = clean_mode
            req['water_flow'] = self.water_flow_var.get()
        if mode == 'rooms':
            req['rooms'] = [int(r) for r in self.selected_rooms]
        elif mode == 'zones':
            req['zones'] = [z for z in self.layout.get('zones') or []]
        elif mode == 'spot' and self.spot_point:
            req['point'] = list(self.spot_point)
        return req

    def _start(self):
        req = self.start_request()
        if req['mode'] == 'rooms' and not req['rooms']:
            self.message_var.set('Select rooms on the map first (Rooms tool).')
            return
        if req['mode'] == 'zones' and not req['zones']:
            self.message_var.set('Draw at least one zone on the map first (Zone tool).')
            return
        self._clear_history_overlay()
        self._send(req)

    def _locate(self):
        self.locate_until = time.monotonic() + 3.0
        self._send({'cmd': 'locate'})
        self._draw_robot()

    # ------------------------------------------------------------------
    # Rooms page actions
    # ------------------------------------------------------------------
    def _room_tree_selected(self, _event=None):
        sel = self.room_tree.selection()
        if len(sel) == 1:
            meta = (self.layout.get('rooms') or {}).get(sel[0], {})
            self.room_suction_var.set(meta.get('suction', '') or '')
            self.room_passes_var.set(str(meta.get('passes', '') or ''))
            edge = meta.get('edge')
            self.room_edge_var.set('' if edge is None else ('on' if edge else 'off'))

    def _move_room(self, delta):
        sel = self.room_tree.selection()
        order = [int(r) for r in self.layout.get('room_order') or []]
        if len(sel) != 1 or int(sel[0]) not in order:
            return
        i = order.index(int(sel[0]))
        j = max(0, min(len(order) - 1, i + delta))
        order[i], order[j] = order[j], order[i]
        self._send({'cmd': 'room_order', 'order': order})

    def _rename_room(self):
        sel = self.room_tree.selection()
        if len(sel) != 1:
            self.message_var.set('Select one room to rename.')
            return
        name = simpledialog.askstring('Rename room', 'New name:', initialvalue=self.room_name(sel[0]), parent=self)
        if name:
            self._send({'cmd': 'rename_room', 'room': int(sel[0]), 'name': name})

    def _merge_rooms(self):
        ids = [int(r) for r in self.room_tree.selection()] or [int(r) for r in self.selected_rooms]
        if len(ids) < 2:
            self.message_var.set('Select two or more adjoining rooms to merge.')
            return
        self._send({'cmd': 'merge_rooms', 'rooms': ids})
        self.selected_rooms = []

    def _apply_room_settings(self):
        sel = self.room_tree.selection()
        if not sel:
            self.message_var.set('Select a room in the list first.')
            return
        for rid in sel:
            edge = self.room_edge_var.get()
            passes = self.room_passes_var.get()
            self._send({'cmd': 'room_settings', 'room': int(rid),
                        'suction': self.room_suction_var.get() or None,
                        'passes': int(passes) if passes else None,
                        'edge': None if not edge else edge == 'on'})

    def _resegment(self):
        self.selected_rooms = []
        self._send({'cmd': 'segment', 'door_width': float(self.door_var.get())})

    # ------------------------------------------------------------------
    # Schedule, care, history
    # ------------------------------------------------------------------
    def _schedule_selected(self, _event=None):
        sel = self.schedule_tree.selection()
        entry = next((e for e in self.layout.get('schedule') or [] if sel and e['id'] == sel[0]), None)
        if entry:
            self.sched_time_var.set(entry.get('time', '09:00'))
            self.sched_enabled_var.set(bool(entry.get('enabled', True)))
            for i, var in enumerate(self.day_vars):
                var.set(i in entry.get('days', []))
            self.sched_mode_var.set(entry.get('mode', 'all'))
            self.sched_suction_var.set(entry.get('suction', 'standard'))

    def schedule_entry_from_form(self):
        t = self.sched_time_var.get().strip()
        try:
            hh, mm = (int(v) for v in t.split(':'))
            assert 0 <= hh < 24 and 0 <= mm < 60
        except (ValueError, AssertionError):
            raise ValueError('Time must be HH:MM')
        days = [i for i, v in enumerate(self.day_vars) if v.get()]
        if not days:
            raise ValueError('Pick at least one day')
        mode = self.sched_mode_var.get()
        if mode == 'rooms' and not self.selected_rooms:
            raise ValueError('Select the rooms on the map first')
        return {'enabled': bool(self.sched_enabled_var.get()), 'time': '%02d:%02d' % (hh, mm), 'days': days,
                'mode': mode, 'rooms': [int(r) for r in self.selected_rooms] if mode == 'rooms' else [],
                'suction': self.sched_suction_var.get(), 'passes': 1}

    def _save_schedule_entry(self, new):
        try:
            entry = self.schedule_entry_from_form()
        except ValueError as exc:
            self.message_var.set(str(exc))
            return
        entries = [dict(e) for e in self.layout.get('schedule') or []]
        sel = self.schedule_tree.selection()
        if not new and sel:
            entries = [dict(entry, id=e['id']) if e['id'] == sel[0] else e for e in entries]
        else:
            entries.append(entry)
        self._send({'cmd': 'schedule', 'entries': entries})

    def _delete_schedule_entry(self):
        sel = set(self.schedule_tree.selection())
        entries = [e for e in self.layout.get('schedule') or [] if e['id'] not in sel]
        self._send({'cmd': 'schedule', 'entries': entries})

    def _save_dnd(self):
        self._send({'cmd': 'settings', 'values': {'dnd': {
            'enabled': bool(self.dnd_var.get()), 'start': self.dnd_start_var.get(), 'end': self.dnd_end_var.get()}}})

    def _save_settings(self):
        try:
            values = {'charge_resume': bool(self.charge_resume_var.get()),
                      'auto_empty': bool(self.auto_empty_var.get()),
                      'low_battery_pct': float(self.low_var.get()),
                      'resume_battery_pct': float(self.resume_var.get()),
                      'battery_time_scale': max(0.1, float(self.scale_var.get())),
                      'speed_scale': max(0.3, min(1.5, float(self.speed_scale_var.get()))),
                      'carpet_boost': bool(self.carpet_boost_var.get()),
                      'auto_refill': bool(self.auto_refill_var.get())}
        except (ValueError, tk.TclError):
            self.message_var.set('Settings must be numbers')
            return
        self._send({'cmd': 'settings', 'values': values})

    def _history_selected(self, _event=None):
        sel = self.history_tree.selection()
        rec = next((r for r in self.history if sel and r['id'] == sel[0]), None)
        if rec:
            self.history_detail.set('%s · %s · suction %s · %d pass(es) · %.0f m driven · %d recharge(s) · '
                                    '%d obstacle event(s) · %d path(s) skipped · battery %.0f → %.0f%%' % (
                                        rec.get('result', ''), rec.get('map', ''), rec.get('suction', ''),
                                        rec.get('passes', 1), rec.get('distance_m', 0), rec.get('recharges', 0),
                                        rec.get('events', 0), rec.get('skipped', 0), rec.get('battery_start', 0),
                                        rec.get('battery_end', 0)))

    def _show_history(self):
        sel = self.history_tree.selection()
        if not sel:
            return
        try:
            from robot_lab_vacuum_cleaning.store import Store
            coverage = Store().load_history_coverage(sel[0])
        except ImportError:
            coverage = None
        if coverage is None or self.map_data is None or coverage.shape != self.map_data.shape:
            self.history_detail.set('That run\'s coverage is not available for this map on this computer.')
            return
        self.history_overlay = coverage
        self._mark_dirty()

    def _clear_history_overlay(self):
        if self.history_overlay is not None:
            self.history_overlay = None
            self._mark_dirty()

    # ------------------------------------------------------------------
    # Map drawing
    # ------------------------------------------------------------------
    def _mark_dirty(self):
        self.dirty_image = True

    def _fit_view(self):
        self.view_user = False
        self.view = None
        self._mark_dirty()

    def _auto_view(self):
        w, h, res, ox, oy = self.map_info
        known = self.map_data >= 0
        if not known.any():
            return (ox, oy, ox + w * res, oy + h * res)
        ys, xs = np.nonzero(known)
        pad = 0.6
        return (ox + xs.min() * res - pad, oy + ys.min() * res - pad,
                ox + (xs.max() + 1) * res + pad, oy + (ys.max() + 1) * res + pad)

    def transform(self):
        """(scale px/m, offset x, offset y, view) for world -> canvas."""
        if self.view is None:
            return None
        cw = max(10, self.canvas.winfo_width())
        ch = max(10, self.canvas.winfo_height())
        x0, y0, x1, y1 = self.view
        scale = min(cw / max(x1 - x0, 1e-3), ch / max(y1 - y0, 1e-3))
        offx = (cw - (x1 - x0) * scale) / 2.0
        offy = (ch - (y1 - y0) * scale) / 2.0
        return scale, offx, offy

    def to_canvas(self, x, y):
        scale, offx, offy = self.transform()
        x0, _, _, y1 = self.view
        return offx + (x - x0) * scale, offy + (y1 - y) * scale

    def to_world(self, cx, cy):
        scale, offx, offy = self.transform()
        x0, _, _, y1 = self.view
        return x0 + (cx - offx) / scale, y1 - (cy - offy) / scale

    def compose_rgb(self):
        """RGB image of the map with rooms, cleaned area and selection."""
        data = self.map_data
        h, w = data.shape
        rgb = np.empty((h, w, 3), dtype=np.uint8)
        rgb[:] = (13, 21, 32)
        free = (data >= 0) & (data < 25)
        rgb[free] = (58, 70, 88)
        rgb[data >= 65] = (200, 210, 225)
        rooms = self.rooms_grid if self.rooms_grid is not None and self.rooms_grid.shape == data.shape else None
        if rooms is not None:
            for rid in np.unique(rooms[rooms > 0]):
                colour = np.array(ROOM_COLOURS[(int(rid) - 1) % len(ROOM_COLOURS)], dtype=float)
                mask = (rooms == rid) & free
                selected = str(rid) in [str(r) for r in self.selected_rooms]
                alpha = 0.55 if selected else 0.22
                rgb[mask] = (rgb[mask] * (1 - alpha) + colour * alpha).astype(np.uint8)
        cov = self.history_overlay if self.history_overlay is not None else self.coverage_grid
        if cov is not None and cov.shape == data.shape:
            mask = cov & free
            rgb[mask] = (rgb[mask] * 0.35 + np.array((79, 191, 118)) * 0.65).astype(np.uint8)
        return rgb

    def _render(self):
        self.dirty_image = False
        self.last_image = time.monotonic()
        self.canvas.delete('all')
        if self.map_data is None or np is None:
            self.canvas.create_text(self.canvas.winfo_width() / 2, self.canvas.winfo_height() / 2,
                                    text='No map yet.\nLaunch the robot and the cleaning system in Setup.',
                                    fill='#96a9c2', justify='center')
            return
        if self.view is None or not self.view_user:
            self.view = self._auto_view()
        w, h, res, ox, oy = self.map_info
        rgb = self.compose_rgb()
        scale, offx, offy = self.transform()
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        # Sample the map at each canvas pixel (nearest cell).
        px = np.arange(max(1, cw))
        py = np.arange(max(1, ch))
        x0, _, _, y1 = self.view
        wx = x0 + (px + 0.5 - offx) / scale
        wy = y1 - (py + 0.5 - offy) / scale
        ix = np.floor((wx - ox) / res).astype(int)
        iy = np.floor((wy - oy) / res).astype(int)
        okx = (ix >= 0) & (ix < w)
        oky = (iy >= 0) & (iy < h)
        img = np.empty((len(py), len(px), 3), dtype=np.uint8)
        img[:] = (13, 21, 32)
        sub = rgb[np.clip(iy, 0, h - 1)][:, np.clip(ix, 0, w - 1)]
        valid = oky[:, None] & okx[None, :]
        img[valid] = sub[valid]
        header = b'P6\n%d %d\n255\n' % (img.shape[1], img.shape[0])
        self.photo = tk.PhotoImage(data=header + img.tobytes(), format='PPM')
        self.canvas.create_image(0, 0, image=self.photo, anchor='nw', tags=('map',))
        self._draw_overlays()

    def _draw_overlays(self):
        c = self.canvas
        c.delete('overlay')
        if self.view is None or self.map_data is None:
            return
        lay = self.layout
        for area in lay.get('carpets') or []:
            pts = [v for p in area['polygon'] for v in self.to_canvas(*p)]
            c.create_polygon(*pts, outline=CARPET_COLOUR, fill=CARPET_COLOUR, stipple='gray12', width=2,
                             dash=(3, 2), tags=('overlay',))
        for zone in lay.get('zones') or []:
            x0, y0, x1, y1 = zone['rect']
            a, b = self.to_canvas(x0, y0), self.to_canvas(x1, y1)
            c.create_rectangle(a[0], a[1], b[0], b[1], outline='#5aa9ff', width=2, dash=(6, 3), tags=('overlay',))
        for zone in lay.get('no_go') or []:
            pts = [v for p in zone['polygon'] for v in self.to_canvas(*p)]
            c.create_polygon(*pts, outline='#ff5c7a', fill='#ff5c7a', stipple='gray25', width=2, tags=('overlay',))
        for wall in lay.get('walls') or []:
            a, b = self.to_canvas(*wall['p1']), self.to_canvas(*wall['p2'])
            c.create_line(a[0], a[1], b[0], b[1], fill='#ff5c7a', width=4, tags=('overlay',))
        plan = self.plan or {}
        for line in plan.get('queued') or []:
            if len(line) >= 2:
                pts = [v for p in line for v in self.to_canvas(*p)]
                c.create_line(*pts, fill='#6b7d96', width=1, tags=('overlay',))
        active = plan.get('active') or []
        if len(active) >= 2:
            pts = [v for p in active for v in self.to_canvas(*p)]
            c.create_line(*pts, fill='#ff9f43', width=2, tags=('overlay',))
        for rid, meta in (lay.get('rooms') or {}).items():
            anchor = meta.get('anchor')
            if not anchor:
                continue
            x, y = self.to_canvas(*anchor)
            label = meta.get('name', rid)
            if rid in [str(r) for r in self.selected_rooms]:
                label = '%d · %s' % ([str(r) for r in self.selected_rooms].index(rid) + 1, label)
            c.create_text(x, y, text=label, fill='#e6edf7', font=('', 9, 'bold'), tags=('overlay',))
        if self.spot_point and self.mode_var.get() == 'spot':
            x, y = self.to_canvas(*self.spot_point)
            r = 0.8 * self.transform()[0]
            c.create_rectangle(x - r, y - r, x + r, y + r, outline='#5aa9ff', width=2, tags=('overlay',))
        dock = lay.get('dock') or (self.status or {}).get('dock')
        if dock:
            x, y = self.to_canvas(dock[0], dock[1])
            c.create_rectangle(x - 7, y - 7, x + 7, y + 7, fill='#ffe066', outline='#101823', tags=('overlay',))
            hx, hy = self.to_canvas(dock[0] + 0.3 * math.cos(dock[2]), dock[1] + 0.3 * math.sin(dock[2]))
            c.create_line(x, y, hx, hy, fill='#ffe066', width=2, arrow='last', tags=('overlay',))
        self._draw_robot()

    def _draw_robot(self):
        c = self.canvas
        c.delete('robot')
        pose = (self.status or {}).get('pose')
        if not pose or self.view is None or self.map_data is None:
            return
        x, y = self.to_canvas(pose[0], pose[1])
        radius = max(5.0, (self.layout.get('robot') or {}).get('radius', 0.165) * self.transform()[0])
        colour = '#72ded0' if self.status.get('state') not in ('error',) else '#f38ba8'
        c.create_oval(x - radius, y - radius, x + radius, y + radius, fill=colour, outline='#101823',
                      width=2, tags=('robot',))
        hx, hy = self.to_canvas(pose[0] + 0.25 * math.cos(pose[2]), pose[1] + 0.25 * math.sin(pose[2]))
        c.create_line(x, y, hx, hy, fill='#101823', width=3, tags=('robot',))
        if self.locate_until:
            for k in (2.0, 3.2):
                c.create_oval(x - k * radius, y - k * radius, x + k * radius, y + k * radius,
                              outline='#ffe066', width=2, tags=('robot',))

    # ------------------------------------------------------------------
    # Map interaction
    # ------------------------------------------------------------------
    def _tool_changed(self):
        tool = self.tool_var.get()
        self.tool_hint.set(dict((k, tip) for k, _, tip in TOOLS)[tool])
        mode = {'select': 'rooms', 'zone': 'zones', 'spot': 'spot'}.get(tool)
        if mode:
            self.mode_var.set(mode)
            self._update_selection_text()

    def room_at(self, x, y):
        if self.rooms_grid is None or self.map_info is None:
            return 0
        w, h, res, ox, oy = self.map_info
        ix, iy = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
        if 0 <= ix < w and 0 <= iy < h and self.rooms_grid.shape == (h, w):
            return int(self.rooms_grid[iy, ix])
        return 0

    def _press(self, event):
        if self.view is None:
            return
        self.drag = {'start': (event.x, event.y), 'world': self.to_world(event.x, event.y)}

    def _motion(self, event):
        if not self.drag or self.view is None:
            return
        self.canvas.delete('drag')
        sx, sy = self.drag['start']
        tool = self.tool_var.get()
        if tool in ('zone', 'nogo', 'carpet'):
            colour = {'zone': '#5aa9ff', 'nogo': '#ff5c7a', 'carpet': CARPET_COLOUR}[tool]
            self.canvas.create_rectangle(sx, sy, event.x, event.y, outline=colour, width=2, dash=(4, 2), tags=('drag',))
        elif tool in ('wall', 'split', 'dock'):
            self.canvas.create_line(sx, sy, event.x, event.y, fill='#ff5c7a' if tool != 'dock' else '#ffe066',
                                    width=3, arrow='last' if tool == 'dock' else None, tags=('drag',))

    def _release(self, event):
        if not self.drag or self.view is None:
            return
        self.canvas.delete('drag')
        start = self.drag['world']
        end = self.to_world(event.x, event.y)
        moved = math.hypot(event.x - self.drag['start'][0], event.y - self.drag['start'][1]) > 6
        self.drag = None
        self.apply_tool(self.tool_var.get(), start, end, moved)

    def apply_tool(self, tool, start, end, moved=True):
        """Apply a map tool between world points start and end."""
        lay = self.layout
        if tool == 'select':
            rid = self.room_at(*start)
            if rid:
                if rid in self.selected_rooms:
                    self.selected_rooms.remove(rid)
                else:
                    self.selected_rooms.append(rid)
                self.mode_var.set('rooms')
            self._update_selection_text()
            self._mark_dirty()
        elif tool in ('zone', 'nogo', 'carpet') and moved:
            x0, x1 = sorted((start[0], end[0]))
            y0, y1 = sorted((start[1], end[1]))
            if (x1 - x0) < 0.3 or (y1 - y0) < 0.3:
                self.message_var.set('Draw a larger rectangle (at least 0.3 m).')
                return
            if tool == 'zone':
                zones = list(lay.get('zones') or [])
                if len(zones) >= 8:
                    self.message_var.set('At most 8 zones.')
                    return
                zones.append({'id': 'z%d' % int(time.time() * 1000 % 1e8), 'rect': [x0, y0, x1, y1],
                              'passes': int(self.passes_var.get() or 1)})
                self._send_layout(zones=zones)
                self.mode_var.set('zones')
            elif tool == 'carpet':
                carpets = list(lay.get('carpets') or [])
                carpets.append({'id': 'c%d' % int(time.time() * 1000 % 1e8),
                                'polygon': [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]})
                self._send_layout(carpets=carpets)
            else:
                nogo = list(lay.get('no_go') or [])
                nogo.append({'id': 'n%d' % int(time.time() * 1000 % 1e8),
                             'polygon': [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]})
                self._send_layout(no_go=nogo)
        elif tool == 'wall' and moved:
            walls = list(lay.get('walls') or [])
            walls.append({'id': 'w%d' % int(time.time() * 1000 % 1e8), 'p1': list(start), 'p2': list(end)})
            self._send_layout(walls=walls)
        elif tool == 'spot':
            self.spot_point = [round(start[0], 2), round(start[1], 2)]
            self.mode_var.set('spot')
            self._update_selection_text()
            self._draw_overlays()
        elif tool == 'dock':
            yaw = math.atan2(end[1] - start[1], end[0] - start[0]) if moved else \
                ((self.status or {}).get('pose') or [0, 0, 0])[2]
            self._send({'cmd': 'set_dock', 'pose': [start[0], start[1], yaw]})
        elif tool == 'split' and moved:
            mid = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            rid = self.room_at(*mid) or self.room_at(*start)
            if not rid:
                self.message_var.set('Draw the split line across a room.')
                return
            self.selected_rooms = []
            self._send({'cmd': 'split_room', 'room': rid, 'line': [list(start), list(end)]})
        elif tool == 'erase':
            self._erase_at(start)
        self._update_selection_text()

    def _erase_at(self, point):
        lay = self.layout
        x, y = point
        zones = [z for z in lay.get('zones') or []
                 if not (z['rect'][0] <= x <= z['rect'][2] and z['rect'][1] <= y <= z['rect'][3])]
        def outside(area):
            poly = area['polygon']
            return not (min(p[0] for p in poly) <= x <= max(p[0] for p in poly)
                        and min(p[1] for p in poly) <= y <= max(p[1] for p in poly))
        nogo = [z for z in lay.get('no_go') or [] if outside(z)]
        carpets = [z for z in lay.get('carpets') or [] if outside(z)]

        def near(wall):
            (x1, y1), (x2, y2) = wall['p1'], wall['p2']
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / max(dx * dx + dy * dy, 1e-9)))
            return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)) < 0.25
        walls = [w for w in lay.get('walls') or [] if not near(w)]
        if (len(zones), len(nogo), len(walls), len(carpets)) != (
                len(lay.get('zones') or []), len(lay.get('no_go') or []), len(lay.get('walls') or []),
                len(lay.get('carpets') or [])):
            self._send_layout(zones=zones, no_go=nogo, walls=walls, carpets=carpets)
        else:
            self.message_var.set('Nothing to erase there.')

    def _send_layout(self, **changes):
        # Show the edit at once; the node's latched layout confirms it.
        self.layout = dict(self.layout, **changes)
        self._draw_overlays()
        self._send(dict({'cmd': 'set_layout'}, **changes))
        self._update_selection_text()

    def _wheel(self, event):
        self._zoom(event, 1.2 if event.delta > 0 else 1 / 1.2)

    def _zoom(self, event, factor):
        if self.view is None:
            return
        wx, wy = self.to_world(event.x, event.y)
        x0, y0, x1, y1 = self.view
        self.view = (wx - (wx - x0) / factor, wy - (wy - y0) / factor,
                     wx + (x1 - wx) / factor, wy + (y1 - wy) / factor)
        self.view_user = True
        self._render()

    def _pan_start(self, event):
        self._pan = (event.x, event.y, self.view)

    def _pan_move(self, event):
        if self.view is None or not getattr(self, '_pan', None):
            return
        sx, sy, view = self._pan
        scale = self.transform()[0]
        dx, dy = (event.x - sx) / scale, (event.y - sy) / scale
        self.view = (view[0] - dx, view[1] + dy, view[2] - dx, view[3] + dy)
        self.view_user = True
        self._render()
