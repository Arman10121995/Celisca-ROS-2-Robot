"""Shared workspace layout; controls retain the launcher's existing callbacks."""
import tkinter as tk
from tkinter import ttk


class ScrollPanel(ttk.Frame):
    def __init__(self, parent, width=320):
        super().__init__(parent)
        self.columnconfigure(0, weight=1); self.rowconfigure(0, weight=1)
        canvas = self.canvas = tk.Canvas(self, width=width, highlightthickness=0, background='#171d29')
        canvas.grid(row=0, column=0, sticky='nsew')
        bar = ttk.Scrollbar(self, orient='vertical', command=canvas.yview)
        bar.grid(row=0, column=1, sticky='ns'); canvas.configure(yscrollcommand=bar.set)
        horizontal = ttk.Scrollbar(self, orient='horizontal', command=canvas.xview)
        horizontal.grid(row=1, column=0, sticky='ew')
        canvas.configure(xscrollcommand=horizontal.set)
        self.body = ttk.Frame(canvas, padding=10)
        window = canvas.create_window((0, 0), window=self.body, anchor='nw')
        tag = 'RobotLabScroll'+str(id(self))
        def scroll(units):
            canvas.yview_scroll(units, 'units')
            return 'break'
        self.bind_class(tag, '<Button-4>', lambda _e: scroll(-1))
        self.bind_class(tag, '<Button-5>', lambda _e: scroll(1))
        self.bind_class(tag, '<MouseWheel>', lambda e: scroll(-int(e.delta/120)))
        def descendants(widget):
            # Editable and independently scrolling widgets keep their own wheel
            # behavior; labels/buttons scroll the panel under the pointer.
            if widget.winfo_class() not in ('Text', 'Treeview', 'Listbox', 'TCombobox', 'TSpinbox'):
                tags = widget.bindtags()
                if tag not in tags:
                    widget.bindtags((tags[0], tag, *tags[1:]))
            for child in widget.winfo_children():
                descendants(child)
        def resize(_event=None):
            requested = self.body.winfo_reqwidth()
            available = canvas.winfo_width()
            canvas.itemconfigure(window, width=max(requested, available))
            canvas.configure(scrollregion=canvas.bbox('all'))
            horizontal.grid() if requested > available else horizontal.grid_remove()
            descendants(self.body)
        self.body.bind('<Configure>', resize)
        canvas.bind('<Configure>', resize)
        canvas.bindtags((str(canvas), tag, *canvas.bindtags()[1:]))


def build_workspace(app):
    """Create setup beside a dedicated control column and workspace navigation."""
    app.columnconfigure(0, weight=0); app.columnconfigure(1, weight=1)
    app.rowconfigure(0, weight=0); app.rowconfigure(1, weight=1)
    header = ttk.Frame(app, padding=(16, 10))
    header.grid(row=0, column=0, columnspan=2, sticky='ew')
    header.columnconfigure(1, weight=1)
    ttk.Label(header, text='ROBOT LAB', style='Title.TLabel').grid(row=0, column=0, padx=(0, 22))
    app.workspace_title = tk.StringVar(value='Launch · configure, run and control')
    ttk.Label(header, textvariable=app.workspace_title, style='Muted.TLabel').grid(row=0, column=1, sticky='w')
    ttk.Button(header, text='Logs', command=app._toggle_console, style='Ghost.TButton').grid(row=0, column=2, padx=6)
    ttk.Button(header, text='Stop motion', command=app._stop_all_motion, style='Danger.TButton').grid(row=0, column=3)
    app.navigation = ttk.Frame(app, padding=(10, 6))
    app.navigation.grid(row=1, column=0, sticky='ns')
    app.notebook = ttk.Notebook(app, style='Workspace.TNotebook')
    app.notebook.grid(row=1, column=1, sticky='nsew')
    app.launch_tab = ttk.Frame(app.notebook, padding=(8, 6))
    app.notebook.add(app.launch_tab, text='Launch')
    app.launch_tab.columnconfigure(0, weight=1); app.launch_tab.rowconfigure(0, weight=1)
    app.launch_split = ttk.Panedwindow(app.launch_tab, orient='horizontal')
    app.launch_split.grid(row=0, column=0, sticky='nsew')
    host = ttk.Frame(app.launch_split)
    host.rowconfigure(1, weight=1)
    app.launch_split.add(host, weight=1)
    compact = app.compact_setup = ttk.Frame(host)
    setup = ttk.LabelFrame(host, text='1 · Set up your experiment', padding=8)
    setup.columnconfigure(0, weight=1); setup.rowconfigure(0, weight=1)
    app.setup_notebook = ttk.Notebook(setup)
    app.setup_notebook.grid(row=0, column=0, sticky='nsew')
    basic, advanced = ScrollPanel(app.setup_notebook), ScrollPanel(app.setup_notebook)
    app.setup_notebook.add(basic, text='Robot & world')
    app.setup_notebook.add(advanced, text='Options')
    app.session_frame = ttk.Frame(host, padding=(12, 0, 4, 0))
    lower = ttk.LabelFrame(app.launch_split, text='2 · Robot controls and limits', padding=8)
    lower.columnconfigure(0, weight=1); lower.rowconfigure(1, weight=1)
    app.launch_split.add(lower, weight=1)
    app.control_context = tk.StringVar(value='Select a robot to see its control support.')
    ttk.Label(lower, textvariable=app.control_context, style='Muted.TLabel', wraplength=390).grid(row=0, column=0, sticky='w', pady=(0, 5))
    app.control_notebook = ttk.Notebook(lower)
    app.control_notebook.grid(row=1, column=0, sticky='nsew')
    app.drive_page = ScrollPanel(app.control_notebook, width=390)
    app.drone_page = ScrollPanel(app.control_notebook, width=390)
    app.control_notebook.add(app.drive_page, text='Drive & limits')
    # Arm and Hand are added after their real controllers are instantiated.
    app.control_notebook.add(app.drone_page, text='Drone & limits')
    app.drive_page.columnconfigure(0, weight=1)
    app.drone_page.columnconfigure(0, weight=1)
    app.drive_pad_host = ttk.LabelFrame(app.drive_page.body, text='Motion', padding=8)
    app.drive_pad_host.grid(row=0, column=0, sticky='new', pady=(0, 10))
    app.drive_limits_host = ttk.LabelFrame(app.drive_page.body, text='Velocity increments and limits', padding=8)
    app.drive_limits_host.grid(row=1, column=0, sticky='new')
    app.drone_actions_host = ttk.LabelFrame(app.drone_page.body, text='Flight', padding=12)
    app.drone_actions_host.grid(row=0, column=0, sticky='new', pady=(0, 10))
    app.drone_limits_host = ttk.LabelFrame(app.drone_page.body, text='Manual flight limits', padding=12)
    app.drone_limits_host.grid(row=1, column=0, sticky='new')
    # One geometry manager owns setup/session through every resize. Swapping
    # these windows between a Notebook and Panedwindow can leave an inactive
    # notebook page unmapped. Grid retains the actual widgets and state.
    previous_width = 0
    narrow_mode = False
    def select_compact(page):
        if not narrow_mode:
            return
        selected, hidden = (setup, app.session_frame) if page == 'setup' else (app.session_frame, setup)
        hidden.grid_remove()
        selected.grid(row=1, column=0, columnspan=2, sticky='nsew')
        compact.setup_button.configure(style='NavActive.TButton' if page == 'setup' else 'Nav.TButton')
        compact.command_button.configure(style='NavActive.TButton' if page == 'command' else 'Nav.TButton')
    compact.setup_button = ttk.Button(compact, text='Set up', command=lambda: select_compact('setup'))
    compact.command_button = ttk.Button(compact, text='Command & algorithms', command=lambda: select_compact('command'))
    compact.setup_button.grid(row=0, column=0, sticky='ew')
    compact.command_button.grid(row=0, column=1, sticky='ew')
    compact.columnconfigure(0, weight=1); compact.columnconfigure(1, weight=1)
    def split(_event=None):
        nonlocal previous_width, narrow_mode
        width = app.launch_split.winfo_width()
        if width < 100 or width == previous_width:
            return
        previous_width = width
        narrow = width < 1100
        if narrow:
            host.columnconfigure(0, weight=1, minsize=0)
            host.columnconfigure(1, weight=0, minsize=0)
            compact.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 5))
            if not narrow_mode:
                narrow_mode = True
                select_compact('command')
        else:
            narrow_mode = False
            compact.grid_remove()
            host.columnconfigure(0, weight=0, minsize=380)
            host.columnconfigure(1, weight=1, minsize=0)
            setup.grid(row=1, column=0, columnspan=1, sticky='nsew')
            app.session_frame.grid(row=1, column=1, columnspan=1, sticky='nsew')
        app.launch_split.sashpos(0, max(385, width-(415 if narrow else 445)))
    app.launch_split.bind('<Configure>', split, add='+')
    app.after(150, split)
    return basic.body, advanced.body


def build_navigation(app):
    groups = [('WORKSPACE', [('Launch', 'Launch & control'), ('Live Monitor', 'Live telemetry')]),
              ('ASSETS', [('Registry', 'Registry · 3D'), ('Worlds', 'Worlds & maps'), ('Installed Extensions', 'Installed assets')]),
              ('EXPERIMENTS', [('Vacuum', 'Vacuum missions'), ('Benchmark', 'Benchmarks'), ('Tests', 'Test suites')]),
              ('PROJECT', [('Health', 'Health & roadmap')])]
    titles = {app.notebook.tab(tab, 'text') for tab in app.notebook.tabs()}
    app.nav_buttons = {}
    row = 0
    for heading, items in groups:
        ttk.Label(app.navigation, text=heading, style='Muted.TLabel').grid(row=row, column=0, sticky='w', pady=(12, 5), padx=6)
        row += 1
        for title, label in items:
            if title not in titles:
                continue
            button = ttk.Button(app.navigation, text=label, width=19, style='Nav.TButton', command=lambda value=title: app.show_tab(value))
            button.grid(row=row, column=0, sticky='ew', pady=2)
            app.nav_buttons[title] = button; row += 1
    descriptions = dict(Launch='Configure an experiment, then run and control it',
        Registry='Browse complete models and inspect source geometry in 3D',
        Worlds='Inspect worlds and generate occupancy maps',
        Health='Current status, verified work and remaining roadmap',
        **{'Installed Extensions': 'Installed robot and world sources', 'Live Monitor': 'Measured robot telemetry'})
    def refresh(_event=None):
        title = app.notebook.tab(app.notebook.select(), 'text')
        app.workspace_title.set(title+' · '+descriptions.get(title, 'Robot Lab workspace'))
        for name, button in app.nav_buttons.items():
            button.configure(style='NavActive.TButton' if name == title else 'Nav.TButton')
    app.notebook.bind('<<NotebookTabChanged>>', refresh, add='+')
    refresh()
