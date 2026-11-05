"""Shared workspace layout; controls retain the launcher's existing callbacks."""
import tkinter as tk
from tkinter import ttk


class ScrollPanel(ttk.Frame):
    def __init__(self, parent, width=320):
        super().__init__(parent)
        self.columnconfigure(0, weight=1); self.rowconfigure(0, weight=1)
        from .themes.modern import BG_DARK
        canvas = self.canvas = tk.Canvas(self, width=width, highlightthickness=0, background=BG_DARK)
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
    from .experience import install_context, open_search, help_for_workspace
    install_context(app)
    header = ttk.Frame(app, padding=(14, 10))
    header.grid(row=0, column=0, columnspan=2, sticky='ew')
    header.columnconfigure(3, weight=1)
    app.navigation_toggle=ttk.Button(header,text='≡',width=3,style='Ghost.TButton',command=lambda:getattr(app,'toggle_navigation',lambda:None)())
    app.navigation_toggle.grid(row=0,column=0,padx=(0,8))
    ttk.Button(header, text='ROBOT LAB', style='Brand.TButton',command=lambda:app.show_tab('Home')).grid(row=0, column=1,padx=(0,16))
    # The page heading names the workspace; the header shows the session.
    app.workspace_title = tk.StringVar(value='Launch · configure, run and control')
    app.session_pill = ttk.Label(header, textvariable=app.session_state, style='PillIdle.TLabel')
    app.session_pill.grid(row=0, column=2, sticky='w', padx=(0, 12))
    app.session_pill.bind('<Button-1>', lambda _e: app.show_tab('Launch'))
    app.header_context = ttk.Label(header, textvariable=app.selection_context, style='Muted.TLabel')
    app.header_context.grid(row=0, column=3, sticky='w')
    ttk.Button(header,text='Find  Ctrl+K',command=lambda:open_search(app),style='Ghost.TButton').grid(row=0,column=4,padx=5)
    ttk.Button(header,text='Help  F1',command=lambda:help_for_workspace(app),style='Ghost.TButton').grid(row=0,column=5,padx=5)
    ttk.Button(header, text='Logs', command=app._toggle_console, style='Ghost.TButton').grid(row=0, column=6,padx=5)
    ttk.Button(header, text='Stop motion', command=app._stop_all_motion, style='Danger.TButton').grid(row=0, column=7,padx=(6,0))
    app.bind('<Control-k>',lambda event:open_search(app,event))
    app.bind('<F1>',lambda _e:help_for_workspace(app))
    def wrap_header(event):
        if event.widget is header:
            app.header_context.configure(wraplength=max(120,event.width-760))
    header.bind('<Configure>',wrap_header)
    _watch_session(app)
    app.navigation = ScrollPanel(app, width=218)
    app.navigation.body.configure(padding=(6,4))
    app.navigation.grid(row=1, column=0, sticky='ns')
    main=ttk.Frame(app,padding=(4,0,10,0));main.grid(row=1,column=1,sticky='nsew')
    main.columnconfigure(0,weight=1);main.rowconfigure(1,weight=1)
    app.page_heading=ttk.Frame(main,padding=(10,6,4,12));app.page_heading.grid(row=0,column=0,sticky='ew')
    app.page_heading.columnconfigure(0,weight=1)
    app.page_title=tk.StringVar(value='Run a robot');app.page_description=tk.StringVar(value='Configure, review and launch your experiment.')
    ttk.Label(app.page_heading,textvariable=app.page_title,style='HeroSmall.TLabel').grid(row=0,column=0,sticky='w')
    description=ttk.Label(app.page_heading,textvariable=app.page_description,style='Muted.TLabel',wraplength=850)
    description.grid(row=1,column=0,sticky='ew',pady=(4,0))
    app.page_heading.bind('<Configure>',lambda e:description.configure(wraplength=max(300,e.width-30)))
    app.notebook = ttk.Notebook(main, style='Workspace.TNotebook')
    app.notebook.grid(row=1, column=0, sticky='nsew')
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
    app.session_frame = ttk.LabelFrame(host, text='2 · Review & launch', padding=8)
    lower = ttk.LabelFrame(app.launch_split, text='3 · Control & observe', padding=8)
    lower.columnconfigure(0, weight=1); lower.rowconfigure(2, weight=1)
    app.launch_split.add(lower, weight=1)
    app.control_context = tk.StringVar(value='Select a robot to see its control support.')
    ttk.Label(lower, textvariable=app.control_context, style='Muted.TLabel', wraplength=390).grid(row=0, column=0, sticky='w', pady=(0, 5))
    control_bar=ttk.Frame(lower);control_bar.grid(row=1,column=0,sticky='ew',pady=(0,8));control_bar.columnconfigure(0,weight=1)
    app.control_page_var=tk.StringVar(value='Drive & limits')
    app.control_page_combo=ttk.Combobox(control_bar,textvariable=app.control_page_var,state='readonly',width=18)
    app.control_page_combo.grid(row=0,column=0,sticky='ew')
    ttk.Button(control_bar,text='Telemetry',command=lambda:app.show_tab('Live Monitor'),style='Ghost.TButton').grid(row=0,column=1,padx=(5,0))
    app.control_notebook = ttk.Notebook(lower,style='Control.TNotebook')
    app.control_notebook.grid(row=2, column=0, sticky='nsew')
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


def session_text(app, now=None):
    """(text, style) for the header session indicator; reads state only."""
    import time
    process = getattr(app, 'process', None)
    running = bool(getattr(app, '_launch_running', False)) or (process is not None and process.poll() is None)
    if running:
        started = getattr(app, '_session_started', None)
        elapsed = int((now if now is not None else time.monotonic()) - started) if started else 0
        parts = [app.robot_var.get(), app.map_var.get(), app.simulator_var.get()]
        return ('●  Running · %s · %02d:%02d' % (' · '.join(p for p in parts if p), elapsed // 60, elapsed % 60),
                'PillRunning.TLabel')
    status = app.status_var.get() if hasattr(app, 'status_var') else ''
    if status.casefold().startswith(('launch blocked', 'error', 'failed')):
        return '▲  ' + status[:60], 'PillWarn.TLabel'
    return '○  No session running', 'PillIdle.TLabel'


def _watch_session(app):
    import time
    app.session_state = getattr(app, 'session_state', None) or tk.StringVar()
    def tick():
        try:
            process = getattr(app, 'process', None)
            running = bool(getattr(app, '_launch_running', False)) or (process is not None and process.poll() is None)
            if running and getattr(app, '_session_started', None) is None:
                app._session_started = time.monotonic()
            elif not running:
                app._session_started = None
            text, style = session_text(app)
            app.session_state.set(text)
            app.session_pill.configure(style=style)
            button = getattr(app, 'nav_buttons', {}).get('Launch')
            if button is not None and not getattr(app, 'navigation_collapsed', False):
                label = next((row[1] for row in getattr(app, 'nav_rows', ()) if row[0] is button), None)
                if label:
                    button.configure(text=label + ('   ●' if running else ''))
            app.after(1000, tick)
        except tk.TclError:
            pass
    app.after(300, tick)


def build_navigation(app):
    from .experience import WORKSPACES, HomeTab
    from .themes.modern import tooltip
    # Optional control-center tabs may be suppressed in lightweight launchers.
    # Add the dashboard when the complete workspace is available.
    if hasattr(app, 'health_tab'):
        app.home_tab = HomeTab(app.notebook, app)
    titles = {app.notebook.tab(tab, 'text') for tab in app.notebook.tabs()}
    app.nav_buttons = {}; headings = []; rows = []
    host = app.navigation.body
    host.columnconfigure(0, weight=1)
    group = None; index = 0
    for title, label, section, icon, keywords in WORKSPACES:
        if title not in titles:
            continue
        if section != group:
            heading = ttk.Label(host, text=section.upper(), style='Muted.TLabel')
            heading.grid(row=index, column=0, sticky='w', pady=(16, 6), padx=10)
            headings.append(heading); index += 1; group = section
        shortcut = len(rows) + 1
        text = '%s   %s' % (icon, label)
        button = ttk.Button(host, text=text, width=23, style='Nav.TButton', command=lambda page=title: app.show_tab(page))
        button.grid(row=index, column=0, sticky='ew', pady=1)
        tooltip(button, label + ' · ' + keywords + ('  (Ctrl+%d)' % shortcut if shortcut <= 9 else ''))
        if shortcut <= 9:
            app.bind('<Control-Key-%d>' % shortcut, lambda _e, page=title: app.show_tab(page))
        app.nav_buttons[title] = button; rows.append((button, text, icon)); index += 1
    app.nav_rows = rows
    app.navigation_collapsed = False
    app.navigation_override = None
    def collapse(value):
        app.navigation_collapsed = value
        app.navigation.canvas.configure(width=60 if value else 218)
        for button, label, icon in rows:
            button.configure(text=icon if value else label, width=3 if value else 23)
        for heading in headings:
            heading.grid_remove() if value else heading.grid()
        app.navigation_toggle.configure(text='»' if value else '≡')
    def toggle():
        app.navigation_override = not app.navigation_collapsed
        collapse(app.navigation_override)
    app.toggle_navigation = toggle
    tooltip(app.navigation_toggle, 'Expand or collapse workspace navigation')
    def responsive(event):
        if event.widget is app and app.navigation_override is None:
            desired = event.width < 1250
            if desired != app.navigation_collapsed:
                collapse(desired)
    app.bind('<Configure>', responsive, add='+')
    def refresh(_event=None):
        current = app.notebook.select()
        if not current: return
        title = app.notebook.tab(current, 'text')
        label = next((label for key,label,group,icon,keywords in WORKSPACES if key==title), title)
        app.workspace_title.set(label)
        descriptions={'Launch':'Choose a robot, world and task. Review the command, then use the control column.',
            'Registry':'Explore complete families, compare variants and inspect source geometry in 3D.',
            'Worlds':'Create occupancy maps from installed worlds with explicit resolution and slice settings.',
            'Fleet & Swarm':'Build a shared-world ground session. Connect and give control explicitly when ready.',
            'Aerial Swarm':'Configure independently identified aircraft in one world. Flight requires explicit controls.',
            'Live Monitor':'Connect deliberately to inspect actual robot measurements and simulation time.',
            'Installed Extensions':'Installed source profiles use the same launch selectors and capability gates.',
            'Benchmark':'Configure real workloads, inspect artifacts and compare scoped outcomes.',
            'Training':'Train and export a model-specific policy, then evaluate it through the deployment workflow.',
            'Tests':'Run focused source and integration checks. Tests do not replace robot mission evidence.',
            'Health':'Learn the architecture, browse tutorials and source, or inspect diagnostics and status.',
            'Vacuum':'Configure and monitor a compatible cleaning or coverage workflow.'}
        app.page_title.set(label);app.page_description.set(descriptions.get(title,'Robot Lab workspace'))
        app.page_heading.grid_remove() if title=='Home' else app.page_heading.grid()
        for name, button in app.nav_buttons.items():
            button.configure(style='NavActive.TButton' if name == title else 'Nav.TButton')
    app.notebook.bind('<<NotebookTabChanged>>', refresh, add='+')
    def refresh_controls(_event=None):
        values = [app.control_notebook.tab(page, 'text') for page in app.control_notebook.tabs()]
        app.control_page_combo.configure(values=values)
        selected = app.control_notebook.select()
        if selected: app.control_page_var.set(app.control_notebook.tab(selected, 'text'))
    def choose_control(_event=None):
        for page in app.control_notebook.tabs():
            if app.control_notebook.tab(page, 'text') == app.control_page_var.get():
                app.control_notebook.select(page); break
    app.control_page_combo.bind('<<ComboboxSelected>>', choose_control)
    app.control_notebook.bind('<<NotebookTabChanged>>', refresh_controls, add='+')
    refresh_controls()
    if hasattr(app, 'home_tab'): app.notebook.select(app.home_tab)
    refresh()
