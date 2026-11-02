"""Neutral workspace navigation, onboarding and session presentation.

This module navigates existing pages; it does not arm input, publish commands,
change capability gates or launch a simulation.
"""
import tkinter as tk
from tkinter import ttk

from .workspace_ui import ScrollPanel


WORKSPACES = [
    ('Home', 'Home', 'Start here', 'H', 'overview welcome getting started'),
    ('Launch', 'Run a robot', 'Operate', 'R', 'configure simulator launch drive'),
    ('Fleet & Swarm', 'Ground fleet', 'Operate', 'F', 'teams multi robot rmf lift'),
    ('Aerial Swarm', 'Aerial swarm', 'Operate', 'A', 'drone px4 flight formation'),
    ('Live Monitor', 'Live telemetry', 'Operate', 'M', 'sensors odometry clock monitor'),
    ('Registry', 'Robot & world registry', 'Explore', '3D', 'models variants preview assets'),
    ('Worlds', 'Worlds & mapping', 'Explore', 'W', 'maps occupancy generation terrain'),
    ('Installed Extensions', 'Installed assets', 'Explore', 'I', 'sources robots worlds extensions'),
    ('Vacuum', 'Cleaning missions', 'Experiment', 'V', 'vacuum coverage'),
    ('Benchmark', 'Experiments', 'Experiment', 'B', 'benchmark queue results compare'),
    ('Training', 'Policy training', 'Experiment', 'T', 'reinforcement learning legged'),
    ('Tests', 'Test suites', 'Experiment', 'Q', 'validation regressions'),
    ('Health', 'Learn & diagnose', 'Understand', '?', 'architecture tutorials glossary network roadmap status health'),
]


def workspace_actions(app):
    titles={app.notebook.tab(tab,'text') for tab in app.notebook.tabs()}
    actions=[dict(title=label,detail=group+' · '+keywords,target=title,kind='workspace')
             for title,label,group,icon,keywords in WORKSPACES if title in titles]
    aliases={'Drive & limits':'Drive','Drone & limits':'Drone'}
    for page in app.control_notebook.tabs():
        title=app.control_notebook.tab(page,'text')
        actions.append(dict(title=title+' controls',detail='Open control page · navigation only',target=aliases.get(title,title),kind='control'))
    return actions


def matching_actions(actions,query):
    words=query.casefold().split()
    matches=[row for row in actions if all(word in (row['title']+' '+row['detail']).casefold() for word in words)]
    if not words:return matches
    def rank(row):
        title=row['title'].casefold()
        return 0 if title.startswith(' '.join(words)) else 1 if all(word in title for word in words) else 2
    return sorted(matches,key=rank)


class WorkspaceSearch(tk.Toplevel):
    def __init__(self,app):
        super().__init__(app);self.app=app;self.title('Find a workspace');self.transient(app)
        self.geometry('620x460');self.minsize(400,300);self.configure(bg='#111b2a')
        self.columnconfigure(0,weight=1);self.rowconfigure(2,weight=1)
        ttk.Label(self,text='Where would you like to go?',style='HeroSmall.TLabel').grid(row=0,column=0,sticky='w',padx=20,pady=(20,10))
        self.query=tk.StringVar();self.entry=ttk.Entry(self,textvariable=self.query)
        self.entry.grid(row=1,column=0,sticky='ew',padx=20,pady=(0,12))
        self.tree=ttk.Treeview(self,columns=('context',),show='tree headings',selectmode='browse',takefocus=False)
        self.tree.heading('#0',text='Workspace or controls');self.tree.heading('context',text='Purpose')
        self.tree.column('#0',width=240);self.tree.column('context',width=280)
        self.tree.grid(row=2,column=0,sticky='nsew',padx=20)
        ttk.Label(self,text='↑ ↓ choose    Enter open    Esc close · Navigation never starts a robot.',style='Muted.TLabel',wraplength=570).grid(row=3,column=0,sticky='w',padx=20,pady=14)
        self.actions=workspace_actions(app);self.rows=[];self.query.trace_add('write',lambda *_:self.refresh())
        self.entry.bind('<Down>',lambda e:self.move(1));self.entry.bind('<Up>',lambda e:self.move(-1))
        self.entry.bind('<Return>',self.activate)
        self.tree.bind('<Button-1>',self.choose);self.tree.bind('<Double-1>',self.activate)
        self.bind('<Escape>',lambda e:self.dismiss());self.bind('<Control-k>',lambda e:self.dismiss())
        # Widget/class bindings still edit text; stop key presses before the
        # application's bind_all drive/launch handlers. Releases remain visible
        # so an already-held movement key can be released normally.
        self.bind('<KeyPress>',lambda e:'break')
        self.refresh();self.after_idle(self.entry.focus_set)

    def refresh(self):
        self.tree.delete(*self.tree.get_children());self.rows=matching_actions(self.actions,self.query.get())
        for i,row in enumerate(self.rows):self.tree.insert('','end',iid=str(i),text=row['title'],values=(row['detail'],))
        if self.rows:self.tree.selection_set('0')

    def move(self,direction):
        if self.rows:
            current=int(self.tree.selection()[0]) if self.tree.selection() else 0
            index=max(0,min(len(self.rows)-1,current+direction));self.tree.selection_set(str(index));self.tree.see(str(index))
        return 'break'

    def choose(self,event):
        row=self.tree.identify_row(event.y)
        if row:self.tree.selection_set(row)
        self.entry.focus_set();return 'break'

    def activate(self,event=None):
        selected=self.tree.selection()
        if selected:
            target=self.rows[int(selected[0])]['target'];self.destroy();self.app.show_tab(target)
        return 'break'

    def dismiss(self):
        self.destroy();return 'break'


def open_search(app,event=None):
    current=getattr(app,'workspace_search',None)
    if current is not None and current.winfo_exists():current.lift();current.entry.focus_set()
    else:app.workspace_search=WorkspaceSearch(app)
    return 'break'


def help_for_workspace(app):
    if not hasattr(app,'health_tab'):return
    title=app.notebook.tab(app.notebook.select(),'text')
    paths={'Launch':'tutorials/gui-workspace.md','Fleet & Swarm':'tutorials/fleet-and-swarm.md',
           'Aerial Swarm':'tutorials/aerial-swarm.md','Worlds':'ASSET_EXTENSION_GUIDE.md',
           'Training':'tutorials/legged-policy-training.md'}
    from .lab_tabs import WORKSPACE_ROOT
    app.show_tab('Health');app.health_tab._show_document(WORKSPACE_ROOT/'docs'/paths.get(title,'architecture/interactive-guide.md'))


def open_learning(app):
    app.show_tab('Health')
    if hasattr(app,'health_tab'):app.health_tab._show_architecture('overview')


class HomeTab(ScrollPanel):
    def __init__(self,notebook,app):
        super().__init__(notebook,width=1000);self.app=app
        notebook.add(self,text='Home');notebook.insert(0,self)
        body=self.body;body.columnconfigure(0,weight=1)
        hero=ttk.Frame(body,style='Surface.TFrame',padding=24);hero.grid(row=0,column=0,sticky='ew',pady=(0,16));hero.columnconfigure(0,weight=1)
        ttk.Label(hero,text='YOUR ROBOTICS WORKSPACE',style='Eyebrow.TLabel').grid(row=0,column=0,sticky='w')
        ttk.Label(hero,text='Explore. Configure. Run.',style='Hero.TLabel').grid(row=1,column=0,sticky='w',pady=(8,8))
        self.intro=ttk.Label(hero,text='One place to work with robots, worlds and algorithms. Start with a single robot, then explore mapping, manipulation and shared-world teams.',style='SurfaceBody.TLabel',wraplength=900)
        self.intro.grid(row=2,column=0,sticky='ew')
        actions=ttk.Frame(hero,style='Surface.TFrame');actions.grid(row=3,column=0,sticky='w',pady=(18,0))
        ttk.Button(actions,text='Configure a robot  →',style='Accent.TButton',command=lambda:app.show_tab('Launch')).pack(side='left',padx=(0,10))
        ttk.Button(actions,text='Learn how it works',command=lambda:open_learning(app)).pack(side='left')
        metrics=ttk.Frame(body);metrics.grid(row=1,column=0,sticky='ew',pady=(0,18))
        for col,(number,label) in enumerate([(len(app.robot_profiles),'robot profiles'),(len(app.map_profiles),'world profiles'),(len(app.simulator_status),'backend adapters')]):
            metrics.columnconfigure(col,weight=1)
            card=ttk.Frame(metrics,style='Surface.TFrame',padding=16);card.grid(row=0,column=col,sticky='ew',padx=(0,10) if col<2 else 0)
            ttk.Label(card,text=str(number),style='Metric.TLabel').pack(anchor='w')
            ttk.Label(card,text=label,style='SurfaceBody.TLabel').pack(anchor='w')
        ttk.Label(body,text='What would you like to do?',style='Section.TLabel').grid(row=2,column=0,sticky='w',pady=(0,10))
        self.cards=ttk.Frame(body);self.cards.grid(row=3,column=0,sticky='ew');self.card_widgets=[]
        routes=[('01','Run a robot','Choose a model, a world and a task. Review the generated command before launch.','Launch'),
                ('02','Inspect robots & worlds','Browse complete families and inspect their geometry in the native 3D viewer.','Registry'),
                ('03','Coordinate a team','Work with named ground robots, mission queues, traffic and physical infrastructure.','Fleet & Swarm'),
                ('04','Build maps','Choose a world and generate a 2D occupancy grid for compatible workflows.','Worlds'),
                ('05','Work with aircraft','Configure a shared PX4 session and use the dedicated aerial controls.','Aerial Swarm'),
                ('06','Understand the system','Follow interactive architecture tours, source links and networking lessons.','Health')]
        for number,title,description,target in routes:
            card=ttk.Frame(self.cards,style='Surface.TFrame',padding=18);card.columnconfigure(0,weight=1)
            ttk.Label(card,text=number,style='Eyebrow.TLabel').grid(row=0,column=0,sticky='w')
            ttk.Label(card,text=title,style='CardHeading.TLabel').grid(row=1,column=0,sticky='w',pady=6)
            label=ttk.Label(card,text=description,style='SurfaceBody.TLabel',wraplength=340,justify='left');label.grid(row=2,column=0,sticky='nw')
            ttk.Button(card,text='Open workspace  →',style='Link.TButton',command=lambda page=target:app.show_tab(page)).grid(row=3,column=0,sticky='w',pady=(12,0))
            card.configure(cursor='hand2');card.bind('<Button-1>',lambda e,page=target:app.show_tab(page))
            for child in card.winfo_children():
                if isinstance(child,ttk.Label):child.configure(cursor='hand2');child.bind('<Button-1>',lambda e,page=target:app.show_tab(page))
            self.card_widgets.append((card,label))
        ttk.Label(body,text='Current selection',style='Section.TLabel').grid(row=4,column=0,sticky='w',pady=(20,8))
        self.selection_label=ttk.Label(body,textvariable=app.selection_context,wraplength=800,style='Context.TLabel')
        self.selection_label.grid(row=5,column=0,sticky='ew')
        ttk.Label(body,textvariable=app.session_badge,style='Badge.TLabel').grid(row=6,column=0,sticky='w',pady=8)
        self.support_label=ttk.Label(body,text='Inventory describes availability. Controller support and measured missions depend on the exact robot, map, backend and task. Open Learn & diagnose for current status.',wraplength=850,style='Muted.TLabel')
        self.support_label.grid(row=7,column=0,sticky='ew',pady=(12,5))
        self.canvas.bind('<Configure>',self.reflow,add='+')

    def reflow(self,event=None):
        width=max(380,self.canvas.winfo_width()-30);columns=3 if width>=1100 else 2 if width>=650 else 1
        self.intro.configure(wraplength=max(300,width-60))
        self.selection_label.configure(wraplength=max(250,width-20));self.support_label.configure(wraplength=max(250,width-20))
        for col in range(3):self.cards.columnconfigure(col,weight=1 if col<columns else 0)
        for i,(card,label) in enumerate(self.card_widgets):
            card.grid(row=i//columns,column=i%columns,sticky='nsew',padx=(0,12) if i%columns<columns-1 else 0,pady=(0,12))
            label.configure(wraplength=max(220,width//columns-55))


def install_context(app):
    """Trace existing state; configuration validity is not runtime readiness."""
    app.selection_context=tk.StringVar()
    app.session_badge=tk.StringVar(value='Not running')
    app.launch_hint=tk.StringVar(value='Choose a robot, world and task to prepare a command.')
    def refresh(*_):
        robot=app.robot_var.get();world=app.map_var.get();backend=app.simulator_var.get();mode=app.mode_var.get()
        labels={'display':'Inspect','loc':'Localize','slam':'2D SLAM','3d_slam':'3D SLAM','nav':'Navigate','flight':'Fly','marine':'Marine'}
        app.selection_context.set(f'{robot}   /   {world}   /   {backend}   /   {labels.get(mode,mode)}')
        prepared=bool(app.command_var.get());validation=app.validation_var.get()
        if not prepared:
            hint='Setup needs attention · '+validation
        elif 'experimental' in validation.casefold():
            hint='Experimental setup · review model support and options before launch.'
        elif validation.startswith('Valid'):
            hint='Command prepared · review options, then launch.'
        else:
            hint='Command prepared · '+validation[:180]
        app.launch_hint.set(hint)
    for name in ('robot_var','map_var','simulator_var','mode_var','command_var','validation_var'):
        getattr(app,name).trace_add('write',refresh)
    app.status_var.trace_add('write',lambda *_:app.session_badge.set('Launch process running' if app._launch_running else 'No launch session'))
    refresh()
