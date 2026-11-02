"""Interactive native architecture atlas embedded in the existing Health tab."""
import json
import os
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext

from .architecture_content import COMPONENTS, VIEWS, VIEW_BY_ID, JOURNEYS, LAYERS
from .architecture_canvas import DiagramCanvas
from .architecture_graph import discover_ros, network_lesson, observed_view, svg_diagram
from .workspace_ui import ScrollPanel


class ArchitectureExplorer(ttk.Frame):
    def __init__(self,parent,root,open_document):
        super().__init__(parent,padding=6)
        self.root=root;self.open_document=open_document;self.current=VIEWS[0];self.selected=None
        self.step=0;self.play_job=None;self.poll_job=None;self.closed=False;self.cancel=threading.Event()
        self.results=queue.Queue();self.snapshot=None;self.worker=None
        self.columnconfigure(0,weight=1);self.rowconfigure(1,weight=1)
        ttk.Label(self,text='Explore how Robot Lab works',font=('TkDefaultFont',17,'bold')).grid(row=0,column=0,sticky='w',pady=(0,6))
        self.tabs=ttk.Notebook(self);self.tabs.grid(row=1,column=0,sticky='nsew')
        self.map_page=ttk.Frame(self.tabs,padding=5);self.network_panel=ScrollPanel(self.tabs,width=720);self.network_page=self.network_panel.body;self.live_page=ttk.Frame(self.tabs,padding=5)
        for title,page in [('System diagrams',self.map_page),('Networking playground',self.network_panel),('Live ROS graph',self.live_page)]:self.tabs.add(page,text=title)
        self._build_map();self._build_network();self._build_live();self.show_view(VIEWS[0]['id'])
        for page in (self.map_page,self.network_page,self.live_page):
            page.bind('<Configure>',self.wrap_labels,add='+')
        self.network_panel.canvas.bind('<Configure>',self.wrap_network,add='+')
        self.bind('<Destroy>',self._destroy,add='+')

    def wrap_labels(self,event):
        if event.widget is self.network_page:
            self.wrap_network();return
        for child in event.widget.winfo_children():
            if isinstance(child,ttk.Label) and int(child.cget('wraplength') or 0)>0:
                child.configure(wraplength=max(250,event.width-20))

    def wrap_network(self,event=None):
        width=max(250,self.network_panel.canvas.winfo_width()-24)
        for child in self.network_page.winfo_children():
            if isinstance(child,ttk.Label) and int(child.cget('wraplength') or 0)>0:
                child.configure(wraplength=width)

    def _build_map(self):
        page=self.map_page;page.columnconfigure(0,weight=1);page.rowconfigure(3,weight=1)
        bar=ttk.Frame(page);bar.grid(row=0,column=0,sticky='ew');bar.columnconfigure(0,weight=1)
        self.view_var=tk.StringVar(value=VIEWS[0]['title'])
        chooser=ttk.Combobox(bar,textvariable=self.view_var,values=[v['title'] for v in VIEWS],state='readonly')
        chooser.grid(row=0,column=0,sticky='ew');chooser.bind('<<ComboboxSelected>>',lambda e:self.change_view(next(v['id'] for v in VIEWS if v['title']==self.view_var.get())))
        ttk.Button(bar,text='−',width=3,command=lambda:self.diagram.zoom(1/1.2)).grid(row=0,column=1,padx=(5,0))
        ttk.Button(bar,text='+',width=3,command=lambda:self.diagram.zoom(1.2)).grid(row=0,column=2)
        ttk.Button(bar,text='Fit',command=lambda:self.diagram.fit()).grid(row=0,column=3)
        ttk.Button(bar,text='Save SVG',command=self.export).grid(row=0,column=4,padx=4)
        ttk.Button(bar,text='Glossary',command=lambda:self.open_document(self.root/'docs/architecture/glossary.md')).grid(row=0,column=5)
        tour=ttk.Frame(page);tour.grid(row=1,column=0,sticky='ew',pady=5);tour.columnconfigure(0,weight=1)
        self.journey=tk.StringVar(value=next(iter(JOURNEYS)))
        combo=ttk.Combobox(tour,textvariable=self.journey,values=list(JOURNEYS),state='readonly');combo.grid(row=0,column=0,sticky='ew')
        combo.bind('<<ComboboxSelected>>',lambda e:self.start_journey())
        ttk.Button(tour,text='Start tour',command=self.start_journey).grid(row=0,column=1,padx=4)
        ttk.Button(tour,text='Previous',command=lambda:self.advance(-1)).grid(row=0,column=2)
        ttk.Button(tour,text='Next',command=lambda:self.advance(1)).grid(row=0,column=3)
        self.play_button=ttk.Button(tour,text='Play',command=self.toggle_play);self.play_button.grid(row=0,column=4,padx=4)
        search=ttk.Frame(page);search.grid(row=2,column=0,sticky='ew',pady=(0,5));search.columnconfigure(1,weight=1)
        ttk.Label(search,text='Find a component').grid(row=0,column=0,padx=(0,5))
        self.search=tk.StringVar();entry=ttk.Entry(search,textvariable=self.search);entry.grid(row=0,column=1,sticky='ew');entry.bind('<KeyRelease>',self.search_components)
        self.match=tk.StringVar();self.matches=ttk.Combobox(search,textvariable=self.match,state='readonly',width=22);self.matches.grid(row=0,column=2,padx=5)
        self.matches.bind('<<ComboboxSelected>>',self.choose_match)
        self.layer=tk.StringVar(value='All layers');layers=ttk.Combobox(search,textvariable=self.layer,values=['All layers']+LAYERS,state='readonly',width=16)
        layers.grid(row=0,column=3);layers.bind('<<ComboboxSelected>>',lambda e:self.select_node(self.selected) if self.selected else self.diagram.highlight(None,self.layer_focus()))
        panes=ttk.Panedwindow(page,orient='horizontal');panes.grid(row=3,column=0,sticky='nsew')
        self.diagram=DiagramCanvas(panes,self.select);panes.add(self.diagram,weight=3)
        detail=ttk.Frame(panes,width=300);detail.columnconfigure(0,weight=1);detail.rowconfigure(0,weight=1);panes.add(detail,weight=2)
        self.inspector=scrolledtext.ScrolledText(detail,wrap='word',font=('TkDefaultFont',11),padx=12,pady=10,bg='#182436',fg='#d8e5f7',width=30)
        self.inspector.grid(row=0,column=0,sticky='nsew')
        self.inspector.tag_configure('title',font=('TkDefaultFont',17,'bold'),foreground='#f1f6ff',spacing3=10)
        self.inspector.tag_configure('heading',font=('TkDefaultFont',11,'bold'),foreground='#8cbeff',spacing1=12,spacing3=5)
        self.inspector.tag_configure('link',foreground='#92c6ff',underline=True)
        self.inspector.tag_configure('mono',font=('TkFixedFont',9),foreground='#a2d6c9')
        ttk.Button(detail,text='Read the related guide',command=self.related_guide).grid(row=1,column=0,sticky='ew',pady=4)
        self.lesson=tk.StringVar();ttk.Label(page,textvariable=self.lesson,wraplength=1000,justify='left').grid(row=4,column=0,sticky='ew',pady=5)
        ttk.Label(page,text='Click cards/arrows · wheel or +/− to zoom · drag background to pan · Left/Right selects cards · Home fits',wraplength=900).grid(row=5,column=0,sticky='ew')

    def show_view(self,key):
        self.current=VIEW_BY_ID[key];self.view_var.set(self.current['title']);self.diagram.set_graph(self.current,COMPONENTS)
        self.lesson.set(self.current['lesson']);self.selected=None
        self._text(self.current['title'],self.current['subtitle']+'\n\n'+self.current['lesson']+'\n\nSelect a card for its contracts, source files, incoming/outgoing links and tutorial. This is a teaching diagram; open Live ROS graph for discovery observations.')

    def change_view(self,key):
        self.pause();self.show_view(key)

    def _text(self,title,body):
        r=self.inspector;r.configure(state='normal');r.delete('1.0','end');r.insert('end',title+'\n','title');r.insert('end',body+'\n');r.configure(state='disabled');r.yview_moveto(0)

    def layer_focus(self):
        return {key for key in self.current['positions'] if COMPONENTS[key]['layer']==self.layer.get()} if self.layer.get()!='All layers' else set()

    def select(self,kind,item):
        self.pause()
        if kind=='node':self.select_node(item)
        else:
            edge=self.current['edges'][item];a,b=COMPONENTS[edge['source']],COMPONENTS[edge['target']]
            self.diagram.highlight(None,{edge['source'],edge['target']})
            self._text(a['title']+' → '+b['title'],edge['label']+'\n\n'+a['summary']+'\n\n'+b['summary']+'\n\nThis arrow describes a logical relationship. Select either card for exact source files and interface examples; a drawn edge is not a live message-rate measurement.')

    def select_node(self,key):
        if key not in self.current['positions']:return
        self.selected=key;row=COMPONENTS[key];self.diagram.highlight(key,self.layer_focus())
        r=self.inspector;r.configure(state='normal');r.delete('1.0','end')
        r.insert('end',row['title']+'\n','title');r.insert('end',row['layer'].upper()+'\n','heading')
        r.insert('end',row['summary']+'\n\n'+row['detail']+'\n')
        if row['ports']:r.insert('end','Interfaces / examples\n','heading');r.insert('end',row['ports']+'\n','mono')
        r.insert('end','Connected in this view\n','heading')
        for edge in self.current['edges']:
            if key not in (edge['source'],edge['target']):continue
            peer=edge['target'] if edge['source']==key else edge['source'];arrow='→' if edge['source']==key else '←'
            tag='peer_'+peer;r.insert('end',arrow+' '+COMPONENTS[peer]['title']+' · '+edge['label']+'\n',('link',tag))
            r.tag_bind(tag,'<Button-1>',lambda e,p=peer:(self.pause(),self.select_node(p)))
        r.insert('end','Inspect the implementation\n','heading')
        for index,path in enumerate(row['sources']):
            tag='source_'+str(index);r.insert('end',path+'\n',('link',tag));r.tag_bind(tag,'<Button-1>',lambda e,p=path:(self.pause(),self.open_document(self.root/p)))
        r.configure(state='disabled');r.yview_moveto(0)

    def related_guide(self):
        self.pause()
        self.open_document(self.root/(COMPONENTS[self.selected]['guide'] if self.selected else 'docs/architecture/interactive-guide.md'))

    def search_components(self,event=None):
        query=self.search.get().casefold().strip()
        self.match_map={row['title']:key for key,row in COMPONENTS.items() if query and query in (' '.join([row['title'],row['summary'],row['detail'],row['ports'],*row['sources']])).casefold()}
        self.matches.configure(values=list(self.match_map));self.match.set(next(iter(self.match_map),''))
        if self.match_map:self.choose_match()

    def choose_match(self,event=None):
        key=getattr(self,'match_map',{}).get(self.match.get())
        if not key:return
        self.pause()
        if key not in self.current['positions']:self.show_view(next(view['id'] for view in VIEWS if key in view['positions']))
        self.select_node(key)

    def start_journey(self):
        self.pause();self.step=0;self.show_step()

    def show_step(self):
        steps=JOURNEYS[self.journey.get()];view,key=steps[self.step]
        self.show_view(view);self.select_node(key)
        self.lesson.set(f'Tour {self.step+1}/{len(steps)} · '+COMPONENTS[key]['summary']+'  Use Next to follow the system.')

    def advance(self,direction):
        self.pause();steps=JOURNEYS[self.journey.get()];self.step=max(0,min(len(steps)-1,self.step+direction));self.show_step()

    def pause(self):
        if self.play_job:self.after_cancel(self.play_job);self.play_job=None
        self.play_button.configure(text='Play')

    def toggle_play(self):
        if self.play_job:self.pause();return
        self.show_step();self.play_button.configure(text='Pause');self.play_job=self.after(4500,self._play)

    def _play(self):
        self.play_job=None;steps=JOURNEYS[self.journey.get()]
        if self.step>=len(steps)-1:self.pause();return
        self.step+=1;self.show_step();self.play_button.configure(text='Pause');self.play_job=self.after(4500,self._play)

    def export(self):
        path=filedialog.asksaveasfilename(parent=self,title='Export architecture diagram',defaultextension='.svg',filetypes=[('Scalable vector diagram','*.svg')],initialdir=self.root/'docs/architecture')
        if path:
            try:Path(path).write_text(svg_diagram(self.current,COMPONENTS),encoding='utf-8');self.lesson.set('Saved standalone vector diagram: '+path)
            except OSError as error:self.lesson.set(str(error))

    def _build_network(self):
        page=self.network_page;page.columnconfigure(0,weight=1);page.rowconfigure(3,weight=1)
        ttk.Label(page,text='Break and repair a ROS connection',font=('TkDefaultFont',15,'bold')).grid(row=0,column=0,sticky='w')
        ttk.Label(page,text='Teaching sandbox only. These fields do not change environment variables, connect to a robot or test actual network reachability.',wraplength=920).grid(row=1,column=0,sticky='ew',pady=6)
        form=ttk.Frame(page);form.grid(row=2,column=0,sticky='ew');form.columnconfigure(1,weight=1);form.columnconfigure(2,weight=1)
        ttk.Label(form,text='Offered by publisher').grid(row=0,column=1);ttk.Label(form,text='Requested by subscriber').grid(row=0,column=2)
        self.network_vars={}
        fields=[('domain','DDS domain ID','42',None),('ns','Namespace','/alpha',None),('topic','Topic','cmd_vel',None),
                ('type','Message type','geometry_msgs/msg/Twist',['geometry_msgs/msg/Twist','nav_msgs/msg/Odometry','sensor_msgs/msg/LaserScan']),
                ('reliability','Reliability','reliable',['reliable','best_effort']),('durability','Durability','volatile',['volatile','transient_local'])]
        for row,(key,label,default,options) in enumerate(fields,1):
            ttk.Label(form,text=label).grid(row=row,column=0,sticky='w',padx=5,pady=4)
            for col,role in enumerate(['publisher','subscriber'],1):
                var=tk.StringVar(value=default);self.network_vars[role+'_'+key]=var
                if options:
                    widget=ttk.Combobox(form,textvariable=var,values=options,state='readonly');widget.bind('<<ComboboxSelected>>',lambda e:self.update_network())
                else:
                    widget=ttk.Entry(form,textvariable=var);widget.bind('<KeyRelease>',lambda e:self.update_network())
                widget.grid(row=row,column=col,sticky='ew',padx=5,pady=4)
        self.network_result=tk.StringVar();ttk.Label(page,textvariable=self.network_result,wraplength=930,justify='left').grid(row=3,column=0,sticky='new',pady=12)
        ttk.Button(page,text='Read networking and troubleshooting guide',command=lambda:self.open_document(self.root/'docs/architecture/networking.md')).grid(row=4,column=0,sticky='ew')
        self.update_network()

    def update_network(self):
        checks=network_lesson({key:var.get() for key,var in self.network_vars.items()})
        text=('These example gates match. This is not an actual connection test.' if all(ok for _,ok,_ in checks) else 'The teaching connection has a mismatch:')
        text+='\n\n'+'\n'.join(('✓ ' if ok else '✗ ')+name+' — '+reason for name,ok,reason in checks)
        text+='\n\nTry best_effort offered / reliable requested, or /beta on one side. An absolute topic ignores the namespace.\n\n'+ 'Actual GUI environment (read only): ROS_DOMAIN_ID='+os.environ.get('ROS_DOMAIN_ID','0')+'; ROS_LOCALHOST_ONLY='+os.environ.get('ROS_LOCALHOST_ONLY','unset')+'; RMW_IMPLEMENTATION='+os.environ.get('RMW_IMPLEMENTATION','default')
        text+='\n\nMatching names and these two QoS policies are not enough: discovery, interfaces/firewalls, security, other QoS policies, TF, clock and live data still matter. Gazebo partitions and MAVLink ports are separate.'
        self.network_result.set(text)

    def _build_live(self):
        page=self.live_page;page.columnconfigure(0,weight=1);page.rowconfigure(2,weight=1)
        bar=ttk.Frame(page);bar.grid(row=0,column=0,sticky='ew');bar.columnconfigure(1,weight=1)
        self.refresh_button=ttk.Button(bar,text='Discover ROS graph',command=self.refresh_live);self.refresh_button.grid(row=0,column=0,padx=(0,5))
        self.live_filter=tk.StringVar();entry=ttk.Entry(bar,textvariable=self.live_filter);entry.grid(row=0,column=1,sticky='ew');entry.bind('<Return>',lambda e:self.render_live())
        ttk.Button(bar,text='Filter node / topic',command=self.render_live).grid(row=0,column=2,padx=4)
        ttk.Button(bar,text='Save snapshot JSON',command=self.save_snapshot).grid(row=0,column=3)
        self.hide_infra=tk.BooleanVar(value=True);ttk.Checkbutton(page,text='Hide common clock / TF / log / parameter topics from the drawing',variable=self.hide_infra,command=self.render_live).grid(row=1,column=0,sticky='w',pady=5)
        split=ttk.Panedwindow(page,orient='horizontal');split.grid(row=2,column=0,sticky='nsew')
        self.live_canvas=DiagramCanvas(split,self.select_live);split.add(self.live_canvas,weight=3)
        self.live_detail=scrolledtext.ScrolledText(split,wrap='word',width=30,font=('TkDefaultFont',10),bg='#182436',fg='#d8e5f7',padx=10,pady=10)
        split.add(self.live_detail,weight=2)
        self.live_status=tk.StringVar(value='Not connected. Discovery starts only on request, in the GUI process domain; it sends no robot commands or service requests.')
        ttk.Label(page,textvariable=self.live_status,wraplength=1000,justify='left').grid(row=3,column=0,sticky='ew',pady=6)
        self._live_text('Start a simulation in the same ROS domain, then Discover.\n\nThis view shows advertised publishers → topics → subscribers and service declarations in the inspector. It does not subscribe to sensor data or claim actual message delivery. Use Live Monitor for observations.\n\nDiscovery briefly creates its own ROS context and node, then destroys them; it never shuts down the GUI/control context.')

    def _live_text(self,text):
        r=self.live_detail;r.configure(state='normal');r.delete('1.0','end');r.insert('end',text);r.configure(state='disabled')

    def refresh_live(self):
        if self.worker and self.worker.is_alive():return
        self.cancel.clear();self.refresh_button.configure(state='disabled');self.live_status.set('Discovering endpoints in the current domain… UI and robot controls remain independent.')
        def work():
            try:self.results.put(('snapshot',discover_ros(self.cancel)))
            except Exception as error:self.results.put(('error',str(error)))
        self.worker=threading.Thread(target=work,name='architecture-discovery',daemon=True);self.worker.start()
        self.poll_job=self.after(100,self.poll_live)

    def poll_live(self):
        self.poll_job=None
        if self.closed:return
        try:kind,value=self.results.get_nowait()
        except queue.Empty:self.poll_job=self.after(100,self.poll_live);return
        self.refresh_button.configure(state='normal')
        if kind=='error':self.live_status.set('Discovery unavailable: '+value+' · launch GUI from the sourced ROS environment.');return
        self.snapshot=value;self.render_live()

    def render_live(self):
        if not self.snapshot:return
        self.live_view,self.live_components=observed_view(self.snapshot,self.live_filter.get(),self.hide_infra.get())
        self.live_canvas.set_graph(self.live_view,self.live_components)
        s=self.snapshot
        self.live_status.set(f"Observed {s['observed_at']} · domain {s['domain']} · {s['rmw']} · {len(s['nodes'])} nodes / {len(s['topics'])} topics"+
            (' · discovery truncated' if s['truncated'] else '')+f" · {len(s['errors'])} discovery warnings. Display: {len(self.live_view['positions'])} cards / {len(self.live_view['edges'])} arrows; drawing is bounded. Refresh for changes.")
        self._live_text(s['scope']+'\n\n'+('\n'.join(s['errors']) if s['errors'] else 'No discovery exceptions recorded.'))

    def select_live(self,kind,item):
        if kind=='node':
            row=self.live_components[item];self.live_canvas.highlight(item);self._live_text(row['title']+'\n\n'+row['detail'])
        else:
            edge=self.live_view['edges'][item]
            self._live_text(edge['source']+'\n'+edge['label']+'\n'+edge['target']+'\n\nAdvertised endpoint relationship only; message traffic and QoS compatibility were not measured.')

    def save_snapshot(self):
        if not self.snapshot:self.live_status.set('Discover first; there is no observation to export.');return
        initial=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT',str(self.root)))
        path=filedialog.asksaveasfilename(parent=self,title='Export actual discovery snapshot',initialdir=initial,defaultextension='.json',filetypes=[('JSON','*.json')])
        if path:
            try:Path(path).write_text(json.dumps(self.snapshot,indent=2),encoding='utf-8');self.live_status.set('Saved observed snapshot: '+path)
            except OSError as error:self.live_status.set(str(error))

    def _destroy(self,event):
        if event.widget is self:
            self.closed=True;self.cancel.set()
            if self.play_job:self.after_cancel(self.play_job);self.play_job=None
            if self.poll_job:self.after_cancel(self.poll_job);self.poll_job=None
