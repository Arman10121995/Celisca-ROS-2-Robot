"""Native zoomable, keyboard-accessible diagram surface."""
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

from .architecture_graph import COLORS, WIDTH, HEIGHT, edge_points, graph_extent


class DiagramCanvas(ttk.Frame):
    def __init__(self,parent,on_select):
        super().__init__(parent)
        self.columnconfigure(0,weight=1);self.rowconfigure(0,weight=1)
        self.canvas=tk.Canvas(self,bg='#101824',highlightthickness=0,width=520,height=400,takefocus=True)
        self.canvas.grid(row=0,column=0,sticky='nsew')
        y=ttk.Scrollbar(self,orient='vertical',command=self.canvas.yview);y.grid(row=0,column=1,sticky='ns')
        x=ttk.Scrollbar(self,orient='horizontal',command=self.canvas.xview);x.grid(row=1,column=0,sticky='ew')
        self.canvas.configure(xscrollcommand=x.set,yscrollcommand=y.set)
        self.on_select=on_select;self.view=None;self.components={};self.scale=1.;self.selected=None;self.focus=set()
        self.hit={};self.auto_fit=True;self.resize_job=None
        self.font_metrics={}
        self.canvas.bind('<Configure>',self._resize)
        self.canvas.bind('<Button-1>',self._click)
        self.canvas.bind('<B1-Motion>',lambda e:self.canvas.scan_dragto(e.x,e.y,gain=1))
        for button in (2,3):
            self.canvas.bind(f'<ButtonPress-{button}>',lambda e:self.canvas.scan_mark(e.x,e.y))
            self.canvas.bind(f'<B{button}-Motion>',lambda e:self.canvas.scan_dragto(e.x,e.y,gain=1))
        self.canvas.bind('<MouseWheel>',lambda e:self.zoom(1.15 if e.delta>0 else 1/1.15))
        self.canvas.bind('<Button-4>',lambda e:self.zoom(1.15))
        self.canvas.bind('<Button-5>',lambda e:self.zoom(1/1.15))
        self.canvas.bind('<Right>',lambda e:self._cycle(1));self.canvas.bind('<Left>',lambda e:self._cycle(-1))
        self.canvas.bind('<Home>',lambda e:self.fit())
        self.bind('<Destroy>',self._destroy,add='+')

    def _destroy(self,event):
        if event.widget is self and self.resize_job:
            self.after_cancel(self.resize_job);self.resize_job=None

    def _resize(self,event=None):
        if self.resize_job:self.after_cancel(self.resize_job)
        self.resize_job=self.after(120,self._resized)

    def _resized(self):
        self.resize_job=None
        if self.auto_fit:self.fit()

    def set_graph(self,view,components):
        self.view,self.components=view,components;self.selected=None;self.focus=set();self.auto_fit=True
        self.fit()

    def fit(self):
        if not self.view:return
        width,height=graph_extent(self.view)
        self.scale=max(.32,min(1.4,(max(300,self.canvas.winfo_width())-20)/width,
                                   (max(220,self.canvas.winfo_height())-20)/height))
        self.auto_fit=True;self.draw();self.canvas.xview_moveto(0);self.canvas.yview_moveto(0)

    def zoom(self,factor):
        self.auto_fit=False;old=self.scale;self.scale=max(.12,min(2.7,self.scale*factor))
        x,y=self.canvas.xview()[0],self.canvas.yview()[0]
        if old!=self.scale:self.draw();self.canvas.xview_moveto(x);self.canvas.yview_moveto(y)

    def highlight(self,key,focus=None):
        self.selected=key;self.focus=set(focus or []);self.draw()

    def _cycle(self,direction):
        if not self.view:return
        keys=list(self.view['positions']);i=keys.index(self.selected) if self.selected in keys else -1
        self.on_select('node',keys[(i+direction)%len(keys)])

    def _click(self,event):
        self.canvas.focus_set();self.canvas.scan_mark(event.x,event.y)
        items=self.canvas.find_overlapping(self.canvas.canvasx(event.x)-2,self.canvas.canvasy(event.y)-2,
                                           self.canvas.canvasx(event.x)+2,self.canvas.canvasy(event.y)+2)
        for item in reversed(items):
            if item in self.hit:
                self.on_select(*self.hit[item]);return

    def draw(self):
        c=self.canvas;c.delete('all');self.hit={}
        if not self.view:return
        s=self.scale;w,h=graph_extent(self.view);c.configure(scrollregion=(0,0,w*s,h*s))
        c.create_text(28*s,25*s,text=self.view['title'],anchor='nw',fill='#eff5ff',font=('TkDefaultFont',max(9,round(17*s)),'bold'))
        c.create_text(28*s,52*s,text='OBSERVED SNAPSHOT' if self.view['id']=='observed' else 'DESIGNED FLOW · select nodes and arrows',anchor='nw',fill='#8499b6',font=('TkDefaultFont',max(7,round(9*s))))
        neighbors={self.selected}
        for edge in self.view['edges']:
            if self.selected in (edge['source'],edge['target']):neighbors.update([edge['source'],edge['target']])
        for i,edge in enumerate(self.view['edges']):
            points=edge_points(self.view['positions'][edge['source']],self.view['positions'][edge['target']])
            active=self.selected in (edge['source'],edge['target'])
            color='#ffd17f' if active else '#5e7594' if not self.selected else '#34465e'
            line=c.create_line(*[v*s for point in points for v in point],fill=color,width=(2.5 if active else 1.4)*max(.6,s),arrow='last',arrowshape=(9*s,11*s,4*s))
            self.hit[line]=('edge',i)
            if active or (not self.selected and len(self.view['edges'])<15 and s>=.7):
                a,b=points[1:3];x,y=(a[0]+b[0])/2,(a[1]+b[1])/2
                label=c.create_text((x+5)*s,(y-8)*s,text=edge['label'],fill=color,anchor='sw',font=('TkDefaultFont',max(7,round(9*s))))
                self.hit[label]=('edge',i)
        for key,(x,y) in self.view['positions'].items():
            row=self.components[key];color=COLORS.get(row['layer'],'#94b5e1')
            bright=(not self.focus or key in self.focus) and (not self.selected or key in neighbors)
            fill='#213550' if key==self.selected else '#19273b' if bright else '#131e2e'
            border='#ffd17f' if key==self.selected else color if bright else '#354055'
            tags=[]
            # Soft shadow plus a deliberate layer stripe retain legibility without a web engine.
            tags.append(c.create_rectangle((x+4)*s,(y+5)*s,(x+WIDTH+4)*s,(y+HEIGHT+5)*s,fill='#0b111c',outline=''))
            tags.append(c.create_rectangle(x*s,y*s,(x+WIDTH)*s,(y+HEIGHT)*s,fill=fill,outline=border,width=2 if key==self.selected else 1))
            tags.append(c.create_rectangle(x*s,y*s,(x+4)*s,(y+HEIGHT)*s,fill=border,outline=''))
            layer_font=('TkDefaultFont',max(7,round(8*s)),'bold')
            title_font=('TkDefaultFont',max(8,round(12*s)),'bold')
            def shorten(text,font):
                metrics=self.metrics(font)
                if metrics.measure(text)<=(WIDTH-23)*s:return text
                while text and metrics.measure(text+'…')>(WIDTH-23)*s:text=text[:-1]
                return text+'…'
            tags.append(c.create_text((x+13)*s,(y+12)*s,anchor='nw',text=shorten(row['layer'].upper(),layer_font),fill=border,font=layer_font))
            tags.append(c.create_text((x+13)*s,(y+31)*s,anchor='nw',text=shorten(row['title'],title_font),fill='#eef4ff' if bright else '#8190a7',font=title_font))
            if s>=.7:
                font=('TkDefaultFont',max(7,round(9*s)))
                metrics=self.metrics(font);lines=[];line=''
                maximum=max(1,int((HEIGHT-57)*s/metrics.metrics('linespace')))
                words=row['summary'].split()
                for word in words:
                    candidate=(line+' '+word).strip()
                    if line and metrics.measure(candidate)>(WIDTH-23)*s:
                        lines.append(line);line=word
                        if len(lines)>=maximum:break
                    else:line=candidate
                if len(lines)<maximum:lines.append(line)
                if len(' '.join(lines))<len(row['summary']):lines[-1]=shorten(lines[-1]+'…',font)
                tags.append(c.create_text((x+13)*s,(y+55)*s,anchor='nw',text='\n'.join(lines),fill='#bbcadf' if bright else '#68778e',font=font))
            for item in tags:self.hit[item]=('node',key)

    def metrics(self,font):
        # Font creation is expensive on the Jetson/X11. Reuse metrics across
        # hover/selection/tour redraws rather than recreating them per card.
        if font not in self.font_metrics:self.font_metrics[font]=tkfont.Font(root=self.canvas,font=font)
        return self.font_metrics[font]
