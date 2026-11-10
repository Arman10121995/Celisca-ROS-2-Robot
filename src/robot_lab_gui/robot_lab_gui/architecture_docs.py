"""Searchable in-GUI Markdown/source reader with local links and history.

The library tree follows docs/library.yaml (sections and page order); any
Markdown page it does not list still appears under "Other documents".
Pages show their figures and animations inline, and Mermaid diagrams as the
images scripts/render_doc_diagrams.py rendered (docs/diagrams/)."""
import re
from itertools import islice
import tkinter as tk
from tkinter import ttk, scrolledtext
from urllib.parse import unquote, urlsplit
import webbrowser

from .architecture_content import repository_packages


class GuideLibrary(ttk.Frame):
    def __init__(self,parent,root,open_diagram=None):
        super().__init__(parent,padding=5)
        self.root=root.resolve();self.current=None;self.history=[];self.heading_lines={};self.open_diagram=open_diagram
        self.columnconfigure(0,weight=1);self.rowconfigure(1,weight=1)
        bar=ttk.Frame(self);bar.grid(row=0,column=0,sticky='ew',pady=(0,5));bar.columnconfigure(1,weight=1)
        ttk.Label(bar,text='Find guide / package').grid(row=0,column=0,padx=(0,8))
        self.query=tk.StringVar();entry=ttk.Entry(bar,textvariable=self.query);entry.grid(row=0,column=1,sticky='ew')
        entry.bind('<KeyRelease>',lambda e:self.filter())
        ttk.Button(bar,text='Back',command=self.back).grid(row=0,column=2,padx=4)
        self.pages=ttk.Panedwindow(self,orient='horizontal');self.pages.grid(row=1,column=0,sticky='nsew')
        left=ttk.Frame(self.pages,width=330);right=ttk.Frame(self.pages);self.pages.add(left,weight=1);self.pages.add(right,weight=3)
        left.rowconfigure(0,weight=1);left.columnconfigure(0,weight=1)
        self.tree=ttk.Treeview(left,show='tree',selectmode='browse');self.tree.grid(row=0,column=0,sticky='nsew')
        scroll=ttk.Scrollbar(left,command=self.tree.yview);scroll.grid(row=0,column=1,sticky='ns');self.tree.configure(yscrollcommand=scroll.set)
        self.tree.bind('<<TreeviewSelect>>',self.selected)
        right.columnconfigure(0,weight=1);right.rowconfigure(2,weight=1)
        self.location=tk.StringVar(value='Choose a guide; links open here.')
        ttk.Label(right,textvariable=self.location,wraplength=650).grid(row=0,column=0,sticky='ew',padx=6)
        head=ttk.Frame(right);head.grid(row=1,column=0,sticky='ew');head.columnconfigure(1,weight=1)
        ttk.Label(head,text='On this page').grid(row=0,column=0,padx=5)
        self.heading=tk.StringVar();self.toc=ttk.Combobox(head,textvariable=self.heading,state='readonly');self.toc.grid(row=0,column=1,sticky='ew')
        self.toc.bind('<<ComboboxSelected>>',lambda e:self.reader.see(self.heading_lines.get(self.heading.get(),'1.0')))
        self.find=tk.StringVar();find=ttk.Entry(head,textvariable=self.find,width=16);find.grid(row=0,column=2,padx=5);find.bind('<Return>',lambda e:self.find_text())
        ttk.Button(head,text='Find text',command=self.find_text).grid(row=0,column=3)
        self.reader=scrolledtext.ScrolledText(right,wrap='word',font=('TkDefaultFont',11),padx=15,pady=12,bg='#131e2e',fg='#d7e4f7',insertbackground='#d7e4f7')
        self.reader.grid(row=2,column=0,sticky='nsew',pady=5)
        for tag,size in [('h1',20),('h2',16),('h3',13)]:self.reader.tag_configure(tag,font=('TkDefaultFont',size,'bold'),foreground='#f0f6ff',spacing1=14,spacing3=6)
        self.reader.tag_configure('code',font=('TkFixedFont',10),foreground='#9edaca',background='#1d2a3b')
        self.reader.tag_configure('bold',font=('TkDefaultFont',11,'bold'),foreground='#e4edfa')
        self.reader.tag_configure('link',foreground='#83bcff',underline=True)
        self.reader.tag_configure('match',background='#6c4e1c',foreground='#ffffff')
        self.reader.configure(state='disabled')
        self.reader.tag_configure('caption',foreground='#9fb3cc',font=('TkDefaultFont',10,'italic'),spacing3=10)
        self.images=[];self.animations=[];self.titles={}
        self.packages=repository_packages(root);self.entries={};self.filter()
        start=self.root/'docs/learn/README.md'
        self.open(start if start.exists() else self.root/'docs/architecture/interactive-guide.md')

    def page_title(self,path):
        """A page's first '# ' heading (cached; reads at most 40 lines)."""
        if path not in self.titles:
            title=path.stem.replace('_',' ').replace('-',' ')
            try:
                with path.open(encoding='utf-8',errors='replace') as stream:
                    for line in islice(stream,40):
                        if line.startswith('# '):title=line[2:].strip();break
            except OSError:pass
            self.titles[path]=title
        return self.titles[path]

    def sections(self):
        """[(title, open, [(path, title)])] from docs/library.yaml, then the rest."""
        import yaml
        try:
            data=yaml.safe_load((self.root/'docs/library.yaml').read_text()) or {}
        except (OSError,yaml.YAMLError):
            data={}
        out=[];listed=set()
        for section in data.get('sections',[]):
            pages=[]
            for page in section.get('pages',[]):
                rel,title=(page,None) if isinstance(page,str) else (page.get('path'),page.get('title'))
                path=self.root/rel
                if path.is_file():
                    pages.append((path,title or self.page_title(path)));listed.add(path.resolve())
            out.append((section.get('title','Documents'),bool(section.get('open')),pages))
        rest=[]
        for folder in ('docs/learn','docs','docs/tutorials','docs/architecture'):
            for path in sorted((self.root/folder).glob('*.md')):
                if path.resolve() not in listed:rest.append((path,self.page_title(path)))
        if rest:out.append(('Other documents',False,rest))
        return out

    def filter(self):
        query=self.query.get().casefold().strip();self.tree.delete(*self.tree.get_children());self.entries={}
        for title,is_open,pages in self.sections():
            matches=[(path,name) for path,name in pages
                     if not query or query in (str(path.relative_to(self.root))+' '+name).casefold()]
            if query and not matches:continue
            group=self.tree.insert('','end',text=title,open=bool(query) or is_open)
            for path,name in matches:
                item=self.tree.insert(group,'end',text=name);self.entries[item]=path
        packages=self.tree.insert('','end',text='Packages (actual package.xml)',open=bool(query))
        for row in self.packages:
            if query and query not in (row['name']+' '+row['description']).casefold():continue
            item=self.tree.insert(packages,'end',text=row['name']);self.entries[item]=row

    def selected(self,event=None):
        selection=self.tree.selection()
        value=self.entries.get(selection[0]) if selection else None
        if isinstance(value,dict):
            local={row['name']:row['path'] for row in self.packages}
            deps='\n'.join('- ['+name+']('+local[name]+')' if name in local else '- '+name+' (external ROS/system dependency)' for name in value['dependencies']) or '- No dependencies declared'
            text=f"# {value['name']}\n\n{value['description']}\n\n## Source manifest\n[{value['path']}]({value['path']})\n\n## Declared dependencies\n{deps}\n\nThese are build/runtime declarations, not a running ROS graph or mission qualification."
            self.current=None;self.location.set('Actual package.xml inventory');self.render(text,markdown=True,base=self.root)
        elif value:self.open(value)

    def open(self,path,anchor='',remember=True):
        path=path.resolve()
        if not path.is_relative_to(self.root):
            self.location.set('Link is outside this workspace; it was not opened.');return
        if path.is_dir():
            if remember and self.current is not None and self.current!=path:self.history.append(self.current)
            self.current=path;self.location.set(str(path.relative_to(self.root))+' /')
            try:
                children=list(islice(path.iterdir(),301))
                children=sorted(children,key=lambda item:(not item.is_dir(),item.name.casefold()))
                text='# '+path.name+'\n\nRead-only source directory. Select a file or folder to inspect it here.\n\n'
                if path!=self.root:text+='[Parent directory](..)\n\n'
                for child in children[:300]:
                    if child.name in ('.git','__pycache__'):continue
                    text+='- ['+child.name+('/' if child.is_dir() else '')+']('+child.name+')\n'
                if len(children)>300:text+='\nDirectory preview limited to 300 entries.\n'
                self.render(text,markdown=True,base=path)
            except OSError as error:self.location.set(str(error))
            return
        if not path.is_file():
            self.location.set('Not installed / unavailable: '+str(path));return
        if path.suffix.lower() in ('.png','.gif','.jpg','.jpeg'):
            if remember and self.current is not None and self.current!=path:self.history.append(self.current)
            self.current=path;self.location.set(str(path.relative_to(self.root)))
            self.render('# '+path.name,markdown=True,base=path.parent)
            self.reader.configure(state='normal');self.insert_image(path,path.name);self.reader.configure(state='disabled')
            return
        if path.suffix.lower() in ('.pgm','.stl','.dae','.onnx','.so','.db'):
            self.location.set('Binary asset: use Registry 3D or the appropriate robot/map viewer.');return
        if remember and self.current is not None and self.current!=path:self.history.append(self.current)
        self.current=path;self.location.set(str(path.relative_to(self.root)))
        try:
            with path.open(encoding='utf-8',errors='replace') as stream:text=stream.read(1_500_000)
            if path.stat().st_size>1_500_000:text+='\n\n[Preview limited to 1.5 MB.]'
            self.render(text,markdown=path.suffix.lower()=='.md',base=path.parent)
            if anchor:
                slug=lambda name:re.sub(r'[^a-z0-9 -]','',name.lower()).replace(' ','-')
                for title,line in self.heading_lines.items():
                    if slug(title.lstrip('# ').strip())==anchor:self.reader.see(line);break
        except OSError as error:self.location.set(str(error))

    def back(self):
        if self.history:self.open(self.history.pop(),remember=False)

    # -- images --------------------------------------------------------
    def max_image_width(self):
        width=self.reader.winfo_width()
        return max(320,(width if width>50 else 900)-60)

    def load_image(self,path):
        """(PhotoImage frames, frame delay ms) scaled to fit the reader."""
        try:
            from PIL import Image,ImageTk
        except ImportError:
            return [tk.PhotoImage(file=str(path))],0
        img=Image.open(path);frames=[];delay=int(img.info.get('duration',100) or 100)
        limit=self.max_image_width();count=getattr(img,'n_frames',1)
        for index in range(min(count,240)):
            img.seek(index);frame=img.convert('RGBA')
            if frame.width>limit:
                frame=frame.resize((limit,max(1,int(frame.height*limit/frame.width))),Image.LANCZOS)
            frames.append(ImageTk.PhotoImage(frame,master=self.reader))
        return frames,(delay if len(frames)>1 else 0)

    def insert_image(self,path,caption=''):
        r=self.reader
        try:
            frames,delay=self.load_image(path)
        except Exception as error:  # unreadable image: say so, keep reading
            r.insert('end','[image not shown: '+path.name+' ('+type(error).__name__+')]\n','caption');return
        self.images.extend(frames);label=tk.Label(r,image=frames[0],bd=0,bg='#131e2e',cursor='hand2')
        label.bind('<Button-1>',lambda e,p=path:self.view_image(p))
        r.window_create('end',window=label,padx=4,pady=6);r.insert('end','\n')
        if caption:r.insert('end',caption+'  (click to enlarge)\n','caption')
        if delay:
            state={'label':label,'frames':frames,'index':0,'job':None}
            def tick(state=state):
                if not state['label'].winfo_exists():return
                state['index']=(state['index']+1)%len(state['frames'])
                state['label'].configure(image=state['frames'][state['index']])
                state['job']=self.after(delay,tick)
            state['job']=self.after(delay,tick);self.animations.append(state)

    def stop_animations(self):
        for state in self.animations:
            if state['job']:
                try:self.after_cancel(state['job'])
                except tk.TclError:pass
        self.animations=[];self.images=[]

    def view_image(self,path):
        """The image at full size in its own window."""
        try:
            from PIL import Image,ImageTk
            img=Image.open(path).convert('RGBA')
            screen=(self.winfo_screenwidth()-80,self.winfo_screenheight()-120)
            scale=min(1.0,screen[0]/img.width,screen[1]/img.height)
            if scale<1:img=img.resize((int(img.width*scale),int(img.height*scale)),Image.LANCZOS)
            photo=ImageTk.PhotoImage(img,master=self)
        except Exception:
            return
        win=tk.Toplevel(self);win.title(path.name);win.configure(bg='#131e2e')
        label=tk.Label(win,image=photo,bg='#131e2e');label.image=photo;label.pack(padx=8,pady=8)
        win.bind('<Escape>',lambda e:win.destroy());label.bind('<Button-1>',lambda e:win.destroy())

    def render(self,text,markdown=False,base=None):
        self.stop_animations()
        r=self.reader;r.configure(state='normal');r.delete('1.0','end');self.heading_lines={};link_count=0;fenced=False;mermaid=False;diagram=None;columns=None;block=[]
        def inline(value):
            nonlocal link_count
            cursor=0
            for match in re.finditer(r'\[([^\]]+)\]\(([^)]+)\)|\*\*([^*]+)\*\*|`([^`]+)`',value):
                r.insert('end',value[cursor:match.start()])
                if match[1] is not None:
                    tag='doc_link_'+str(link_count);link_count+=1
                    r.insert('end',match[1],('link',tag));target=match[2]
                    r.tag_bind(tag,'<Button-1>',lambda e,t=target,b=base:self.follow(t,b))
                    r.tag_bind(tag,'<Enter>',lambda e:r.configure(cursor='hand2'));r.tag_bind(tag,'<Leave>',lambda e:r.configure(cursor='xterm'))
                else:r.insert('end',match[3] or match[4],'bold' if match[3] is not None else 'code')
                cursor=match.end()
            r.insert('end',value[cursor:])
        def finish_diagram():
            nonlocal link_count,diagram
            from .doc_diagrams import image_for
            if diagram and self.open_diagram:
                tag='diagram_'+str(link_count);link_count+=1
                r.insert('end','Open interactive flowchart →\n',('link',tag))
                r.tag_bind(tag,'<Button-1>',lambda e,key=diagram:self.open_diagram(key))
            png=image_for(self.root,'\n'.join(block)+'\n')
            if png:self.insert_image(png,'Diagram')
            else:
                r.insert('end','(diagram not rendered yet: python3 scripts/render_doc_diagrams.py)\n','caption')
                r.insert('end','\n'.join(block)+'\n','code')
            diagram=None
        para=[]
        def flush():
            # Source line breaks inside a paragraph or list item are soft:
            # the text flows to the width of the pane.
            if para:
                inline(' '.join(para));r.insert('end','\n');para.clear()
        for line in text.splitlines():
            if markdown and not fenced:
                stripped=line.strip()
                item=re.match(r'^(\s*)(?:[-*]|\d+\.)\s+(.*)',line)
                plain=(stripped and not stripped.startswith(('#','|','```','<!--','![')) and not item)
                if plain and para:
                    para.append(stripped);continue
                flush()
                if item:
                    bullet='• ' if line.lstrip()[0] in '-*' else line.lstrip().split()[0]+' '
                    para.append(item[1].replace('  ','    ')+bullet+item[2]);continue
                if plain:
                    para.append(stripped);continue
            annotation=re.fullmatch(r'<!-- diagram: ([a-z0-9_-]+) -->',line)
            if markdown and annotation:diagram=annotation[1];continue
            if markdown and line.startswith('```'):
                if fenced:
                    if mermaid:finish_diagram()
                    fenced=False;mermaid=False;r.insert('end','\n');continue
                fenced=True;mermaid=line[3:].strip()=='mermaid';block=[]
                if not mermaid:r.insert('end','\n')
                continue
            if fenced and mermaid:block.append(line);continue
            if fenced or not markdown:
                r.insert('end',line+'\n','code' if fenced or not markdown else ());continue
            image=re.fullmatch(r'\s*!\[([^\]]*)\]\(([^)\s]+)\)\s*',line)
            if image and base is not None:
                path=(base/unquote(image[2])).resolve()
                if path.is_file() and path.is_relative_to(self.root):
                    self.insert_image(path,image[1]);continue
            heading=re.match(r'^(#{1,6}) (.*)',line)
            if heading:
                title=heading[2];self.heading_lines[title]=r.index('end-1c')
                r.insert('end',title+'\n','h'+str(min(3,len(heading[1]))));continue
            if line.startswith('|') and line.endswith('|'):
                values=[value.strip() for value in line.strip('|').split('|')]
                if all(re.fullmatch(r':?-+:?',value) for value in values):continue
                if columns is None:columns=values;continue
                # Stacked table records stay readable in a narrow native pane.
                r.insert('end','\n');inline(values[0]);r.insert('end','\n')
                for label,value in zip(columns[1:],values[1:]):
                    r.insert('end',label+': ','bold');inline(value);r.insert('end','\n')
                continue
            columns=None
            inline(line);r.insert('end','\n')
        flush()
        self.toc.configure(values=list(self.heading_lines));self.heading.set(next(iter(self.heading_lines),''))
        r.configure(state='disabled');r.yview_moveto(0)

    def follow(self,target,base):
        url=urlsplit(target)
        if url.scheme in ('http','https'):
            webbrowser.open(target);return
        if url.scheme or url.netloc:
            self.location.set('Unsupported link scheme.');return
        path=(base or self.root)/unquote(url.path) if url.path else self.current
        if path:self.open(path,anchor=unquote(url.fragment))

    def find_text(self):
        r=self.reader;r.tag_remove('match','1.0','end');text=self.find.get()
        if not text:return
        start='1.0';first=None;count=0
        while count<500:
            start=r.search(text,start,stopindex='end',nocase=True)
            if not start:break
            end=f'{start}+{len(text)}c';r.tag_add('match',start,end);first=first or start;start=end;count+=1
        if first:r.see(first)
        self.location.set((str(self.current.relative_to(self.root)) if self.current else 'Package inventory')+f' · {count} text matches')
