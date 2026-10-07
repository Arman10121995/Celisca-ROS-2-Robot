"""An embedded Tk/OpenGL 3D inspector with owned background asset loading.

The Linux GLX context belongs to the Tk frame itself. No external viewer
window or ROS command publisher is involved.
"""
import ctypes
import ctypes.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
import tkinter as tk
from tkinter import ttk


class NativeScene(tk.Frame):
    def __init__(self, parent, status, **kwargs):
        super().__init__(parent, background='#23272f', **kwargs)
        self.status = status
        self.display = self.context = None
        self.drawable = 0
        self.buffers = {}
        self.scene = None
        self.arrays = None
        self.target, self.distance, self.yaw, self.pitch = np.zeros(3), 2., .75, .5
        self.draw_job = None
        self.draws = 0
        self.error = None
        self.bind('<Map>', self._mapped)
        self.bind('<Configure>', lambda _event: self.request_draw())
        self.bind('<Expose>', lambda _event: self.request_draw())
        self.bind('<ButtonPress-1>', self._press)
        self.bind('<ButtonPress-3>', self._press)
        self.bind('<B1-Motion>', self._orbit)
        self.bind('<B3-Motion>', self._pan)
        self.bind('<Button-4>', lambda _event: self.zoom(.85))
        self.bind('<Button-5>', lambda _event: self.zoom(1/.85))
        self.bind('<MouseWheel>', lambda event: self.zoom(.85 if event.delta > 0 else 1/.85))
        self.bind('<Destroy>', lambda event: self.close() if event.widget is self else None)

    def _mapped(self, _event):
        # Tk and GLX use separate X connections. Let Tk flush its newly
        # created child window before GLX wraps that native drawable.
        self.after(50, self._initialize_mapped)

    def _initialize_mapped(self):
        if not self.winfo_exists() or not self.winfo_ismapped():
            return
        if self.context is None and self.error is None:
            try:
                self._create_context()
            except Exception as exc:
                self.error = str(exc)
                self.status.set('Native 3D viewer unavailable: '+str(exc))
                return
        self.request_draw()

    def _create_context(self):
        from OpenGL import GL, GLX
        from OpenGL.raw.GLX._types import Display
        self.GL, self.GLX = GL, GLX
        self.x11 = ctypes.CDLL(ctypes.util.find_library('X11'))
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.POINTER(Display)
        self.x11.XDefaultScreen.argtypes = [ctypes.POINTER(Display)]
        self.x11.XDefaultScreen.restype = ctypes.c_int
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.argtypes = [ctypes.POINTER(Display)]
        self.display = self.x11.XOpenDisplay(self.winfo_screen().encode())
        if not self.display:
            raise RuntimeError('Cannot open the Tk display for OpenGL')
        screen = self.x11.XDefaultScreen(self.display)
        attributes = [GLX.GLX_X_RENDERABLE, 1, GLX.GLX_DRAWABLE_TYPE, GLX.GLX_WINDOW_BIT,
            GLX.GLX_RENDER_TYPE, GLX.GLX_RGBA_BIT, GLX.GLX_DOUBLEBUFFER, 1,
            GLX.GLX_DEPTH_SIZE, 16, 0]
        count = ctypes.c_int()
        configs = GLX.glXChooseFBConfig(self.display, screen,
            (ctypes.c_int*len(attributes))(*attributes), ctypes.byref(count))
        try:
            ideal = int(self.winfo_visualid(), 16)
            for index in range(count.value):
                visual = GLX.glXGetVisualFromFBConfig(self.display, configs[index])
                matches = bool(visual) and visual.contents.visualid == ideal
                if visual:
                    self.x11.XFree(visual)
                if matches:
                    self.context = GLX.glXCreateNewContext(self.display, configs[index], GLX.GLX_RGBA_TYPE, None, True)
                    self.drawable = GLX.glXCreateWindow(self.display, configs[index], self.winfo_id(), None)
                    break
        finally:
            if configs:
                self.x11.XFree(configs)
        if not self.context or not self.drawable or not GLX.glXMakeContextCurrent(self.display, self.drawable, self.drawable, self.context):
            raise RuntimeError('No compatible OpenGL visual for the native Tk viewport')
        self.renderer = GL.glGetString(GL.GL_RENDERER).decode(errors='replace')
        GL.glClearColor(.137, .153, .184, 1.)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_NORMALIZE)
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)
        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, [.8, .8, .8, 1.])
        GL.glLightModelfv(GL.GL_LIGHT_MODEL_AMBIENT, [.3, .3, .3, 1.])
        GL.glLightModeli(GL.GL_LIGHT_MODEL_TWO_SIDE, GL.GL_TRUE)

    def load(self, scene, arrays):
        self.scene, self.arrays = scene, arrays
        self.fit()
        self.request_draw()

    def fit(self, whole_scene=False):
        if self.scene:
            bounds = self.scene['bounds']
            if whole_scene:
                lo, hi = np.asarray(bounds['raw_min_xyz']), np.asarray(bounds['raw_max_xyz'])
                self.target, self.fit_span = (lo+hi)/2, float(np.linalg.norm(hi-lo))
            else:
                self.target = np.asarray(bounds['center_xyz'], dtype=float)
                self.fit_span = bounds['span_m']
            self.distance = max(.3, self.fit_span*1.25)
        self.yaw, self.pitch = .75, .5
        self.request_draw()

    def view(self, name):
        if name == 'Top':
            self.yaw, self.pitch = 0., math.pi/2-.001
        elif name == 'Front':
            self.yaw, self.pitch = 0., .001
        else:
            self.yaw, self.pitch = math.pi/2, .001
        self.request_draw()

    def _press(self, event):
        self.drag = (event.x, event.y)

    def _orbit(self, event):
        previous = getattr(self, 'drag', (event.x, event.y))
        self.yaw -= (event.x-previous[0])*.01
        self.pitch = float(np.clip(self.pitch+(event.y-previous[1])*.01, -1.55, 1.55))
        self.drag = event.x, event.y
        self.request_draw()

    def _pan(self, event):
        previous = getattr(self, 'drag', (event.x, event.y))
        right = np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.])
        up = np.array([-math.sin(self.pitch)*math.cos(self.yaw),
                       -math.sin(self.pitch)*math.sin(self.yaw), math.cos(self.pitch)])
        self.target += self.distance*.002*(-(event.x-previous[0])*right+(event.y-previous[1])*up)
        self.drag = event.x, event.y
        self.request_draw()

    def zoom(self, factor):
        span = max(.1, self.fit_span) if self.scene else 1.
        self.distance = float(np.clip(self.distance*factor, span*.02, span*100))
        self.request_draw()

    def request_draw(self):
        if self.draw_job is None and self.winfo_exists() and self.winfo_ismapped():
            self.draw_job = self.after_idle(self._draw)

    def _delete_buffers(self):
        if self.buffers:
            buffers = [int(buffer) for item in self.buffers.values() for buffer in item[:3]]
            self.GL.glDeleteBuffers(len(buffers), buffers)
            self.buffers.clear()

    def _upload(self):
        if self.arrays is None:
            return
        GL = self.GL
        self._delete_buffers()
        for index in range(self.scene['render_geometry']['meshes']):
            vertex, normal, face = GL.glGenBuffers(3)
            for buffer, name, target in [(vertex, 'vertices_', GL.GL_ARRAY_BUFFER),
                                         (normal, 'normals_', GL.GL_ARRAY_BUFFER),
                                         (face, 'faces_', GL.GL_ELEMENT_ARRAY_BUFFER)]:
                array = self.arrays[name+str(index)]
                GL.glBindBuffer(target, int(buffer))
                GL.glBufferData(target, array.nbytes, array, GL.GL_STATIC_DRAW)
            self.buffers[index] = int(vertex), int(normal), int(face), len(self.arrays['faces_'+str(index)])
        self.arrays = None

    def _draw(self, read_pixels=False):
        self.draw_job = None
        if not self.context or not self.winfo_ismapped():
            return
        try:
            self.GLX.glXMakeContextCurrent(self.display, self.drawable, self.drawable, self.context)
            GL = self.GL
            self._upload()
            width, height = max(1, self.winfo_width()), max(1, self.winfo_height())
            GL.glViewport(0, 0, width, height)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            GL.glMatrixMode(GL.GL_PROJECTION); GL.glLoadIdentity()
            span = max(.1, self.fit_span) if self.scene else 1.
            near, far = max(.001, span*.0001), self.distance+span*20
            top = near*math.tan(math.radians(40)/2)
            right = top*width/height
            GL.glFrustum(-right, right, -top, top, near, far)
            GL.glMatrixMode(GL.GL_MODELVIEW); GL.glLoadIdentity()
            GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, [0., 0., 1., 0.])
            direction = np.array([math.cos(self.pitch)*math.cos(self.yaw),
                                  math.cos(self.pitch)*math.sin(self.yaw), math.sin(self.pitch)])
            eye = self.target+self.distance*direction
            forward = -direction
            side = np.cross(forward, [0., 0., 1.]); side /= np.linalg.norm(side)
            up = np.cross(side, forward)
            view = np.eye(4)
            view[:3, :3] = [side, up, -forward]
            view[:3, 3] = [-np.dot(side, eye), -np.dot(up, eye), np.dot(forward, eye)]
            GL.glMultMatrixf(np.asarray(view.T, dtype=np.float32))
            if self.scene:
                from robot_lab_utils.asset_preview import _rotation
                GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
                GL.glEnableClientState(GL.GL_NORMAL_ARRAY)
                for shape in self.scene['shapes']:
                    GL.glPushMatrix()
                    transform = np.eye(4)
                    transform[:3, :3] = _rotation(shape['orientation']) @ np.diag(shape.get('scale', [1., 1., 1.]))
                    transform[:3, 3] = shape['position']
                    GL.glMultMatrixf(np.asarray(transform.T, dtype=np.float32))
                    GL.glColor4f(*shape.get('rgba', [.68, .73, .8, 1.]))
                    vertex, normal, face, count = self.buffers[shape['render_mesh']]
                    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vertex)
                    GL.glVertexPointer(3, GL.GL_FLOAT, 0, ctypes.c_void_p(0))
                    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, normal)
                    GL.glNormalPointer(GL.GL_FLOAT, 0, ctypes.c_void_p(0))
                    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, face)
                    GL.glDrawElements(GL.GL_TRIANGLES, count, GL.GL_UNSIGNED_INT, ctypes.c_void_p(0))
                    GL.glPopMatrix()
                GL.glDisableClientState(GL.GL_VERTEX_ARRAY); GL.glDisableClientState(GL.GL_NORMAL_ARRAY)
            GL.glFlush()
            pixels = None
            if read_pixels:
                # Read the rendered back buffer before presenting it. Mesa
                # can expose an empty GL_FRONT for a Tk/GLX child drawable.
                GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
                GL.glReadBuffer(GL.GL_BACK)
                raw = GL.glReadPixels(0, 0, width, height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
                pixels = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)[::-1].copy()
            self.GLX.glXSwapBuffers(self.display, self.drawable)
            self.draws += 1
            return pixels
        except Exception as exc:
            self.error = str(exc)
            self.status.set('3D rendering error: '+str(exc))

    def pixels(self):
        """Actual framebuffer readback for screenshots and rendering checks."""
        if not self.context or self.error:
            raise RuntimeError(self.error or 'OpenGL viewport is not initialized')
        pixels = self._draw(read_pixels=True)
        if pixels is None:
            raise RuntimeError(self.error or 'Viewport is not visible')
        return pixels

    def close(self):
        if self.draw_job is not None:
            self.after_cancel(self.draw_job); self.draw_job = None
        if self.context:
            try:
                self.GLX.glXMakeContextCurrent(self.display, self.drawable, self.drawable, self.context)
                self._delete_buffers()
                self.GLX.glXMakeContextCurrent(self.display, 0, 0, None)
                self.GLX.glXDestroyContext(self.display, self.context)
                self.GLX.glXDestroyWindow(self.display, self.drawable)
            finally:
                self.context = None
                self.drawable = 0
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None

    def destroy(self):
        # Release GLX while its native Tk window still exists.
        self.close()
        super().destroy()


class NativePreviewPane(ttk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent, padding=4)
        self.app, self.process, self.job, self.canvas = app, None, None, None
        self.generation = 0
        self.output = None
        self.status = tk.StringVar(value='Select a robot or world and click Preview 3D.')
        self.title = tk.StringVar(value='Registry 3D preview')
        self.whole_scene = tk.BooleanVar(value=False)
        self.columnconfigure(0, weight=1); self.rowconfigure(1, weight=1)
        controls = ttk.Frame(self); controls.grid(row=0, column=0, sticky='ew')
        controls.columnconfigure(3, weight=1)
        title = ttk.Label(controls, textvariable=self.title, wraplength=400)
        title.grid(row=0, column=0, columnspan=4, sticky='w', pady=(0, 4))
        ttk.Button(controls, text='Fit', width=6, style='Small.TButton', command=lambda: self.canvas.fit(self.whole_scene.get()) if self.canvas else None).grid(row=1, column=0, sticky='w')
        for column, name in enumerate(('Top', 'Front', 'Side'), start=1):
            ttk.Button(controls, text=name, width=6, style='Small.TButton', command=lambda value=name: self.canvas.view(value) if self.canvas else None).grid(row=1, column=column, sticky='w', padx=3)
        self.placeholder = ttk.Label(self, text='Robot and world geometry will appear here.', anchor='center')
        self.placeholder.grid(row=1, column=0, sticky='nsew')
        input_bar = ttk.Frame(self); input_bar.grid(row=2, column=0, sticky='ew')
        ttk.Label(input_bar, text='Left drag: orbit  ·  Right drag: pan  ·  Wheel: zoom', wraplength=340).pack(side='left')
        ttk.Checkbutton(input_bar, text='Whole scene', variable=self.whole_scene,
            command=lambda: self.canvas.fit(self.whole_scene.get()) if self.canvas else None).pack(side='right')
        status = ttk.Label(self, textvariable=self.status, wraplength=400, justify='left')
        status.grid(row=3, column=0, sticky='ew', pady=4)
        self.bind('<Configure>', lambda event: (title.configure(wraplength=max(150, event.width-12)),
            status.configure(wraplength=max(150, event.width-12))))

    def load(self, kind, asset_id, profile):
        from .launcher import subprocess_env
        from .process_control import stop_owned_launch
        if self.process is not None:
            threading.Thread(target=stop_owned_launch, args=(self.process,), daemon=False).start()
        if self.job is not None:
            self.after_cancel(self.job); self.job = None
        self.generation += 1
        generation = self.generation
        root = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
        if Path('/workspace') not in root.resolve().parents:
            raise ValueError('Registry preview storage must be on the workspace SSD')
        self.output = root/'registry_previews'/uuid.uuid4().hex
        self.output.mkdir(parents=True)
        request = dict(schema_version=1, kind=kind, asset_id=asset_id, profile=profile,
                       output=str(self.output), scope='Embedded static inspection; no ROS nodes or simulator')
        (self.output/'request.json').write_text(json.dumps(request, indent=2)+'\n')
        self.title.set(asset_id)
        self.whole_scene.set(False)
        self.status.set('Preparing 3D geometry…')
        env = subprocess_env()
        env['ROBOT_LAB_PREVIEW_ID'] = self.output.name
        env['ROS_LOG_DIR'] = str(self.output/'ros_logs')
        with (self.output/'loader.log').open('w') as log:
            self.process = subprocess.Popen([sys.executable, '-m', 'robot_lab_gui.registry_preview',
                '--request', str(self.output/'request.json')], env=env, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
        self.deadline = time.monotonic()+120
        self.job = self.after(100, lambda: self._poll(generation))

    def _poll(self, generation):
        self.job = None
        if generation != self.generation or self.process is None:
            return
        code = self.process.poll()
        if code is None:
            if time.monotonic() > self.deadline:
                self.status.set('3D asset preparation exceeded its time limit; see loader log.')
                from .process_control import stop_owned_launch
                threading.Thread(target=stop_owned_launch, args=(self.process,), daemon=False).start()
                self.process = None
            else:
                self.job = self.after(100, lambda: self._poll(generation))
            return
        if code or not (self.output/'scene.json').is_file():
            log = (self.output/'loader.log').read_text()[-2000:]
            self.status.set('Could not load 3D geometry. See the console for details.')
            self.app.log('[3D preview] '+log+'\n')
            self.process = None
            return
        scene = json.loads((self.output/'scene.json').read_text())
        # Buffer I/O runs in a thread; GL calls always stay on the Tk thread.
        self.pending = None
        def read_buffers():
            try:
                with np.load(scene['render_geometry']['path']) as archive:
                    arrays = {name: archive[name] for name in archive.files}
                if generation == self.generation:
                    self.pending = (generation, scene, arrays, None)
            except Exception as exc:
                if generation == self.generation:
                    self.pending = (generation, scene, None, str(exc))
        threading.Thread(target=read_buffers, daemon=True).start()
        self.process = None
        self.job = self.after(50, lambda: self._accept(generation))

    def _accept(self, generation):
        self.job = None
        if generation != self.generation:
            return
        pending = self.pending
        if pending is None or pending[0] != generation:
            self.job = self.after(50, lambda: self._accept(generation)); return
        _, scene, arrays, error = pending
        self.pending = None
        if error:
            self.status.set('Cannot read preview buffers: '+error); return
        if self.canvas is None:
            self.placeholder.destroy()
            self.canvas = NativeScene(self, self.status, width=640, height=480)
            self.canvas.grid(row=1, column=0, sticky='nsew')
        self.canvas.load(scene, arrays)
        text = f"{len(scene['shapes'])} shapes · {scene['render_geometry']['triangles']:,} triangles · source units: metres"
        if scene['notes']:
            text += '\n'+'; '.join(scene['notes'][:3])
        self.status.set(text)
        self.app.set_status('Embedded 3D preview: '+scene['asset_id'])

    def close(self):
        self.generation += 1
        if self.job is not None:
            self.after_cancel(self.job); self.job = None
        if self.process is not None:
            from .process_control import stop_owned_launch
            threading.Thread(target=stop_owned_launch, args=(self.process,), daemon=False).start()
            self.process = None
        if self.canvas is not None:
            self.canvas.close()
