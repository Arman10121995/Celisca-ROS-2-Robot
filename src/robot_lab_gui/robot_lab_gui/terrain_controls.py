"""Worlds → Terrain: generate a terrain world, preview it and register it (R6.7).

The work is ``robot_lab_utils.terrain_generator``: one elevation field (from
a seed, or an imported elevation raster with its attribution) becomes the
Gazebo world, the MuJoCo scene, a 2D occupancy grid and a manifest under
``$ROBOT_LAB_RUNTIME_ROOT/terrain/<id>/``. Generation runs in its own process
(``python3 -m robot_lab_utils.terrain_generator --spec``), never a thread: a
garbage collection in a worker thread can finalise a Tk image and block on the
Tcl interpreter (seen 2026-11-13). The result is registered as a map and
appears in Launch after a refresh.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


FIELDS = (  # (key, label, kind, default) — defaults mirror terrain_generator.DEFAULTS
    ('size_m', 'Area side (m)', float, 40.0),
    ('relief_m', 'Height range (m)', float, 2.0),
    ('seed', 'Seed', int, 7),
    ('smoothness', 'Smoothness (1.5–4.5)', float, 3.0),
    ('min_wavelength_m', 'Smallest feature (m)', float, 4.0),
    ('roughness', 'Roughness (0–1, named styles)', float, 0.5),
)
PROCEDURAL_ONLY = ('seed', 'smoothness', 'min_wavelength_m', 'roughness')
ROBOT_FIELDS = (
    ('spawn_x', 'Spawn X (m)', float, 0.0),
    ('spawn_y', 'Spawn Y (m)', float, 0.0),
    ('spawn_pad_m', 'Flat spawn pad radius (m)', float, 1.5),
    ('max_slope_deg', 'Drivable slope limit (°)', float, 15.0),
    ('max_step_m', 'Drivable step limit (m)', float, 0.08),
    ('occupancy_resolution_m', 'Grid resolution (m)', float, 0.1),
    ('friction', 'Surface friction', float, 1.0),
)
RASTER_FIELDS = (
    ('source_metres_per_pixel', 'Source metres per pixel', float, 1.0),
    ('attribution', 'Attribution (required)', str, ''),
    ('license', 'License / terms (required)', str, ''),
)


class TerrainPage(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=4)
        notebook.add(self, text='Terrain')
        self.app = app
        self.worker = None
        self.result = None
        self.images = []
        self.name_var = tk.StringVar(value='my_terrain')
        self.provider_var = tk.StringVar(value='procedural')
        self.samples_var = tk.StringVar(value='129')
        self.style_var = tk.StringVar(value='spectral')
        self.raster_var = tk.StringVar()
        self.status_var = tk.StringVar(value='Choose a source and area, then Generate.')
        self.existing_var = tk.StringVar()
        self.vars = {key: (tk.StringVar(value=str(default)), kind)
                     for key, _label, kind, default in FIELDS + ROBOT_FIELDS + RASTER_FIELDS}
        from .lab_tabs import step_card
        for column in range(3):
            self.columnconfigure(column, weight=1, uniform='terrain')
        source = step_card(self, 1, 'Source and area', 'Procedural from a seed, or an elevation raster you '
                           'downloaded (GeoTIFF, .npy or 16-bit PNG) with its attribution.')
        source.master.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        source.columnconfigure(1, weight=1)
        ttk.Label(source, text='Name', style='SurfaceField.TLabel').grid(row=0, column=0, sticky='w')
        self.name_entry = ttk.Entry(source, textvariable=self.name_var)
        self.name_entry.grid(row=0, column=1, sticky='ew', pady=2)
        providers = ttk.Frame(source, style='Surface.TFrame')
        providers.grid(row=1, column=0, columnspan=2, sticky='w', pady=4)
        for column, (value, text) in enumerate((('procedural', 'Procedural'), ('raster', 'Elevation raster'))):
            ttk.Radiobutton(providers, text=text, value=value, variable=self.provider_var,
                            command=self.provider_changed).grid(row=0, column=column, padx=(0, 10))
        ttk.Label(source, text='Samples per side', style='SurfaceField.TLabel').grid(row=2, column=0, sticky='w')
        ttk.Combobox(source, textvariable=self.samples_var, values=('33', '65', '129', '257', '513'),
                     state='readonly', width=8).grid(row=2, column=1, sticky='w', pady=2)
        from robot_lab_utils.terrain_generator import STYLES
        style = (ttk.Label(source, text='Style', style='SurfaceField.TLabel'),
                 ttk.Combobox(source, textvariable=self.style_var, values=STYLES, state='readonly', width=14))
        style[0].grid(row=3, column=0, sticky='w')
        style[1].grid(row=3, column=1, sticky='w', pady=2)
        self.procedural_rows = [style]
        for row, (key, label, _kind, _default) in enumerate(FIELDS, start=4):
            widgets = (ttk.Label(source, text=label, style='SurfaceField.TLabel'),
                       ttk.Entry(source, textvariable=self.vars[key][0], width=10))
            widgets[0].grid(row=row, column=0, sticky='w')
            widgets[1].grid(row=row, column=1, sticky='ew', pady=2)
            if key in PROCEDURAL_ONLY:
                self.procedural_rows.append(widgets)
        self.raster_rows = []
        row = 4 + len(FIELDS)
        browse = ttk.Frame(source, style='Surface.TFrame')
        browse.columnconfigure(0, weight=1)
        ttk.Entry(browse, textvariable=self.raster_var).grid(row=0, column=0, sticky='ew')
        ttk.Button(browse, text='Browse…', command=self.browse).grid(row=0, column=1, padx=(4, 0))
        widgets = (ttk.Label(source, text='Raster file', style='SurfaceField.TLabel'), browse)
        widgets[0].grid(row=row, column=0, sticky='w')
        widgets[1].grid(row=row, column=1, sticky='ew', pady=2)
        self.raster_rows.append(widgets)
        for offset, (key, label, _kind, _default) in enumerate(RASTER_FIELDS, start=1):
            widgets = (ttk.Label(source, text=label, style='SurfaceField.TLabel'),
                       ttk.Entry(source, textvariable=self.vars[key][0]))
            widgets[0].grid(row=row + offset, column=0, sticky='w')
            widgets[1].grid(row=row + offset, column=1, sticky='ew', pady=2)
            self.raster_rows.append(widgets)

        robot = step_card(self, 2, 'Robot use', 'Where robots start, and what the 2D navigation grid '
                          'treats as blocked (slopes and steps a ground robot cannot take).')
        robot.master.grid(row=0, column=1, sticky='nsew', padx=6)
        robot.columnconfigure(1, weight=1)
        for row, (key, label, _kind, _default) in enumerate(ROBOT_FIELDS):
            ttk.Label(robot, text=label, style='SurfaceField.TLabel').grid(row=row, column=0, sticky='w')
            ttk.Entry(robot, textvariable=self.vars[key][0], width=10).grid(row=row, column=1, sticky='ew', pady=2)

        result = step_card(self, 3, 'Generate and use', 'Writes the world for all four simulators, the grid and a '
                           'manifest on the SSD, then adds it to the world list.')
        result.master.grid(row=0, column=2, sticky='nsew', padx=(6, 0))
        result.columnconfigure((0, 1), weight=1)
        self.generate_button = ttk.Button(result, text='▶  Generate terrain', command=self.generate,
                                          style='Accent.TButton')
        self.generate_button.grid(row=0, column=0, columnspan=2, sticky='ew')
        self.preview = ttk.Frame(result, style='Surface.TFrame')
        self.preview.grid(row=1, column=0, columnspan=2, sticky='ew', pady=6)
        ttk.Label(result, textvariable=self.status_var, style='SurfaceBody.TLabel', wraplength=330,
                  justify='left').grid(row=2, column=0, columnspan=2, sticky='ew')
        self.launch_button = ttk.Button(result, text='Use in Launch', command=self.use_in_launch)
        self.launch_button.grid(row=3, column=0, sticky='ew', pady=(6, 0), padx=(0, 3))
        self.preview_button = ttk.Button(result, text='Preview 3D', command=self.preview_3d)
        self.preview_button.grid(row=3, column=1, sticky='ew', pady=(6, 0), padx=(3, 0))

        saved = step_card(self, 4, 'Saved terrains', 'Reload a terrain\'s settings, confirm it regenerates '
                          'byte for byte, or remove it from the world list (files stay on the SSD).')
        saved.master.grid(row=1, column=0, columnspan=3, sticky='ew', pady=(12, 0))
        saved.columnconfigure(0, weight=1)
        self.existing_combo = ttk.Combobox(saved, textvariable=self.existing_var, state='readonly')
        self.existing_combo.grid(row=0, column=0, sticky='ew', padx=(0, 6))
        for column, (text, command) in enumerate((('Load settings', self.load_settings),
                                                  ('Check regeneration', self.check_regeneration),
                                                  ('Remove from list', self.remove)), start=1):
            ttk.Button(saved, text=text, command=command).grid(row=0, column=column, padx=3)
        ttk.Label(self, text='Generated terrain is experimental: the support height is measured on Gazebo, MuJoCo, '
                  'PyBullet and Isaac, but driving, walking and flight on each terrain need their own runs. '
                  'An imported raster keeps its provider terms in the manifest; no download or token is used here.',
                  style='Muted.TLabel', wraplength=1100, justify='left').grid(row=2, column=0, columnspan=3,
                                                                             sticky='w', pady=10)
        self.provider_changed()
        self.refresh_existing()
        self.update_buttons()

    # ── inputs ──────────────────────────────────────────────────────────────
    def provider_changed(self):
        raster = self.provider_var.get() == 'raster'
        for widgets in self.procedural_rows:
            for widget in widgets:
                widget.grid_remove() if raster else widget.grid()
        for widgets in self.raster_rows:
            for widget in widgets:
                widget.grid() if raster else widget.grid_remove()

    def browse(self):
        path = filedialog.askopenfilename(title='Elevation raster', filetypes=[
            ('Elevation rasters', '*.tif *.tiff *.npy *.png'), ('All files', '*')])
        if path:
            self.raster_var.set(path)

    def spec(self):
        """The generator spec from the fields; ValueError names the bad field."""
        spec = dict(id=self.name_var.get().strip(), provider=self.provider_var.get(),
                    samples=int(self.samples_var.get()))
        if spec['provider'] == 'procedural':
            spec['style'] = self.style_var.get()
        keys = [key for key, *_ in FIELDS + ROBOT_FIELDS]
        if spec['provider'] == 'raster':
            keys = [key for key in keys if key not in PROCEDURAL_ONLY]
            keys += [key for key, *_ in RASTER_FIELDS]
            spec['raster_path'] = self.raster_var.get().strip()
        for key in keys:
            variable, kind = self.vars[key]
            text = variable.get().strip()
            try:
                spec[key] = kind(text) if kind is not str else text
            except ValueError:
                raise ValueError('%s: %r is not a number' % (key, text))
        return spec

    # ── generation ──────────────────────────────────────────────────────────
    def generate(self):
        if self.busy():
            return False
        try:
            spec = self.spec()
            from robot_lab_utils.terrain_generator import validate
            validate(spec)
        except ValueError as exc:
            self.status_var.set('Not generated: %s' % exc)
            return False
        self.status_var.set('Generating %s …' % spec['id'])
        self.result = None
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
            json.dump(spec, handle)
            self.spec_path = handle.name
        self.worker = subprocess.Popen([sys.executable, '-m', 'robot_lab_utils.terrain_generator',
                                        '--spec', self.spec_path], stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
        self.update_buttons()
        self.after(100, self.poll)
        return True

    def busy(self):
        return bool(self.worker and self.worker.poll() is None)

    def collect(self):
        """The finished process's result: ('ok', manifest) or ('error', message)."""
        output, errors = self.worker.communicate()
        os.unlink(self.spec_path)
        try:
            data = json.loads(output.strip().splitlines()[-1])
        except (ValueError, IndexError):
            return 'error', (errors.strip().splitlines() or ['generator exited %s' % self.worker.returncode])[-1]
        return ('ok', data['manifest']) if 'manifest' in data else ('error', data.get('error', 'unknown error'))

    def poll(self):
        if self.busy():
            self.after(100, self.poll)
            return
        if self.worker and self.result is None:
            self.result = self.collect()
        self.update_buttons()
        if not self.result:
            return
        kind, value = self.result
        if kind == 'error':
            self.status_var.set('Not generated: %s' % value)
            return
        manifest = value
        self.status_var.set('%s: %.1f m × %.1f m, heights 0–%.2f m (%.0f mm steps), %.1f %% of the grid blocked, '
                            'spawn at %.2f m. Saved in %s.' % (
                                manifest['id'], manifest['inputs']['size_m'], manifest['inputs']['size_m'],
                                manifest['elevation']['max_m'], manifest['raster']['quantisation_m'] * 1000,
                                manifest['occupancy']['blocked_fraction'] * 100, manifest['elevation']['spawn_m'],
                                self.folder(manifest['id'])))
        self.show_preview(manifest['id'])
        self.app._refresh_maps()
        self.refresh_existing(select=manifest['id'])

    def folder(self, name):
        from robot_lab_utils.terrain_generator import terrain_root
        return terrain_root() / name

    def show_preview(self, name):
        from PIL import Image, ImageTk
        for child in self.preview.winfo_children():
            child.destroy()
        self.images = []
        folder = self.folder(name)
        for column, (path, title) in enumerate(((folder / 'preview.png', 'Terrain'),
                                                (folder / 'maps' / 'map.pgm', 'Drivable (black = blocked)'))):
            if not path.is_file():
                continue
            with Image.open(path) as image:
                image = image.convert('RGB').resize((150, 150), Image.NEAREST)
            photo = ImageTk.PhotoImage(image, master=self)
            self.images.append(photo)
            ttk.Label(self.preview, image=photo).grid(row=0, column=column, padx=3)
            ttk.Label(self.preview, text=title, style='SurfaceBody.TLabel').grid(row=1, column=column)

    # ── saved terrains ──────────────────────────────────────────────────────
    def refresh_existing(self, select=None):
        from robot_lab_utils.terrain_generator import registered_profiles
        names = sorted(registered_profiles())
        self.existing_combo.configure(values=names)
        if select in names:
            self.existing_var.set(select)
        elif self.existing_var.get() not in names:
            self.existing_var.set(names[0] if names else '')
        self.update_buttons()

    def selected(self):
        name = self.existing_var.get()
        return name if name and (self.folder(name) / 'manifest.json').is_file() else None

    def load_settings(self):
        name = self.selected()
        if not name:
            return False
        inputs = json.loads((self.folder(name) / 'manifest.json').read_text())['inputs']
        self.name_var.set(name + '_2')
        self.provider_var.set(inputs['provider'])
        self.samples_var.set(str(inputs['samples']))
        self.style_var.set(inputs.get('style', 'spectral'))
        self.raster_var.set(inputs.get('raster_path', ''))
        for key, (variable, _kind) in self.vars.items():
            if key in inputs:
                variable.set(str(inputs[key]))
        self.provider_changed()
        self.show_preview(name)
        self.status_var.set('Loaded the settings of %s; change the name before generating.' % name)
        return True

    def check_regeneration(self):
        name = self.selected()
        if not name:
            return False
        from robot_lab_utils.terrain_generator import regenerate
        try:
            result = regenerate(self.folder(name))
        except Exception as exc:
            self.status_var.set('%s cannot be checked: %s' % (name, exc))
            return False
        different = [path for path, same in result.items() if not same]
        self.status_var.set('%s regenerates byte for byte from its manifest (%d files).' % (name, len(result))
                            if not different else '%s differs from its manifest: %s' % (name, ', '.join(different)))
        return not different

    def remove(self):
        name = self.selected()
        if not name or not messagebox.askyesno('Remove terrain', 'Remove %s from the world list? Its files stay in '
                                               '%s.' % (name, self.folder(name)), parent=self):
            return False
        from robot_lab_utils.terrain_generator import unregister
        unregister(name)
        self.app._refresh_maps()
        self.refresh_existing()
        self.status_var.set('%s removed from the world list.' % name)
        return True

    def use_in_launch(self):
        name = self.existing_var.get()
        if name not in self.app.map_profiles:
            self.app._refresh_maps()
        if name not in self.app.map_profiles:
            self.status_var.set('Generate a terrain first.')
            return False
        self.app.map_var.set(name)
        self.app.notebook.select(self.app.launch_tab)
        return True

    def preview_3d(self):
        name = self.existing_var.get()
        if name in self.app.map_profiles:
            self.app.preview_asset('maps', name)

    def update_buttons(self):
        busy = self.busy()
        self.generate_button.state(['disabled'] if busy else ['!disabled'])
        has = bool(self.existing_var.get())
        for button in (self.launch_button, self.preview_button):
            button.state(['!disabled'] if has and not busy else ['disabled'])
