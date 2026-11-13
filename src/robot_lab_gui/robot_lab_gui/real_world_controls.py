"""Worlds → Real place: import a real area as a world (ROADMAP R6.8).

**Open globe** starts ``robot_lab_utils.real_world_server`` (its own process,
127.0.0.1 only) and opens its Cesium globe in the browser: search a place,
draw a rectangle, choose buildings/roads/trees/water/people/traffic and
import. The same import can be typed here as a latitude/longitude box.
Imports run in child processes (``real_world_import --spec``); this page only
polls their output and the server's status file from the Tk timer.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import tkinter as tk
from tkinter import ttk
import webbrowser

OPTIONS = (('buildings', 'Buildings', True), ('roads', 'Roads', True), ('trees', 'Trees', True),
           ('water', 'Water', True), ('people', 'People (SUMO)', False), ('traffic', 'Traffic (SUMO)', False))


class RealPlacePage(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=4)
        notebook.add(self, text='Real place')
        self.app = app
        self.server = None
        self.job = None
        self.spec_path = None
        self.seen_status = None
        self.result_id = ''
        from robot_lab_utils.terrain_generator import terrain_root
        self.status_file = terrain_root() / '.globe_status.json'
        self.server_var = tk.StringVar(value='Globe closed.')
        self.status_var = tk.StringVar(value='Pick an area on the globe, or type its corners.')
        self.name_var = tk.StringVar(value='my_place')
        self.box_vars = {key: tk.StringVar(value=value) for key, value in
                         (('south', '54.0874'), ('west', '12.1382'), ('north', '54.0901'), ('east', '12.1428'))}
        self.samples_var = tk.StringVar(value='257')
        self.option_vars = {key: tk.BooleanVar(value=default) for key, _label, default in OPTIONS}
        from .lab_tabs import step_card
        for column in range(3):
            self.columnconfigure(column, weight=1, uniform='real')
        globe = step_card(self, 1, 'Pick it on a globe', 'Opens a globe in your browser: search a place, draw '
                          'a rectangle (40 m to 2 km a side) and import it with its buildings, roads, trees, '
                          'water and optional people and traffic.')
        globe.master.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        globe.columnconfigure((0, 1), weight=1)
        # Plain text: Tk 8.6 segfaults rendering characters outside the BMP (a globe emoji, 2026-11-13).
        self.open_button = ttk.Button(globe, text='Open globe', command=self.open_globe, style='Accent.TButton')
        self.open_button.grid(row=0, column=0, columnspan=2, sticky='ew')
        self.close_button = ttk.Button(globe, text='Close globe', command=self.close_globe)
        self.close_button.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(6, 0))
        ttk.Label(globe, textvariable=self.server_var, style='SurfaceBody.TLabel', wraplength=330,
                  justify='left').grid(row=2, column=0, columnspan=2, sticky='ew', pady=(8, 0))

        typed = step_card(self, 2, 'Or type the area', 'Latitude/longitude of the corners (WGS84 degrees).')
        typed.master.grid(row=0, column=1, sticky='nsew', padx=6)
        typed.columnconfigure(1, weight=1)
        ttk.Label(typed, text='World name', style='SurfaceField.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Entry(typed, textvariable=self.name_var).grid(row=0, column=1, sticky='ew', pady=2)
        for row, key in enumerate(('south', 'west', 'north', 'east'), start=1):
            ttk.Label(typed, text=key.capitalize(), style='SurfaceField.TLabel').grid(row=row, column=0, sticky='w')
            ttk.Entry(typed, textvariable=self.box_vars[key]).grid(row=row, column=1, sticky='ew', pady=2)
        ttk.Label(typed, text='Terrain detail', style='SurfaceField.TLabel').grid(row=5, column=0, sticky='w')
        ttk.Combobox(typed, textvariable=self.samples_var, values=('129', '257', '513'), state='readonly',
                     width=8).grid(row=5, column=1, sticky='w', pady=2)
        checks = ttk.Frame(typed, style='Surface.TFrame')
        checks.grid(row=6, column=0, columnspan=2, sticky='w', pady=4)
        for index, (key, label, _default) in enumerate(OPTIONS):
            ttk.Checkbutton(checks, text=label, variable=self.option_vars[key]).grid(
                row=index // 2, column=index % 2, sticky='w', padx=(0, 10))
        self.import_button = ttk.Button(typed, text='▶  Import area', command=self.import_typed)
        self.import_button.grid(row=7, column=0, columnspan=2, sticky='ew', pady=(6, 0))

        result = step_card(self, 3, 'Result', 'Imported places join the world list and the Terrain page\'s saved '
                           'terrains (reload settings, check regeneration, remove).')
        result.master.grid(row=0, column=2, sticky='nsew', padx=(6, 0))
        result.columnconfigure((0, 1), weight=1)
        ttk.Label(result, textvariable=self.status_var, style='SurfaceBody.TLabel', wraplength=330,
                  justify='left').grid(row=0, column=0, columnspan=2, sticky='ew')
        self.launch_button = ttk.Button(result, text='Use in Launch', command=self.use_in_launch)
        self.launch_button.grid(row=1, column=0, sticky='ew', pady=(8, 0), padx=(0, 3))
        self.preview_button = ttk.Button(result, text='Preview 3D', command=self.preview_3d)
        self.preview_button.grid(row=1, column=1, sticky='ew', pady=(8, 0), padx=(3, 0))
        ttk.Label(self, text='Data: elevation from Terrain Tiles (AWS Open Data), features from OpenStreetMap '
                  '(© OpenStreetMap contributors, ODbL) through Overpass, people and traffic from Eclipse SUMO; '
                  'every source is credited in the world\'s manifest. Downloads are cached and saved with the world. '
                  'Experimental: support height and building contact are measured on four simulators; robot '
                  'missions in imported places are not. People and vehicles are Gazebo-only actors.',
                  style='Muted.TLabel', wraplength=1100, justify='left').grid(row=1, column=0, columnspan=3,
                                                                             sticky='w', pady=10)
        self.update_buttons()
        self.after(1000, self.poll)

    # ── globe server ────────────────────────────────────────────────────────
    def open_globe(self):
        if self.server and self.server.poll() is None:
            webbrowser.open(self.server_url)
            return True
        self.status_file.parent.mkdir(parents=True, exist_ok=True)
        self.server = subprocess.Popen([sys.executable, '-m', 'robot_lab_utils.real_world_server', '--port', '0',
                                        '--status', str(self.status_file)], stdout=subprocess.PIPE, text=True,
                                       start_new_session=True)
        line = self.server.stdout.readline()
        try:
            self.server_url = json.loads(line)['url']
        except (ValueError, KeyError):
            self.server_var.set('The globe server did not start: %s' % line.strip())
            return False
        self.server_var.set('Globe open at %s (needs internet for the map and data).' % self.server_url)
        webbrowser.open(self.server_url)
        self.update_buttons()
        return True

    def close_globe(self):
        if self.server and self.server.poll() is None:
            self.server.terminate()
            try:
                self.server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server.kill()
        self.server = None
        self.server_var.set('Globe closed.')
        self.update_buttons()

    # ── typed import ────────────────────────────────────────────────────────
    def request(self):
        request = dict(id=self.name_var.get().strip(), samples=int(self.samples_var.get()))
        for key, variable in self.box_vars.items():
            try:
                request[key] = float(variable.get())
            except ValueError:
                raise ValueError('%s: %r is not a number' % (key, variable.get()))
        request.update({key: variable.get() for key, variable in self.option_vars.items()})
        return request

    def import_typed(self):
        if self.job and self.job.poll() is None:
            return False
        try:
            request = self.request()
            from robot_lab_utils.real_world_import import plan
            plan(request)
        except ValueError as exc:
            self.status_var.set('Not imported: %s' % exc)
            return False
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
            json.dump(request, handle)
            self.spec_path = handle.name
        self.job = subprocess.Popen([sys.executable, '-m', 'robot_lab_utils.real_world_import', '--spec',
                                     self.spec_path], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self.status_var.set('Importing %s (downloads elevation and OpenStreetMap data) …' % request['id'])
        self.update_buttons()
        return True

    def finish_job(self):
        output, _ = self.job.communicate()
        os.unlink(self.spec_path)
        self.job = None
        messages = [json.loads(line) for line in output.splitlines() if line.startswith('{')]
        final = messages[-1] if messages else dict(error='the importer produced no result')
        self.show_result(final.get('manifest'), final.get('error'))

    def show_result(self, manifest, error=None):
        if error or not manifest:
            self.status_var.set('Not imported: %s' % (error or 'unknown error'))
            return
        counts = manifest['counts']
        self.result_id = manifest['id']
        self.status_var.set('%s: %d buildings, %d roads, %d trees, %d water areas; heights 0–%.1f m; spawn at '
                            '(%.1f, %.1f).' % (manifest['id'], counts['buildings'], counts['roads'], counts['trees'],
                                               counts['water'], manifest['elevation']['max_m'],
                                               manifest['spawn']['x'], manifest['spawn']['y']))
        self.app._refresh_maps()
        terrain = getattr(getattr(self.app, 'worlds_tab', None), 'terrain', None)
        if terrain is not None:
            terrain.refresh_existing(select=manifest['id'])

    # ── polling ─────────────────────────────────────────────────────────────
    def poll(self):
        try:
            if self.job and self.job.poll() is not None:
                self.finish_job()
            if self.server and self.server.poll() is not None:
                self.server = None
                self.server_var.set('Globe server stopped.')
            if self.status_file.is_file():
                status = self.status_file.read_text()
                if status != self.seen_status:
                    self.seen_status = status
                    data = json.loads(status)
                    if data.get('state') == 'done':
                        self.show_result(data['result'])
                    elif data.get('state') == 'failed':
                        self.status_var.set('Globe import failed: %s' % data.get('error'))
                    elif data.get('state') == 'running':
                        self.status_var.set('Importing %s from the globe …' % data.get('id'))
        except (OSError, ValueError) as exc:
            self.status_var.set('Status unavailable: %s' % exc)
        self.update_buttons()
        self.after(1000, self.poll)

    def update_buttons(self):
        busy = bool(self.job and self.job.poll() is None)
        self.import_button.state(['disabled'] if busy else ['!disabled'])
        self.close_button.state(['!disabled'] if self.server and self.server.poll() is None else ['disabled'])
        for button in (self.launch_button, self.preview_button):
            button.state(['!disabled'] if self.result_id else ['disabled'])

    def use_in_launch(self):
        if self.result_id not in self.app.map_profiles:
            self.app._refresh_maps()
        if self.result_id in self.app.map_profiles:
            self.app.map_var.set(self.result_id)
            self.app.notebook.select(self.app.launch_tab)
            return True
        return False

    def preview_3d(self):
        if self.result_id in self.app.map_profiles:
            self.app.preview_asset('maps', self.result_id)

    def destroy(self):
        self.close_globe()
        super().destroy()
