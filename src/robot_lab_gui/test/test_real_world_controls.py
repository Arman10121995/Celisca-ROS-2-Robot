"""Worlds → Real place page and the globe server, without network."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import tkinter as tk
from tkinter import ttk
from unittest.mock import Mock
import urllib.request

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_globe_server_serves_the_page_and_validates_imports(tmp_path):
    server = subprocess.Popen([sys.executable, '-m', 'robot_lab_utils.real_world_server', '--port', '0',
                               '--status', str(tmp_path / 'status.json')], stdout=subprocess.PIPE, text=True)
    try:
        url = json.loads(server.stdout.readline())['url']
        page = urllib.request.urlopen(url, timeout=10).read().decode()
        assert 'Cesium' in page and 'Import this area' in page
        assert json.loads(urllib.request.urlopen(url + 'status', timeout=10).read())['state'] == 'idle'
        request = urllib.request.Request(url + 'import', data=b'{"south": 1}', method='POST',
                                         headers={'Content-Type': 'application/json'})
        reply = json.loads(urllib.request.urlopen(request, timeout=10).read())
        assert reply == dict(ok=False, error='a world name is required')
    finally:
        server.terminate()
        server.wait(timeout=10)


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Actual Tk requires Xvfb or a display')
def test_page_validates_typed_areas_and_shows_globe_results(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    from robot_lab_gui.real_world_controls import RealPlacePage
    app = tk.Tk()
    try:
        app.notebook = ttk.Notebook(app)
        app.notebook.pack()
        app.launch_tab = ttk.Frame(app.notebook)
        app.notebook.add(app.launch_tab, text='Launch')
        pages = ttk.Notebook(app.notebook)
        app.notebook.add(pages, text='Worlds')
        app.map_var = tk.StringVar(value='nav_empty')
        app.map_profiles = {'my_place': {}}
        app._refresh_maps = Mock()
        app.preview_asset = Mock()
        page = RealPlacePage(pages, app)
        page.box_vars['north'].set('54.0874')                    # north == south
        assert not page.import_typed() and 'south < north' in page.status_var.get()
        page.box_vars['north'].set('not a number')
        assert not page.import_typed() and 'north' in page.status_var.get()
        assert page.launch_button.instate(['disabled'])
        page.status_file.parent.mkdir(parents=True, exist_ok=True)
        page.status_file.write_text(json.dumps(dict(id='my_place', state='done', error='', result=dict(
            id='my_place', counts=dict(buildings=3, roads=2, trees=1, water=0), elevation=dict(max_m=4.),
            spawn=dict(x=1., y=2.)))))
        page.poll()
        assert '3 buildings' in page.status_var.get() and app._refresh_maps.called
        page.launch_button.invoke()
        assert app.map_var.get() == 'my_place' and app.notebook.select() == str(app.launch_tab)
    finally:
        for job in app.tk.call('after', 'info'):
            app.tk.call('after', 'cancel', job)
        app.destroy()
