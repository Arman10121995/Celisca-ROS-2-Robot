"""Worlds → Terrain page through real Tk: inputs, generation, reuse, launch hand-off."""
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytestmark = pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Actual Tk requires Xvfb or a display')


@pytest.fixture
def page(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    from robot_lab_gui.terrain_controls import TerrainPage
    app = tk.Tk()
    app.geometry('1400x900')
    app.notebook = ttk.Notebook(app)
    app.notebook.pack(fill='both', expand=True)
    app.launch_tab = ttk.Frame(app.notebook)
    app.notebook.add(app.launch_tab, text='Launch')
    pages = ttk.Notebook(app.notebook)
    app.notebook.add(pages, text='Worlds')
    app.map_var = tk.StringVar(value='nav_empty')
    app.map_profiles = {}

    def refresh():
        from robot_lab_utils.terrain_generator import registered_profiles
        app.map_profiles = dict(registered_profiles())
    app._refresh_maps = Mock(side_effect=refresh)
    app.preview_asset = Mock()
    widget = TerrainPage(pages, app)
    app.update()
    try:
        yield widget
    finally:
        for job in app.tk.call('after', 'info'):
            app.tk.call('after', 'cancel', job)
        app.destroy()


def wait(page):
    """Until the generator process has ended and poll() has shown its result."""
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and (page.busy() or page.status_var.get().startswith('Generating')):
        page.update()
        time.sleep(.02)


def test_invalid_fields_explain_and_generate_nothing(page, tmp_path):
    page.name_var.set('Bad Name')
    assert not page.generate()
    assert 'Not generated' in page.status_var.get()
    page.name_var.set('ok_name')
    page.vars['relief_m'][0].set('tall')
    assert not page.generate() and 'relief_m' in page.status_var.get()
    assert not (tmp_path / 'terrain').exists()


def test_generate_registers_previews_checks_and_hands_off_to_launch(page, tmp_path):
    page.name_var.set('gui_hills')
    page.style_var.set('crater')
    page.samples_var.set('65')
    page.vars['size_m'][0].set('20')
    page.generate_button.invoke()
    wait(page)
    assert 'gui_hills: 20.0 m' in page.status_var.get(), page.status_var.get()
    assert (tmp_path / 'terrain' / 'gui_hills' / 'manifest.json').is_file()
    assert page.app._refresh_maps.called and 'gui_hills' in page.app.map_profiles
    assert len(page.images) == 2                                   # terrain and drivable grid
    assert page.existing_var.get() == 'gui_hills'
    assert page.check_regeneration() and 'byte for byte' in page.status_var.get()
    assert page.load_settings() and page.name_var.get() == 'gui_hills_2' and page.samples_var.get() == '65'
    assert page.style_var.get() == 'crater'
    page.launch_button.invoke()
    assert page.app.map_var.get() == 'gui_hills'
    assert page.app.notebook.select() == str(page.app.launch_tab)
    page.preview_button.invoke()
    page.app.preview_asset.assert_called_once_with('maps', 'gui_hills')


def test_raster_source_shows_its_fields_and_requires_attribution(page, tmp_path):
    import numpy as np
    page.provider_var.set('raster')
    page.provider_changed()
    page.update()
    assert all(widget.winfo_manager() == '' for widgets in page.procedural_rows for widget in widgets)
    assert all(widget.winfo_manager() == 'grid' for widgets in page.raster_rows for widget in widgets)
    path = tmp_path / 'dem.npy'
    np.save(path, np.random.RandomState(0).rand(80, 80) * 5.)
    page.raster_var.set(str(path))
    page.name_var.set('gui_dem')
    page.vars['source_metres_per_pixel'][0].set('1')
    page.vars['size_m'][0].set('60')
    page.samples_var.set('33')
    assert not page.generate() and 'attribution' in page.status_var.get()
    page.vars['attribution'][0].set('Synthetic test DEM')
    page.vars['license'][0].set('CC0')
    assert page.generate()
    wait(page)
    assert 'gui_dem:' in page.status_var.get(), page.status_var.get()
