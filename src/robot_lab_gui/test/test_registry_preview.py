"""Exercise grouped selectors and registry preview routing through real Tk."""
from unittest.mock import Mock
import os

import pytest

from test_command_autofill import app, select, displayed_command  # noqa: F401
from robot_lab_gui.lab_tabs import RegistryTab


pytestmark = [pytest.mark.integration,
              pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb or a display')]


def test_native_viewport_renders_in_tk_and_changes_actual_framebuffer(tmp_path):
    pytest.importorskip('OpenGL')
    pytest.importorskip('trimesh')
    import tkinter as tk
    import numpy as np
    from robot_lab_gui.native_viewer import NativeScene
    from robot_lab_utils.asset_preview import scene_bounds, prepare_render_geometry
    import time
    root = tk.Tk()
    root.geometry('400x300')
    viewport = NativeScene(root, tk.StringVar(), width=400, height=300)
    viewport.pack(fill='both', expand=True)
    shape = dict(type='box', size=[1., 1., 1.], position=[0., 0., 0.],
                 orientation=[1., 0., 0., 0.], rgba=[1., .3, .1, 1.])
    scene = dict(shapes=[shape], bounds=scene_bounds([shape]))
    scene['render_geometry'] = prepare_render_geometry(scene, tmp_path/'geometry.npz')
    with np.load(tmp_path/'geometry.npz') as archive:
        arrays = {name: archive[name] for name in archive.files}
    try:
        viewport.load(scene, arrays)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not viewport.draws:
            root.update(); time.sleep(.01)
        assert viewport.error is None
        before = viewport.pixels()
        assert len(np.unique(before.reshape(-1, 3), axis=0)) > 3
        viewport.event_generate('<ButtonPress-1>', x=100, y=100)
        viewport.event_generate('<B1-Motion>', x=170, y=140)
        root.update()
        after = viewport.pixels()
        assert np.mean(np.any(before != after, axis=2)) > .01
        viewport.destroy()
        assert viewport.context is None and not viewport.buffers
    finally:
        root.destroy()


def test_complete_robot_selection_uses_the_authored_assembly_and_retains_variants(app):
    if 'asset_r2_description_r2c6' not in app.robot_profiles:
        pytest.skip('R2 source assets are not installed on this host')
    assert 'nasa_r2' in app.robot_combo['values']
    assert 'asset_r2_description_r2_left_forearm' not in app.robot_combo['values']
    select(app, app.robot_combo, 'nasa_r2')
    assert app.robot_var.get() == 'asset_r2_description_r2c6'
    assert 'robot_model:=asset_r2_description_r2c6' in displayed_command(app)
    assert 'asset_r2_description_r2_left_gripper' not in app.robot_variant_combo['values']
    select(app, app.robot_variant_combo, 'asset_r2_description_r2b')
    assert app.robot_family_var.get() == 'nasa_r2'
    assert 'robot_model:=asset_r2_description_r2b' in displayed_command(app)


def test_world_variant_preserves_exact_world_command_and_saved_selection(app):
    select(app, app.map_combo, 'celisca_floor_2')
    select(app, app.map_variant_combo, 'celisca_floor_2_furniture')
    assert app.map_family_var.get() == 'celisca_floor_2'
    assert 'map_name:=celisca_floor_2_furniture' in displayed_command(app)
    app.map_var.set('celisca_f1_actor')
    app._update_from_selection()
    assert app.map_family_var.get() == 'celisca_floor_1'
    assert app.map_var.get() == 'celisca_f1_actor'
    assert 'map_name:=celisca_f1_actor' in displayed_command(app)


def test_installed_dataset_rooms_hide_duplicate_sources_but_keep_exact_world_commands(app):
    if 'dataset_room2_world_dynamic' not in app.map_profiles:
        pytest.skip('Dataset room worlds are not installed on this host')
    for room in (2, 3, 4):
        family = f'dataset_room{room}'
        dynamic = family+'_world_dynamic'
        assert family in app.map_combo['values']
        assert dynamic not in app.map_combo['values']
        select(app, app.map_combo, family)
        assert app.map_var.get() == family
        select(app, app.map_variant_combo, dynamic)
        assert app.map_family_var.get() == family
        assert f'map_name:={dynamic}' in displayed_command(app)


def test_family_picks_compatible_source_without_borrowing_native_panda_controller(app):
    if 'asset_franka_panda_panda' not in app.robot_profiles:
        pytest.skip('Panda source assets are not installed on this host')
    app.simulator_var.set('pybullet')
    select(app, app.robot_combo, 'franka_panda')
    assert app.robot_var.get() == 'asset_franka_panda_panda'
    assert app.simulator_var.get() == 'pybullet'
    command = displayed_command(app)
    assert 'robot_model:=asset_franka_panda_panda' in command
    assert not any(part.startswith(('arm_control:=', 'arm_planning:=')) for part in command)


def test_registry_nested_components_are_previewable_and_cannot_replace_launch(app):
    tab = RegistryTab(app.notebook, app)
    if 'asset_r2_description_r2_left_forearm' not in tab.rows:
        pytest.skip('R2 component is not installed on this host')
    assert tab.tree.parent('asset_r2_description_r2_left_forearm') == 'family:nasa_r2'
    before = (app.robot_var.get(), app.map_var.get(), app.command_var.get())
    app.preview_asset = Mock()
    tab.tree.selection_set('asset_r2_description_r2_left_forearm')
    tab._show_details()
    assert 'disabled' not in tab.preview_button.state()
    assert 'disabled' in tab.launch_button.state()
    tab.preview_button.invoke()
    assert app.preview_asset.call_args.args[:2] == ('robots', 'asset_r2_description_r2_left_forearm')
    tab._open_selected()
    assert (app.robot_var.get(), app.map_var.get(), app.command_var.get()) == before
    tab.type_var.set('Algorithms'); tab._refresh_tree()
    algorithm = tab.tree.get_children()[0]
    tab.tree.selection_set(algorithm); tab._show_details()
    assert 'disabled' in tab.preview_button.state()
    assert 'disabled' in tab.launch_button.state()


def test_registry_core_alias_previews_actual_installed_policy_model(app):
    tab = RegistryTab(app.notebook, app)
    assert tab.tree.parent('go2') == 'family:unitree_go2'
    app.preview_asset = Mock()
    tab.tree.selection_set('go2'); tab._show_details()
    tab.preview_button.invoke()
    assert app.preview_asset.call_args.args[:2] == ('robots', 'unitree_go2')
