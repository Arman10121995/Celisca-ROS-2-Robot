"""Vacuum workspace: cleaning app map tools, requests and status display.

The node is replaced by recorded commands and fed messages, so no ROS graph
is needed."""
import json
import os
from types import SimpleNamespace

import numpy as np
import pytest

from test_command_autofill import app  # noqa: F401

pytestmark = [pytest.mark.integration,
              pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb or a display')]


def msg(data):
    return SimpleNamespace(data=json.dumps(data))


@pytest.fixture
def tab(app):  # noqa: F811
    from robot_lab_gui.cleaning_tab import VacuumTab
    tab = VacuumTab(app.notebook, app)
    tab._ensure_ros = lambda: False
    sent = []
    tab._send = lambda cmd: sent.append(cmd) or True
    tab.sent = sent
    app.show_tab('Vacuum')
    app.update()
    # 8 x 4 m map, 0.05 m cells, two rooms split at x = 4 m.
    data = np.zeros((80, 160), dtype=np.int16)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    data[:, 80] = 100
    data[30:50, 80] = 0
    rooms = np.zeros_like(data)
    rooms[1:-1, 1:80] = 1
    rooms[1:-1, 81:-1] = 2
    tab.map_info = (160, 80, 0.05, 0.0, 0.0)
    tab.map_data = data
    tab.rooms_grid = rooms
    tab.coverage_grid = np.zeros(data.shape, dtype=bool)
    tab.coverage_grid[10:20, 10:40] = True
    tab._on_json('layout', msg({
        'map': 'unit', 'rooms': {'1': {'name': 'Kitchen', 'area_m2': 15.2, 'anchor': [2.0, 2.0]},
                                 '2': {'name': 'Hall', 'area_m2': 15.0, 'anchor': [6.0, 2.0], 'suction': 'max'}},
        'room_order': [2, 1], 'no_go': [], 'walls': [], 'zones': [], 'dock': [1.0, 1.0, 0.0],
        'settings': {'charge_resume': True, 'auto_empty': False, 'low_battery_pct': 25,
                     'resume_battery_pct': 70, 'battery_time_scale': 10.0, 'speed_scale': 1.0,
                     'dnd': {'enabled': True, 'start': '23:00', 'end': '07:00'}},
        'schedule': [{'id': 'a', 'enabled': True, 'days': [0, 2], 'time': '09:30', 'mode': 'rooms',
                      'rooms': [1], 'suction': 'eco', 'passes': 1}],
        'power_modes': {'eco': {}, 'standard': {}, 'turbo': {}, 'max': {}},
        'robot': {'radius': 0.165, 'cleaning_width': 0.26}}))
    tab._render()
    app.update()
    yield tab
    tab.close()


def test_vacuum_tab_launches_the_cleaning_system(tab):
    command = tab.launch_command()
    assert command[:4] == ['ros2', 'launch', 'robot_lab_bringup', 'simulated_room_vacuum.launch.py']
    assert 'controller:=cleaning' in command and 'robot_model:=oomwoo' in command
    assert 'map_name:=small_house' in command and 'mode:=nav' in command
    from ros2launch.api.api import parse_launch_arguments
    assert dict(parse_launch_arguments(command[4:]))['vacuum_detector'] == 'segmentation'
    assert tab.node_command()[:4] == ['ros2', 'run', 'robot_lab_vacuum_cleaning', 'vacuum_cleaner']


def test_detector_launch_selection_and_missing_checkpoint(tab, tmp_path):
    tab.vacuum_detector_var.set('learned')
    launched = []
    tab.app.start_launch_command = lambda command: launched.append(command)
    tab._launch()
    assert not launched and 'checkpoint' in tab.message_var.get()
    checkpoint = tmp_path / 'candidate.pt'
    checkpoint.touch()
    tab.vacuum_detector_model_var.set(str(checkpoint))
    tab._launch()
    assert 'vacuum_detector:=learned' in launched[0]
    assert 'vacuum_detector_model:=%s' % checkpoint in launched[0]


def test_object_gallery_displays_real_crop_and_is_bounded(tab):
    import base64
    from io import BytesIO
    from PIL import Image
    buffer = BytesIO()
    Image.new('RGB', (30, 20), color='red').save(buffer, format='JPEG')
    crop = {'encoding': 'jpeg', 't': 3.0,
            'data': base64.b64encode(buffer.getvalue()).decode('ascii')}
    objects = [{'class': 'shoe', 'x': float(i), 'y': 2., 'snapshot': crop} for i in range(40)]
    for i in range(0, 40, 10):
        tab._on_object_observations(msg({'detector': 'segmentation', 'ideal': True, 't': 3.,
                                         'objects': objects[i:i+10]}))
    assert len(tab.object_observations) == 32
    tab.object_tree.selection_set('0')
    tab._show_object_snapshot()
    assert tab.object_photo is not None
    assert 'ideal simulator' in tab.detector_status_var.get()
    assert 'camera time 3.0' in tab.object_detail_var.get()
    tab._clear_object_gallery()
    assert not tab.object_observations and tab.object_photo is None


def test_layout_fills_rooms_schedule_and_settings(tab):
    rows = [tab.room_tree.item(i, 'values') for i in tab.room_tree.get_children()]
    assert [r[1] for r in rows] == ['Hall', 'Kitchen']   # room order from the node
    assert rows[0][3] == 'max'
    sched = tab.schedule_tree.item('a', 'values')
    assert sched[1] == '09:30' and sched[2] == 'Mon Wed' and sched[3] == 'Kitchen'
    assert not tab.auto_empty_var.get() and tab.low_var.get() == 25 and tab.scale_var.get() == '10'
    assert tab.dnd_var.get() and tab.dnd_start_var.get() == '23:00'


def test_map_renders_and_converts_coordinates(tab):
    assert tab.photo is not None and tab.view is not None
    cx, cy = tab.to_canvas(3.0, 1.5)
    x, y = tab.to_world(cx, cy)
    assert abs(x - 3.0) < 1e-6 and abs(y - 1.5) < 1e-6
    rgb = tab.compose_rgb()
    assert tuple(rgb[15, 20]) != tuple(rgb[60, 20])   # cleaned cells are tinted


def test_room_selection_builds_an_ordered_rooms_job(tab):
    tab.apply_tool('select', (6.0, 2.0), (6.0, 2.0), moved=False)
    tab.apply_tool('select', (2.0, 2.0), (2.0, 2.0), moved=False)
    assert tab.selected_rooms == [2, 1] and tab.mode_var.get() == 'rooms'
    assert 'Hall → Kitchen' in tab.selection_var.get()
    tab.suction_var.set('turbo')
    tab.passes_var.set(2)
    tab._start()
    assert tab.sent[-1] == {'cmd': 'start', 'mode': 'rooms', 'rooms': [2, 1], 'suction': 'turbo',
                            'passes': 2, 'edge': True}
    tab.apply_tool('select', (6.0, 2.0), (6.0, 2.0), moved=False)
    assert tab.selected_rooms == [1]


def test_zone_no_go_wall_and_erase_tools(tab):
    tab.apply_tool('zone', (0.5, 0.5), (2.5, 2.0))
    assert tab.sent[-1]['cmd'] == 'set_layout' and tab.sent[-1]['zones'][0]['rect'] == [0.5, 0.5, 2.5, 2.0]
    assert tab.mode_var.get() == 'zones'
    tab._start()
    assert tab.sent[-1]['mode'] == 'zones' and len(tab.sent[-1]['zones']) == 1
    tab.apply_tool('nogo', (5.0, 1.0), (6.0, 2.0))
    assert tab.sent[-1]['no_go'][0]['polygon'][2] == [6.0, 2.0]
    tab.apply_tool('wall', (4.0, 1.0), (4.0, 3.0))
    assert tab.sent[-1]['walls'][0]['p2'] == [4.0, 3.0]
    tab.apply_tool('zone', (1.0, 1.0), (1.1, 1.1))   # too small: ignored
    assert len(tab.layout['zones']) == 1
    tab.apply_tool('erase', (4.05, 2.0), (4.05, 2.0), moved=False)
    assert tab.sent[-1]['walls'] == [] and len(tab.sent[-1]['zones']) == 1
    tab.apply_tool('erase', (1.0, 1.0), (1.0, 1.0), moved=False)
    assert tab.sent[-1]['zones'] == []


def test_spot_dock_and_split_tools(tab):
    tab.apply_tool('spot', (3.0, 2.0), (3.0, 2.0), moved=False)
    assert tab.mode_var.get() == 'spot'
    assert tab.start_request()['point'] == [3.0, 2.0]
    tab.apply_tool('dock', (1.0, 1.0), (1.0, 2.0))
    assert tab.sent[-1]['cmd'] == 'set_dock' and abs(tab.sent[-1]['pose'][2] - 1.5708) < 1e-3
    tab.apply_tool('split', (2.0, 0.2), (2.0, 3.8))
    assert tab.sent[-1] == {'cmd': 'split_room', 'room': 1, 'line': [[2.0, 0.2], [2.0, 3.8]]}


def test_schedule_form_validation(tab):
    tab.sched_time_var.set('7:5')
    tab.sched_mode_var.set('all')
    entry = tab.schedule_entry_from_form()
    assert entry['time'] == '07:05' and entry['days'] == [0, 1, 2, 3, 4]
    tab.sched_time_var.set('25:00')
    with pytest.raises(ValueError):
        tab.schedule_entry_from_form()
    tab.sched_time_var.set('08:00')
    tab._save_schedule_entry(new=True)
    entries = tab.sent[-1]['entries']
    assert tab.sent[-1]['cmd'] == 'schedule' and len(entries) == 2 and entries[-1]['time'] == '08:00'


def test_status_drives_the_header_and_buttons(tab):
    status = {'state': 'cleaning', 'battery_pct': 64.0, 'battery_runtime_min': 96, 'area_m2': 12.34,
              'coverage_pct': 41.0, 'elapsed_s': 300, 'remaining_s': 420, 'room_name': 'Hall',
              'dustbin_pct': 8.0, 'map_ready': True, 'localised': True, 'task': 'lane', 'tasks_left': 30,
              'suction_label': 'Turbo', 'job': {'mode': 'rooms'}, 'pose': [6.0, 2.0, 0.0],
              'consumables': {'filter': {'left_pct': 90.0, 'used_h': 15.0, 'life_h': 150}}}
    tab._on_json('status', msg(status))
    assert tab.state_var.get().endswith('Cleaning')
    assert tab.metric_vars['area'].get() == '12.3 m²' and tab.metric_vars['room'].get() == 'Hall'
    assert tab.start_button.instate(['disabled']) and not tab.pause_button.instate(['disabled'])
    assert tab.part_widgets['filter'][1].get().startswith('90%')
    assert tab.canvas.find_withtag('robot')
    tab._on_json('status', msg(dict(status, state='error', error='Stuck - move the robot')))
    assert tab.message_var.get() == 'Stuck - move the robot'
    assert not tab.resume_button.instate(['disabled'])


def test_events_and_history_are_listed(tab):
    tab._on_event(msg({'text': 'Docked', 'level': 'info', 't': 0}))
    assert 'Docked' in tab.event_log.get('1.0', 'end')
    tab._on_json('history', msg([{'id': 'r1', 'started': 0, 'mode': 'rooms', 'room_names': ['Hall'],
                                  'result': 'completed', 'area_m2': 14.5, 'coverage_pct': 93.0,
                                  'duration_s': 900}]))
    row = tab.history_tree.item('r1', 'values')
    assert row[1] == 'Hall' and row[2] == 'completed' and row[3] == '14.5'
    assert tab.totals_var.get().startswith('1 runs')


def test_carpet_tool_mop_request_and_water_status(tab):
    tab.apply_tool('carpet', (5.0, 1.0), (6.0, 2.0))
    assert tab.sent[-1]['carpets'][0]['polygon'][2] == [6.0, 2.0]
    assert tab.canvas.find_withtag('overlay')
    status = {'state': 'docked', 'battery_pct': 100.0, 'map_ready': True, 'localised': True,
              'mop_module': False, 'water_pct': None}
    tab._on_json('status', msg(status))
    assert tab.clean_mode_combo.instate(['disabled']) and tab.water_var.get() == 'no mop module'
    tab._on_json('status', msg(dict(status, mop_module=True, water_pct=40.0)))
    assert not tab.clean_mode_combo.instate(['disabled']) and tab.water_var.get() == '40%'
    tab.clean_mode_var.set('Vacuum + mop')
    tab.water_flow_var.set('high')
    tab.mode_var.set('all')
    tab._start()
    assert tab.sent[-1]['clean_mode'] == 'vacuum_mop' and tab.sent[-1]['water_flow'] == 'high'
    tab._on_json('status', msg(dict(status, state='refilling', mop_module=True, water_pct=10.0)))
    assert tab.state_var.get().endswith('Refilling water tank') and tab.start_button.instate(['disabled'])
    tab.apply_tool('erase', (5.5, 1.5), (5.5, 1.5), moved=False)
    assert tab.sent[-1]['carpets'] == []
    tab.carpet_boost_var.set(False)
    tab._save_settings()
    assert tab.sent[-1]['values']['carpet_boost'] is False and tab.sent[-1]['values']['auto_refill'] is True


def test_recognised_objects_are_drawn_and_can_be_ignored(tab):
    status = {'state': 'cleaning', 'map_ready': True, 'localised': True, 'job': {'mode': 'all'},
              'objects': [{'id': 'o1', 'class': 'pet_waste', 'x': 5.0, 'y': 2.0, 'radius': 0.03, 'ignored': False},
                          {'id': 'o2', 'class': 'cable', 'x': 2.0, 'y': 1.0, 'radius': 0.3, 'ignored': True}]}
    tab._on_json('status', msg(status))
    before = len(tab.canvas.find_withtag('overlay'))
    tab._draw_overlays()
    assert len(tab.canvas.find_withtag('overlay')) >= before
    texts = [tab.canvas.itemcget(i, 'text') for i in tab.canvas.find_withtag('overlay')
             if tab.canvas.type(i) == 'text']
    assert '!' in texts and 'C' in texts
    tab.apply_tool('erase', (5.05, 2.0), (5.05, 2.0), moved=False)
    assert tab.sent[-1] == {'cmd': 'ignore_object', 'id': 'o1'}
    count = len(tab.sent)
    tab.apply_tool('erase', (2.0, 1.0), (2.0, 1.0), moved=False)      # already ignored: nothing to send
    assert len(tab.sent) == count
