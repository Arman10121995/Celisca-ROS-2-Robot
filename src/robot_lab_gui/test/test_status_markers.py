"""Status marks in combobox lists: rows decorated, selection still the exact ID."""
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_gui.status_markers import decorate_combobox
from robot_lab_gui.workspace_ui import focused_widget

pytestmark = pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Actual Tk requires Xvfb or a display')


def test_rows_show_marks_and_choosing_one_selects_the_exact_id():
    root = tk.Tk()
    try:
        variable = tk.StringVar(value='robot_b')
        combo = ttk.Combobox(root, textvariable=variable, values=('robot_a', 'robot_b', 'none'), state='readonly')
        combo.pack()
        marks = {'robot_a': ('● robot_a   D N', '#4ade80'), 'robot_b': ('○ robot_b   d', '#9aa0a6')}
        decorate_combobox(combo, marks.get)
        root.update()
        combo.tk.call('ttk::combobox::Post', combo)
        for _ in range(20):
            root.update(); time.sleep(.02)
        listbox = str(combo.tk.call('ttk::combobox::PopdownWindow', combo)) + '.f.l'
        assert combo.tk.splitlist(combo.tk.call(listbox, 'get', 0, 'end')) == ('● robot_a   D N', '○ robot_b   d', 'none')
        assert str(combo.tk.call(listbox, 'itemcget', 0, '-foreground')) == '#4ade80'
        assert combo.tk.call(listbox, 'curselection') == (1,)        # the current value stays highlighted
        assert focused_widget(root) in (combo, None)               # popdown focus never raises
        combo.tk.call(listbox, 'selection', 'clear', 0, 'end')
        combo.tk.call(listbox, 'selection', 'set', 0)
        combo.tk.call('ttk::combobox::LBSelected', listbox)
        root.update()
        assert variable.get() == 'robot_a'                        # the value, never the decorated text
        assert combo.cget('values') == ('robot_a', 'robot_b', 'none')
    finally:
        root.destroy()
