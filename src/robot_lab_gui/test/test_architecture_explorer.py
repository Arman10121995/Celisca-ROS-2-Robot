"""Real Tk interaction; no simulator or robot command is launched."""
import os
import time
from pathlib import Path

import pytest
from test_command_autofill import app  # noqa: F401

pytestmark=[pytest.mark.integration,pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Requires isolated Xvfb/display')]


@pytest.fixture
def learning_app(app):
    # The shared fixture suppresses auxiliary tabs; add this real tab explicitly.
    from robot_lab_gui.lab_tabs import HealthTab
    HealthTab(app.notebook, app)
    app.update()
    return app


def test_native_explorer_tour_source_links_and_networking_remain_neutral(learning_app):
    app=learning_app
    from robot_lab_gui.architecture_content import COMPONENTS,JOURNEYS
    health=app.health_tab;explorer=health.explorer
    app.show_tab('Health');app.update()
    assert health.pages.select()==str(explorer)
    assert explorer.worker is None and explorer.snapshot is None
    assert app.ros_node is None
    explorer.start_journey()
    for view,key in JOURNEYS[explorer.journey.get()]:
        assert explorer.current['id']==view and explorer.selected==key
        assert COMPONENTS[key]['title'] in explorer.inspector.get('1.0','end')
        explorer.advance(1)
    explorer.search.set('MAVLink');explorer.search_components()
    assert explorer.selected and 'MAVLink' in explorer.inspector.get('1.0','end')
    explorer.network_vars['publisher_reliability'].set('best_effort');explorer.update_network()
    assert '✗ Reliability' in explorer.network_result.get()
    explorer.related_guide();app.update()
    assert health.pages.select()==str(health.guide_library)
    assert health.guide_library.current.is_file()
    health._show_document(health.guide_library.root/'docs/architecture/interactive-guide.md')
    assert health.guide_library.heading_lines
    assert 'Open interactive flowchart' in health.guide_library.reader.get('1.0','end')
    health._show_architecture('network');app.update()
    assert explorer.current['id']=='network' and health.pages.select()==str(explorer)
    health._show_document(health.guide_library.root/'docs/architecture/glossary.md')
    glossary=health.guide_library.reader.get('1.0','end')
    assert 'Plain meaning:' in glossary and '|---|' not in glossary
    health._show_document(health.guide_library.root/'docs/architecture/interactive-guide.md')
    health.guide_library.find.set('Nav2');health.guide_library.find_text()
    assert health.guide_library.reader.tag_ranges('match')
    health.guide_library.open(health.guide_library.root/'src/robot_lab_gui')
    assert 'package.xml' in health.guide_library.reader.get('1.0','end')
    health.guide_library.back()
    assert health.guide_library.current.name=='interactive-guide.md'
    health.guide_library.open(Path('/etc/passwd'))
    assert 'outside this workspace' in health.guide_library.location.get()
    assert explorer.worker is None and app.ros_node is None


def test_owned_tour_timer_is_cancelled_on_widget_destroy(learning_app):
    app=learning_app
    explorer=app.health_tab.explorer
    explorer.toggle_play();assert explorer.play_job
    job=explorer.play_job;explorer.destroy();app.update()
    assert explorer.closed and explorer.cancel.is_set() and explorer.play_job is None
    assert job not in app.tk.splitlist(app.tk.call('after','info'))


def test_networking_lesson_fits_narrow_viewport_and_scrolls_vertically(learning_app):
    app=learning_app;app.geometry('1024x768');app.show_tab('Health')
    explorer=app.health_tab.explorer
    explorer.tabs.select(explorer.network_panel)
    deadline=time.monotonic()+.6
    while time.monotonic()<deadline:app.update();time.sleep(.01)
    assert explorer.network_panel.canvas.xview()[1]==1.
    assert explorer.network_panel.canvas.bbox('all')[3]>explorer.network_panel.canvas.winfo_height()
    assert explorer.worker is None and app.ros_node is None
