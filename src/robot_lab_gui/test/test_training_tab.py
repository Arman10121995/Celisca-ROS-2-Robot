"""Training tab: run discovery, progress display and handing a policy to Launch."""
import json
import os

import pytest

from test_command_autofill import app, displayed_command, select  # noqa: F401

pytestmark = [pytest.mark.integration,
              pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Requires Xvfb or a display')]


def _fake_run(root, name='unitree_go2_walk_unit', robot='unitree_go2', with_policy=True):
    run = root / 'training' / name
    (run / 'checkpoints').mkdir(parents=True)
    (run / 'run.json').write_text(json.dumps(dict(robot=robot, task='walk', iterations=10, pid=0)))
    lines = [dict(iteration=i, environment_steps=i * 1000, steps_per_s=900, wall_s=i * 1.5,
                  mean_return=-5.0 + i, mean_episode_s=2.0 + i, terms={'tracking_lin_vel': 0.01 * i},
                  kl=0.01, learning_rate=1e-3, action_std=0.9) for i in range(1, 4)]
    (run / 'progress.jsonl').write_text(''.join(json.dumps(line) + '\n' for line in lines))
    (run / 'checkpoints' / 'latest.pt').write_bytes(b'x')
    if with_policy:
        (run / 'policy').mkdir()
        (run / 'policy' / 'policy.onnx').write_bytes(b'x')
        (run / 'policy' / 'source.json').write_text(json.dumps(dict(profile=robot)))
    return run


def test_helpers_build_the_trainer_command_and_summaries(tmp_path):
    from robot_lab_gui.training_tab import evaluation_summary, run_summary, trainer_command, trainer_pid
    run = _fake_run(tmp_path)
    (run / 'evaluation.json').write_text(json.dumps(dict(phases=[
        dict(command=[0.5, 0, 0], measured=[0.431, 0.012, -0.02], fallen=False),
        dict(command=[0, 0, 0.8], measured=[0.0, 0.0, 0.2], fallen=True)])))
    line = evaluation_summary(run / 'evaluation.json')
    assert '(0.5, 0, 0) → (0.43, 0.01, -0.02)' in line and '(0, 0, 0.8) → (0.00, 0.00, 0.20) FELL' in line
    summary = run_summary(run)
    assert summary['iteration'] == 3 and summary['planned'] == 10 and summary['policy']
    assert summary['mean_return'] == -2.0 and not summary['finished']
    assert trainer_pid(run) == 0
    command = trainer_command('unitree_go2', 'stand', 64, 20, 24, 1e-3, 4, run, resume='')
    assert command[1:4] == ['-m', 'robot_lab_adapter.legged_train', 'train']
    assert command[command.index('--envs') + 1] == '64' and command[command.index('--task') + 1] == 'stand'
    assert '--resume' not in command
    # A live trainer started elsewhere (ros2 run ... legged-train) is found by its run.json PID.
    import subprocess
    import sys
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', 'legged-train', 'train',
                              '--out', str(run)])
    try:
        info = json.loads((run / 'run.json').read_text())
        (run / 'run.json').write_text(json.dumps(dict(info, pid=child.pid)))
        assert trainer_pid(run) == child.pid
        assert trainer_pid(run.parent / 'other') == 0
    finally:
        child.kill()
        child.wait()
    assert trainer_pid(run) == 0


def test_tab_lists_runs_plots_progress_and_hands_the_policy_to_launch(app, tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    run = _fake_run(tmp_path)
    _fake_run(tmp_path, name='unitree_a1_walk_other', robot='unitree_a1', with_policy=False)
    from robot_lab_gui.training_tab import TrainingTab
    app.training_tab = tab = TrainingTab(app.notebook, app)
    try:
        app.update()
        assert set(tab.tree.get_children()) == {str(run), str(tmp_path / 'training' / 'unitree_a1_walk_other')}
        assert tab.robot_var.get() in {'unitree_a1', 'unitree_b2', 'unitree_go2'}
        tab.tree.selection_set(str(run))
        tab._select_run()
        app.update()
        assert 'iteration 3 of 10' in tab.status.get()
        assert tab.plot.find_all(), 'the return and episode curves are drawn'
        assert tab.export_button.instate(['!disabled']) and tab.use_button.instate(['!disabled'])
        assert tab.stop_button.instate(['disabled'])
        select(app, app.robot_combo, 'unitree_go2')
        select(app, app.simulator_combo, 'mujoco')
        tab.use_in_launch()
        app.update()
        assert app.trained_policy_var.get() == str(run / 'policy')
        assert 'legged_policy:=' + str(run / 'policy') in displayed_command(app)
        assert 'Trained policy:' in app.trained_policy_label.cget('text')
        # Another robot never receives the Go2 policy: it keeps its own.
        select(app, app.robot_combo, 'unitree_a1')
        app.mode_buttons['loc'].invoke()
        command = displayed_command(app)
        assert not any(part == 'legged_policy:=' + str(run / 'policy') for part in command)
        assert 'not for this robot' in app.trained_policy_label.cget('text')
        app.trained_policy_clear.invoke()
        app.update()
        select(app, app.robot_combo, 'unitree_go2')
        assert 'legged_policy:=go2_himloco' in displayed_command(app)
        # Results of export/evaluate threads reach the page on the Tk thread.
        tab.tool_results.append((0, lambda: 'Exported somewhere'))
        tab.tool_results.append((2, lambda: 'never shown'))
        tab._poll()
        assert tab.note.get() == 'Failed (exit 2): see the console.' and not tab.tool_results
        # A selected run without a policy cannot be evaluated or used.
        tab.tree.selection_set(str(tmp_path / 'training' / 'unitree_a1_walk_other'))
        tab._select_run()
        assert tab.use_button.instate(['disabled']) and tab.evaluate_button.instate(['disabled'])
    finally:
        tab.close()


def test_start_runs_the_trainer_in_its_own_session_and_stop_requests_an_export(app, tmp_path, monkeypatch):
    from unittest.mock import Mock, patch
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    from robot_lab_gui import training_tab
    app.training_tab = tab = training_tab.TrainingTab(app.notebook, app)
    process = Mock(pid=4321)
    process.poll.return_value = None
    try:
        tab.robot_var.set('unitree_b2')
        tab.task_var.set('push_recovery')
        tab.envs_var.set(32)
        tab.iterations_var.set(5)
        tab.out_var.set(str(tmp_path / 'training' / 'b2_push'))
        with patch.object(training_tab.importlib.util, 'find_spec', return_value=object()), \
                patch.object(training_tab.subprocess, 'Popen', return_value=process) as popen, \
                patch.object(training_tab.os, 'killpg') as killpg:
            tab.start()
            command = popen.call_args.args[0]
            assert command[command.index('--robot') + 1] == 'unitree_b2'
            assert command[command.index('--task') + 1] == 'push_recovery'
            assert command[command.index('--out') + 1] == str(tmp_path / 'training' / 'b2_push')
            assert popen.call_args.kwargs['start_new_session'] is True
            assert tab.running() and tab.start_button.instate(['disabled'])
            tab.stop()
            assert (tmp_path / 'training' / 'b2_push' / 'STOP').exists()
            killpg.assert_called_once()
            assert killpg.call_args.args[0] == 4321
    finally:
        tab.process = None
        tab.close()
