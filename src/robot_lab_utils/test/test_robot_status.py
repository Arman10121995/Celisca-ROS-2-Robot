"""Robot status marks (R14.5): derived from profiles and recorded evidence only."""
import json

import pytest

from robot_lab_utils import robot_status as rs


def profiles():
    return {
        'measured_bot': dict(supported_simulators=['gazebo', 'mujoco'], supported_modes=['display', 'loc', 'nav'],
                             features=['velocity_base'], drive_in_display=True),
        'opt_in_bot': dict(supported_simulators=['pybullet'], supported_modes=['display'], drive_in_display=True,
                           experimental_modes_by_simulator={'pybullet': ['display', 'nav']},
                           measured_screens=[dict(simulator='pybullet', mode='nav', evidence='docs/x.md')]),
        'arm_bot': dict(supported_simulators=['mujoco'], supported_modes=['display'], native_articulation=True),
        'statue': dict(supported_simulators=['mujoco'], supported_modes=['display']),
    }


@pytest.fixture
def evidence(tmp_path):
    folder = tmp_path / 'docs' / 'status' / 'evidence' / 'support-matrix-2026-01-01'
    folder.mkdir(parents=True)
    (folder / 'support-matrix-current.json').write_text(json.dumps(dict(support_cells=[
        dict(robot_id='measured_bot', simulator='gazebo', task_type='navigation_mission', test_result='passed_screen'),
        dict(robot_id='measured_bot', simulator='gazebo', task_type='localization', test_result='failed_screen'),
        dict(robot_id='measured_bot', simulator='mujoco', task_type='drive', test_result='passed_screen'),
    ])))
    return rs.load_measurements(profiles(), root=tmp_path)


def test_tiers_follow_evidence_and_capabilities(evidence):
    p = profiles()
    gazebo = rs.classify('measured_bot', p['measured_bot'], 'gazebo', evidence)
    assert gazebo['tier'] == rs.MEASURED and gazebo['symbol'] == '●'
    assert dict(gazebo['modes']) == {'display': rs.EXPERIMENTAL, 'loc': rs.EXPERIMENTAL, 'nav': rs.MEASURED}
    assert rs.mode_letters(gazebo) == 'd l N'                       # a failed screen is not "measured"
    assert 'measured on mujoco' in gazebo['summary']
    assert rs.classify('measured_bot', p['measured_bot'], 'isaac', evidence)['tier'] == rs.UNAVAILABLE
    opt_in = rs.classify('opt_in_bot', p['opt_in_bot'], 'pybullet', evidence)
    assert opt_in['tier'] == rs.MEASURED and rs.mode_letters(opt_in) == 'd N'
    assert rs.classify('arm_bot', p['arm_bot'], 'mujoco', evidence)['tier'] == rs.EXPERIMENTAL
    statue = rs.classify('statue', p['statue'], 'mujoco', evidence)
    assert statue['tier'] == rs.DISPLAY_ONLY and rs.mode_letters(statue) == 'd'


def test_opt_in_modes_are_bracketed_until_measured(evidence):
    profile = dict(profiles()['opt_in_bot'], measured_screens=[])
    status = rs.classify('opt_in_bot', profile, 'pybullet', rs.load_measurements({'opt_in_bot': profile}))
    assert rs.mode_letters(status) == 'd (n)' and status['tier'] == rs.EXPERIMENTAL
    assert 'experimental switch: Navigation' in status['summary']


def test_family_mark_is_its_best_member(evidence):
    p = profiles()
    members = [rs.classify(name, p[name], 'mujoco', evidence) for name in ('statue', 'arm_bot', 'measured_bot')]
    assert rs.best(members)['tier'] == rs.MEASURED
    assert rs.best([]) is None


def test_real_workspace_evidence_marks_known_robots():
    """The committed matrix and measured_screens produce marks for real profiles."""
    import yaml
    from robot_lab_utils.installed_assets import merge_installed_profiles
    root = rs.workspace_root()
    robots = merge_installed_profiles(yaml.safe_load((root / 'src/robot_lab_robots/config/robots.yaml').read_text()),
                                      'robots')['robots']
    measured = rs.load_measurements(robots, root=root)
    assert rs.classify('bumperbot', robots['bumperbot'], 'gazebo', measured)['tier'] == rs.MEASURED
    assert rs.classify('lab_usv', robots['lab_usv'], 'gazebo', measured)['tier'] == rs.MEASURED
