"""Failed/partial imports cannot uninstall unrelated source-pinned assets."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
spec = importlib.util.spec_from_file_location('asset_provision', ROOT / 'scripts/provision_extension_assets.py')
provisioner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(provisioner)


@pytest.mark.parametrize('source', [None, 'vendor'])
def test_full_or_scoped_partial_import_preserves_profiles_entities_and_sibling_reports(tmp_path, monkeypatch, source):
    installed = tmp_path / 'installed'
    installed.mkdir()
    previous = {'retained': {'xacro': '/retained/model.urdf', 'drive': {'max_speed': .2}},
                'repaired': {'xacro': '/old/model.urdf'}}
    (installed / 'robots.yaml').write_text(yaml.safe_dump({'robots': previous}))
    (installed / 'registry_robots.yaml').write_text(yaml.safe_dump([
        {'id': name, 'source': name} for name in previous]))
    report = installed / 'integration-report.json'
    report.write_text(json.dumps({'entries': [
        {'source_id': 'vendor', 'entry': 'sibling', 'status': 'installed'},
        {'source_id': 'vendor', 'entry': 'broken', 'status': 'installed'}]}))
    monkeypatch.setattr(provisioner, 'install_robots', lambda *args: (
        {'repaired': {'xacro': '/new/model.urdf'}}, [{'id': 'repaired', 'source': 'new'}],
        [{'source_id': 'vendor', 'entry': 'broken', 'status': 'needs integration repair'}]))
    options = SimpleNamespace(kind='robots', no_download=True, source=source, entry=None)
    provisioner.provision(options, tmp_path, installed, report)
    actual = yaml.safe_load((installed / 'robots.yaml').read_text())['robots']
    assert actual['retained'] == previous['retained']
    assert actual['repaired']['xacro'] == '/new/model.urdf'
    entities = {item['id']: item for item in yaml.safe_load((installed / 'registry_robots.yaml').read_text())}
    assert entities['retained']['source'] == 'retained'
    assert entities['repaired']['source'] == 'new'
    rows = {item['entry']: item for item in json.loads(report.read_text())['entries']}
    assert rows['sibling']['status'] == 'installed'
    assert rows['broken']['status'] == 'needs integration repair'


def test_stale_source_certificate_keeps_model_and_requires_explicit_candidate_opt_in():
    from robot_lab_utils.asset_support import apply_support_or_candidate, experimental_asset_profile
    def apply_recorded_modes(*args):
        raise ValueError('Recorded runtime configuration differs: changed-controller.py')
    profile = dict(name='base', supported_simulators=['mujoco'], supported_modes=['display'],
                   drive_in_display=True, features=['velocity_base', 'lidar_2d'])
    support = dict(source_revision='pin', screens=[dict(backend='mujoco', mode='nav')])
    actual = apply_support_or_candidate(apply_recorded_modes, profile, 'pin', support, ROOT)
    assert actual['supported_modes'] == ['display']
    assert actual['supported_modes_by_simulator']['mujoco'] == ['display']
    assert 'runtime_screens' not in actual
    assert experimental_asset_profile(actual)['supported_modes'] == ['display']
    enabled = experimental_asset_profile(actual, True)
    assert enabled['supported_modes_by_simulator']['mujoco'] == ['display', 'nav']
    assert enabled['qualification_state'] == 'source_changed'
    assert 'runtime_screens' not in enabled
    assert profile['supported_modes'] == ['display']


def test_changed_robot_geometry_gets_no_experimental_modes():
    from robot_lab_utils.asset_support import apply_support_or_candidate
    def apply_recorded_modes(*args):
        raise ValueError('Recorded executed robot geometry/inertia differs: changed.urdf')
    result = apply_support_or_candidate(apply_recorded_modes,
        dict(supported_modes=['display'], drive_in_display=True), 'pin',
        dict(source_revision='pin', screens=[dict(backend='mujoco', mode='nav')]), ROOT)
    assert result['supported_modes'] == ['display']
    assert not result.get('experimental_modes_by_simulator')
