"""Downloaded source alone must not grant a runnable robot or hide missing meshes."""
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('catalog_assets', Path(__file__).with_name('catalog_external_assets.py'))
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


@pytest.mark.parametrize('entry', ['../secret', '/etc/passwd', '-unsafe'])
def test_catalog_entry_cannot_escape_its_source(entry):
    with pytest.raises(ValueError):
        assets.relative_entry(entry)


def test_valid_xml_with_missing_geometry_is_not_dependency_complete(tmp_path):
    model = tmp_path/'robot.urdf'
    model.write_text('<robot name="fixture"><link name="base"><visual><geometry>'
                     '<mesh filename="missing.stl"/></geometry></visual></link></robot>')
    report = assets.inspect_urdf(model, tmp_path)
    assert report['unresolved_dependencies'] == ['missing.stl']
    assert not report['dependencies_complete']
    assert report['native_import']['status'] == 'not_tested'


def test_lfs_pointer_and_escaping_symlink_are_not_meshes(tmp_path):
    checkout = tmp_path/'source'
    checkout.mkdir()
    (checkout/'pointer.stl').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:abc\n')
    outside = tmp_path/'outside.stl'
    outside.write_text('outside')
    (checkout/'escape.stl').symlink_to(outside)
    model = checkout/'robot.urdf'
    model.write_text('<robot name="fixture"><link name="base"><visual><geometry>'
        '<mesh filename="pointer.stl"/></geometry></visual><collision><geometry>'
        '<mesh filename="escape.stl"/></geometry></collision></link></robot>')
    report = assets.inspect_urdf(model, checkout)
    assert report['lfs_pointers'] == ['pointer.stl']
    assert report['unresolved_dependencies'] == ['escape.stl']
    assert not report['dependencies_complete']


def test_complete_urdf_download_is_not_simulator_or_mission_support(tmp_path):
    model = tmp_path/'robot.urdf'
    model.write_text('<robot name="fixture"><link name="base"/></robot>')
    source = {'id': 'fixture', 'repository': 'unit-fixture', 'revision': 'a'*40,
              'license_note': 'unit fixture, not runtime evidence'}
    report = assets.inspect(source, 'robot.urdf', tmp_path, model)
    assert report['models'][0]['dependencies_complete']
    assert report['models'][0]['native_import']['status'] == 'not_tested'
    assert not report['runnable_profile'] and not report['mission_qualified']


def test_web_directory_snapshot_cannot_be_cloned_as_pinned_robot(tmp_path):
    source = {'id': 'directory', 'revision': 'web snapshot 2026-10-05',
              'repository': 'https://www.urdfhub.com/#robots'}
    with pytest.raises(ValueError, match='pinned'):
        assets.checkout(source, 'Panda', tmp_path)


def test_failed_source_refresh_preserves_previously_installed_robot(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace
    import yaml
    monkeypatch.syspath_prepend(str(Path(__file__).parent))
    import provision_extension_assets as provisioning
    store = tmp_path/'store'; installed = store/'installed'; installed.mkdir(parents=True)
    original = installed/'robot.urdf'; original.write_text('<robot name="retained"><link name="base"/></robot>')
    profile = dict(source_id='fixture', xacro=str(original), supported_modes=['display'])
    entity = dict(id='retained', tags=['extension','fixture'], capabilities=['display'])
    (installed/'robots.yaml').write_text(yaml.safe_dump({'robots': {'retained':profile}}))
    (installed/'registry_robots.yaml').write_text(yaml.safe_dump([entity]))
    failure = dict(source_id='fixture', entry='missing.urdf', status='needs integration repair',
                   reason='Missing source geometry')
    monkeypatch.setattr(provisioning, 'install_robots', lambda *args: ({}, [], [failure]))
    args = SimpleNamespace(source='fixture', kind='robots', no_download=True)
    assert provisioning.provision(args, store, installed, installed/'integration-report.json') == 0
    assert yaml.safe_load((installed/'robots.yaml').read_text())['robots']['retained'] == profile
    assert yaml.safe_load((installed/'registry_robots.yaml').read_text()) == [entity]
    assert original.read_text() == '<robot name="retained"><link name="base"/></robot>'
    assert json.loads((installed/'integration-report.json').read_text())['entries'] == [failure]
