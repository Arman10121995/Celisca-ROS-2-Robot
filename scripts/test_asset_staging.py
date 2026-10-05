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
