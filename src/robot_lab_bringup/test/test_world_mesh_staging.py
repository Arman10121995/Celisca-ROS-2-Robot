"""A facet budget must not punch holes in a building's walls."""
from pathlib import Path
import sys
import pytest

trimesh = pytest.importorskip('trimesh')
pytest.importorskip('fast_simplification')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_utils'))
from robot_lab_utils import mesh_assets


def test_large_mesh_reduction_keeps_closed_surfaces(tmp_path, monkeypatch):
    walls = trimesh.util.concatenate([
        trimesh.creation.box([.1, 4, 1], transform=trimesh.transformations.translation_matrix([x, 0, .5]))
        for x in (-2, 2)])
    dense = walls.subdivide().subdivide()
    source = tmp_path / 'walls.stl'
    dense.export(source)
    monkeypatch.setattr(mesh_assets, '_WORLD_MAX_STL_FACES', 40)
    staged = mesh_assets.stage_world_mesh(str(source))
    reduced = trimesh.load(staged, process=True)
    assert len(reduced.faces) < len(dense.faces)
    assert reduced.is_watertight
    assert len(reduced.split()) == 2
    assert reduced.bounds == pytest.approx(walls.bounds, abs=1e-4)
    assert reduced.volume == pytest.approx(walls.volume, rel=.01)
    assert mesh_assets.stage_world_mesh(str(source)) == staged
