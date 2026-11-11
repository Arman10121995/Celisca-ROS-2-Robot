"""Optional torchvision contracts; no weights, downloads or training jobs required."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('torchvision')

from robot_lab_vacuum_cleaning.learned_detector import LearnedDetector, project_detections


def test_no_checkpoint_never_loads_network_or_downloads():
    with pytest.raises(FileNotFoundError):
        LearnedDetector('/does/not/exist.pt')


def test_training_has_disjoint_splits_and_same_input_range_as_inference():
    from PIL import Image
    path = Path(__file__).resolve().parents[3] / 'scripts/train_vacuum_detector.py'
    spec = importlib.util.spec_from_file_location('vacuum_training_contract', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    data = NS(ids=[1, 2, 3, 4], val_ids={2, 4})
    train, val = module.split_dataset(data)
    assert train.indices == [0, 2] and val.indices == [1, 3]
    pixel = module.DetData._default_transform(Image.new('RGB', (2, 2), (255, 128, 0)))
    assert pixel.shape == (3, 2, 2)
    assert pixel[:, 0, 0].tolist() == pytest.approx([1., 128 / 255, 0.])
    with pytest.raises(ValueError):
        module.split_dataset(NS(ids=[1], val_ids={1}))
    with pytest.raises(ValueError, match='same scene'):
        module.split_dataset(NS(ids=[1, 2], val_ids={2}, images=[
            {'id': 1, 'scene': 'same-house'}, {'id': 2, 'scene': 'same-house'}]))
    with pytest.raises(ValueError, match='lacks classes'):
        module.split_dataset(NS(ids=[1, 2], val_ids={2},
                                img2anns={1: [{'category_id': 1}], 2: [{'category_id': 1}]}))


def test_projection_retains_box_and_rejects_non_floor_and_invalid_depth():
    detection = {'class': 'cable', 'label': 20, 'bbox': [1, 1, 4, 4], 'score': .9}
    depth = np.ones((5, 5))
    k = [100., 0., 2., 0., 100., 2., 0., 0., 1.]
    # Optical z is forward in the floor plane; optical y points down.
    transform = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                          [0., -1., 0., 0.], [0., 0., 0., 1.]])
    obj, = project_detections([detection], depth, k, transform)
    assert obj['bbox'] == [1, 1, 4, 4] and obj['radius'] == .4
    assert obj['x'] == 1.
    assert project_detections([detection], depth * np.nan, k, transform) == []
    assert project_detections([detection], depth, k, np.eye(4)) == []
