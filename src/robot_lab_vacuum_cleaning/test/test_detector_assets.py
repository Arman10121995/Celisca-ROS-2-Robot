"""Keep deployed experimental camera weights source-bound and locally resolved."""
import hashlib
import json

import pytest

from robot_lab_vacuum_cleaning.detector_assets import CLASSES, candidate_model_path, resolve_model_path


def test_candidate_resolves_only_matching_local_content(monkeypatch, tmp_path):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    assert candidate_model_path() is None
    with pytest.raises(FileNotFoundError):
        resolve_model_path('auto')
    root = tmp_path / 'models/vacuum_camera'
    root.mkdir(parents=True)
    (root / 'weights.pt').write_bytes(b'checkpoint fixture, no model inference')
    manifest = {'schema_version': 1, 'status': 'experimental_candidate', 'cat_id': CLASSES,
                'path': 'weights.pt', 'sha256': hashlib.sha256((root / 'weights.pt').read_bytes()).hexdigest()}
    (root / 'current.json').write_text(json.dumps(manifest))
    assert resolve_model_path() == str(root / 'weights.pt')
    (root / 'weights.pt').write_bytes(b'changed')
    with pytest.raises(ValueError, match='checksum'):
        resolve_model_path()
    manifest['path'] = '../../outside.pt'
    (root / 'current.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='escapes'):
        resolve_model_path()
