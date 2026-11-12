"""Discover locally provisioned camera candidates without importing torch."""
import hashlib
import json
import os
from pathlib import Path

CLASSES = {'cable': 1, 'pet_waste': 2, 'shoe': 3, 'sock': 4}


def detector_root():
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'models/vacuum_camera'


def candidate_model_path(verify=True):
    root = detector_root().resolve()
    manifest = root / 'current.json'
    if not manifest.is_file():
        return None
    data = json.loads(manifest.read_text())
    if (data.get('schema_version') != 1 or data.get('cat_id') != CLASSES
            or data.get('status') not in ('experimental_candidate', 'qualified')):
        raise ValueError('Installed camera model manifest has incompatible classes/status')
    path = (root / data['path']).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError('Installed camera model is unavailable or escapes its asset directory')
    if verify:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        if digest.hexdigest() != data.get('sha256'):
            raise ValueError('Installed camera model checksum changed')
    return path


def resolve_model_path(value='auto'):
    if value and value != 'auto':
        path = Path(value).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError('Learned vacuum checkpoint does not exist: %s' % path)
        if path.is_relative_to(detector_root().resolve()):
            current = candidate_model_path(verify=False)
            if path == current:
                candidate_model_path(verify=True)
        return str(path)
    path = candidate_model_path()
    if path is None:
        raise FileNotFoundError('No local camera model is provisioned; select a trained checkpoint')
    return str(path)
