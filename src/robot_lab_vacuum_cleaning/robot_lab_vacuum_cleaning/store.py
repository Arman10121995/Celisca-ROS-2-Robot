"""On-disk state of the cleaning system: per-map layouts (rooms, keep-out
areas, zones, dock), settings, schedules, consumables and run history."""

import json
import os
import re
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

DEFAULT_SETTINGS = {
    'suction': 'standard',
    'passes': 1,
    'edge': True,
    'speed_scale': 1.0,
    'battery_time_scale': 1.0,
    'auto_empty': True,
    'charge_resume': True,
    'low_battery_pct': 20.0,
    'resume_battery_pct': 80.0,
    'dnd': {'enabled': False, 'start': '22:00', 'end': '08:00'},
}

CONSUMABLES = {
    # Replacement intervals in cleaning hours (typical commercial values).
    'main_brush': {'label': 'Main brush', 'life_h': 300.0},
    'side_brush': {'label': 'Side brush', 'life_h': 200.0},
    'filter': {'label': 'Filter', 'life_h': 150.0},
    'sensors': {'label': 'Sensors (clean)', 'life_h': 30.0},
}


def default_root() -> Path:
    runtime = os.environ.get('ROBOT_LAB_RUNTIME_ROOT')
    if runtime:
        return Path(runtime) / 'vacuum'
    return Path.home() / '.ros' / 'robot_lab_vacuum'


def safe_key(name: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', name or 'default')[:80] or 'default'


def _read_json(path: Path, default):
    try:
        with open(path, encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return default


def _write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    with open(tmp, 'w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=1, sort_keys=True)
    os.replace(tmp, path)


class Store:
    def __init__(self, root: Optional[Path] = None):
        self.root = Path(root) if root else default_root()

    # -- layouts --------------------------------------------------------
    def layout_dir(self, map_key: str) -> Path:
        return self.root / 'maps' / safe_key(map_key)

    def load_layout(self, map_key: str):
        d = self.layout_dir(map_key)
        layout = _read_json(d / 'layout.json', None)
        labels = None
        if layout is not None and (d / 'rooms.npy').exists():
            try:
                labels = np.load(d / 'rooms.npy')
            except (OSError, ValueError):
                labels = None
        return layout, labels

    def save_layout(self, map_key: str, layout: Dict, labels: Optional[np.ndarray]) -> None:
        d = self.layout_dir(map_key)
        _write_json(d / 'layout.json', layout)
        if labels is not None:
            d.mkdir(parents=True, exist_ok=True)
            np.save(d / 'rooms.npy', labels.astype(np.int16))
        elif (d / 'rooms.npy').exists():
            (d / 'rooms.npy').unlink()

    def list_maps(self) -> List[str]:
        d = self.root / 'maps'
        return sorted(p.name for p in d.iterdir() if p.is_dir()) if d.exists() else []

    # -- settings, schedule, consumables --------------------------------
    def load_settings(self) -> Dict:
        data = dict(DEFAULT_SETTINGS)
        data.update(_read_json(self.root / 'settings.json', {}))
        return data

    def save_settings(self, settings: Dict) -> None:
        _write_json(self.root / 'settings.json', settings)

    def load_schedule(self) -> List[Dict]:
        return _read_json(self.root / 'schedule.json', [])

    def save_schedule(self, schedule: List[Dict]) -> None:
        _write_json(self.root / 'schedule.json', schedule)

    def load_consumables(self) -> Dict[str, float]:
        used = _read_json(self.root / 'consumables.json', {})
        return {key: float(used.get(key, 0.0)) for key in CONSUMABLES}

    def save_consumables(self, used: Dict[str, float]) -> None:
        _write_json(self.root / 'consumables.json', used)

    # -- history ---------------------------------------------------------
    def append_history(self, record: Dict, coverage: Optional[np.ndarray] = None) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        with open(self.root / 'history.jsonl', 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, sort_keys=True) + '\n')
        if coverage is not None:
            d = self.root / 'history'
            d.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(d / ('%s.npz' % safe_key(record['id'])),
                                coverage=np.packbits(coverage.astype(bool)),
                                shape=np.array(coverage.shape))

    def load_history(self, limit: int = 50) -> List[Dict]:
        path = self.root / 'history.jsonl'
        if not path.exists():
            return []
        out = []
        with open(path, encoding='utf-8') as handle:
            for line in handle:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    continue
        return out[-limit:]

    def load_history_coverage(self, record_id: str) -> Optional[np.ndarray]:
        path = self.root / 'history' / ('%s.npz' % safe_key(record_id))
        try:
            data = np.load(path)
        except (OSError, ValueError):
            return None
        shape = tuple(int(v) for v in data['shape'])
        bits = np.unpackbits(data['coverage'])[:shape[0] * shape[1]]
        return bits.reshape(shape).astype(bool)


def schedule_due(entry: Dict, now_struct: time.struct_time) -> bool:
    """True when ``entry`` (days 0=Mon..6, 'HH:MM') matches this minute."""
    if not entry.get('enabled', True):
        return False
    days = entry.get('days', list(range(7)))
    if now_struct.tm_wday not in days:
        return False
    try:
        hh, mm = (int(v) for v in str(entry.get('time', '')).split(':'))
    except ValueError:
        return False
    return now_struct.tm_hour == hh and now_struct.tm_min == mm


def in_window(start: str, end: str, now_struct: time.struct_time) -> bool:
    """True when the local time lies in [start, end) (may wrap midnight)."""
    try:
        sh, sm = (int(v) for v in start.split(':'))
        eh, em = (int(v) for v in end.split(':'))
    except ValueError:
        return False
    now = now_struct.tm_hour * 60 + now_struct.tm_min
    a, b = sh * 60 + sm, eh * 60 + em
    if a == b:
        return False
    return a <= now < b if a < b else (now >= a or now < b)
