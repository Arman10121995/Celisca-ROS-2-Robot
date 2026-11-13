"""What a robot can do on a simulator, and what has been measured (ROADMAP R14.5).

The GUI marks every robot in its selectors from this module, so the marks are
derived from the same profiles and evidence the launch uses, never written by
hand:

* **measured** (●, green): at least one *passed* measured screen on the
  selected simulator;
* **experimental** (◐, amber): the robot runs and can be controlled there (a
  mode beyond Display, Drive in Display, or joint control) but nothing on that
  simulator has been measured;
* **display only** (○, grey): it only loads and shows itself there;
* **unavailable** (✕, red): not supported on the selected simulator.

Evidence, in order: the newest generated support matrix
(``docs/status/evidence/support-matrix-*/support-matrix-current.json``), the
named runtime screens of installed robots
(``docs/status/asset-runtime-support.yaml``), and ``measured_screens`` in a
profile (``[{simulator, mode, map, evidence}]``) for measurements made after
the last matrix. Each entry names its evidence document.
"""
import json
from pathlib import Path

import yaml

MEASURED, EXPERIMENTAL, DISPLAY_ONLY, UNAVAILABLE = 'measured', 'experimental', 'display_only', 'unavailable'
TIERS = (MEASURED, EXPERIMENTAL, DISPLAY_ONLY, UNAVAILABLE)
SYMBOLS = {MEASURED: '●', EXPERIMENTAL: '◐', DISPLAY_ONLY: '○', UNAVAILABLE: '✕'}
COLOURS = {MEASURED: '#4ade80', EXPERIMENTAL: '#fbbf24', DISPLAY_ONLY: '#9aa0a6', UNAVAILABLE: '#f87171'}
LABELS = {MEASURED: 'measured', EXPERIMENTAL: 'experimental', DISPLAY_ONLY: 'display only',
          UNAVAILABLE: 'not on this simulator'}
MODE_ORDER = ('display', 'loc', 'slam', '3d_slam', 'nav', 'flight', 'marine', 'gait')
MODE_LETTERS = {'display': 'D', 'loc': 'L', 'slam': 'S', '3d_slam': '3', 'nav': 'N', 'flight': 'F',
                'marine': 'M', 'gait': 'G'}
MODE_NAMES = {'display': 'Display', 'loc': 'Localization', 'slam': 'SLAM', '3d_slam': '3D SLAM',
              'nav': 'Navigation', 'flight': 'Flight', 'marine': 'Marine', 'gait': 'Gait'}
# Support-matrix task types -> the mode they exercise (drive and arm screens run in Display).
TASK_MODES = {
    'navigation': 'nav', 'navigation_mission': 'nav', 'trained_policy_navigation_go2_walk_r1': 'nav',
    'localization': 'loc', 'slam': 'slam', 'slam_3d': '3d_slam', '3d_slam': '3d_slam', 'flight': 'flight',
    'teleoperation': 'display', 'drive': 'display', 'command_loss': 'display', 'car_course': 'display',
    'legged_policy_drive': 'display', 'trained_policy_drive': 'display',
    'trained_policy_drive_go2_walk_r2': 'display', 'camera_calibration': 'display',
    'gui_articulation': 'display', 'arm_cartesian': 'display', 'bounded_live_smoke': 'display',
}
CONTROL_KEYS = ('drive_in_display', 'articulation', 'arm_control', 'native_articulation', 'mobile_control_config',
                'native_hand', 'marine_model', 'gait_model')
ALL_SIMULATORS = ('gazebo', 'mujoco', 'pybullet', 'isaac')


def workspace_root():
    """The workspace holding docs/status (the installed copy lives under it too)."""
    for parent in Path(__file__).resolve().parents:
        if (parent / 'docs' / 'status' / 'platform-status.yaml').is_file():
            return parent
    return Path(__file__).resolve().parents[3]


def _matrix_path(root):
    candidates = sorted((root / 'docs' / 'status' / 'evidence').glob('support-matrix-*/support-matrix-current.json'))
    return candidates[-1] if candidates else None


def load_measurements(profiles, root=None):
    """{robot: {(simulator, mode): {'passed': bool, 'evidence': str}}} from every evidence source."""
    root = Path(root) if root else workspace_root()
    found = {}

    def add(robot, simulator, mode, passed, evidence):
        cell = found.setdefault(robot, {}).setdefault((simulator, mode), dict(passed=False, evidence=evidence))
        if passed and not cell['passed']:
            cell.update(passed=True, evidence=evidence)

    matrix = _matrix_path(root)
    if matrix:
        for cell in json.loads(matrix.read_text()).get('support_cells', []):
            mode = TASK_MODES.get(cell.get('task_type'))
            if mode and cell.get('simulator'):
                add(cell['robot_id'], cell['simulator'], mode, cell.get('test_result') == 'passed_screen',
                    str(matrix.relative_to(root)))
    runtime = root / 'docs' / 'status' / 'asset-runtime-support.yaml'
    if runtime.is_file():
        for robot, entry in ((yaml.safe_load(runtime.read_text()) or {}).get('robots') or {}).items():
            for screen in entry.get('screens', []):
                add(robot, screen.get('backend'), screen.get('mode', 'display'), True, str(runtime.relative_to(root)))
    for robot, profile in (profiles or {}).items():
        for screen in profile.get('measured_screens') or []:
            add(robot, screen['simulator'], screen.get('mode', 'display'), screen.get('passed', True),
                screen.get('evidence', ''))
    return found


def simulators_of(profile):
    return list(profile.get('supported_simulators') or ALL_SIMULATORS)


def modes_on(profile, simulator, experimental=False):
    """(supported modes, experimental-only modes) on ``simulator``."""
    by_simulator = profile.get('supported_modes_by_simulator') or {}
    supported = list(by_simulator.get(simulator, profile.get('supported_modes') or ['display']))
    extra = [mode for mode in (profile.get('experimental_modes_by_simulator') or {}).get(simulator, [])
             if mode not in supported]
    return supported, extra


def classify(robot, profile, simulator, measurements):
    """Status of one robot on one simulator: tier, symbol, colour, per-mode states and a summary line."""
    if simulator not in simulators_of(profile):
        elsewhere = [sim for sim in simulators_of(profile)]
        return dict(tier=UNAVAILABLE, symbol=SYMBOLS[UNAVAILABLE], colour=COLOURS[UNAVAILABLE], modes=[],
                    summary='Not available on %s; runs on %s.' % (simulator, ', '.join(elsewhere) or 'no simulator'))
    supported, extra = modes_on(profile, simulator)
    cells = measurements.get(robot, {})
    measured = {mode for (sim, mode), cell in cells.items() if sim == simulator and cell['passed']}
    modes = []
    for mode in MODE_ORDER:
        if mode in measured:
            modes.append((mode, MEASURED))
        elif mode in supported:
            modes.append((mode, EXPERIMENTAL if mode != 'display' or _controllable(profile) else DISPLAY_ONLY))
        elif mode in extra:
            modes.append((mode, 'opt_in'))
    if measured:
        tier = MEASURED
    elif len(supported) > 1 or extra or _controllable(profile):
        tier = EXPERIMENTAL
    else:
        tier = DISPLAY_ONLY
    measured_elsewhere = sorted({sim for (sim, _mode), cell in cells.items() if cell['passed'] and sim != simulator})
    parts = []
    names = lambda state: ', '.join(MODE_NAMES[m] for m, s in modes if s == state)
    if names(MEASURED):
        parts.append('measured: ' + names(MEASURED))
    if names(EXPERIMENTAL):
        parts.append('runs, not measured here: ' + names(EXPERIMENTAL))
    if names('opt_in'):
        parts.append('experimental switch: ' + names('opt_in'))
    if tier == DISPLAY_ONLY:
        parts.append('display only (no controller)')
    if measured_elsewhere:
        parts.append('measured on ' + ', '.join(measured_elsewhere))
    return dict(tier=tier, symbol=SYMBOLS[tier], colour=COLOURS[tier], modes=modes,
                summary='%s %s on %s — %s.' % (SYMBOLS[tier], LABELS[tier].capitalize(), simulator, '; '.join(parts)))


def _controllable(profile):
    return any(profile.get(key) for key in CONTROL_KEYS)


def mode_letters(status):
    """Compact mode string for a list row, e.g. ``D L S N`` (measured modes upper case,
    unmeasured lower case, opt-in modes in brackets)."""
    letters = []
    for mode, state in status['modes']:
        letter = MODE_LETTERS[mode]
        letters.append(letter if state == MEASURED else '(%s)' % letter.lower() if state == 'opt_in' else letter.lower())
    return ' '.join(letters)


def best(statuses):
    """The family's mark: its best member."""
    return min(statuses, key=lambda status: TIERS.index(status['tier'])) if statuses else None


LEGEND = ('● measured  ◐ experimental  ○ display only  ✕ not on this simulator\n'
          'Modes on the chosen simulator: D display · L localization · S SLAM · 3 3D SLAM · N navigation · '
          'F flight · M marine · G gait. CAPITAL = measured, small = runs but not measured here, '
          '(x) = needs the experimental switch.')
