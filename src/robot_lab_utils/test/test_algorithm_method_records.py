"""Per-method records from the catalog, dispatch and code (R7)."""
import importlib.util
from pathlib import Path

import yaml

spec = importlib.util.spec_from_file_location('algorithm_method_records',
    Path(__file__).resolve().parents[3] / 'scripts' / 'algorithm_method_records.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def rows(prefixes=()):
    return module.records(*module.load(), {}, list(prefixes))


def test_one_record_per_catalog_or_dispatch_id():
    entries, routes, _ = module.load()
    ids = [r['id'] for r in rows()]
    assert sorted(ids) == sorted(set(entries) | set(routes))


def test_curated_notes_name_existing_methods():
    entries, routes, _ = module.load()
    for method, (state, note) in module.NAME_MATH.items():
        assert method in entries or method in routes, method
        assert state in ('mismatch', 'corrected', 'not_a_method') and note


def test_every_method_states_its_mathematics():
    missing = [r['id'] for r in rows() if not r['mathematics']]
    assert not missing, missing
    entries, routes, maths = module.load()
    assert not set(module.MATHEMATICS) & set(maths), 'curated text would hide a subrecord'


def test_identical_dispatch_entries_are_one_method():
    by_id = {r['id']: r for r in rows()}
    assert by_id['dijkstra_planner']['name_math'] == 'mismatch'
    assert any('navfn_planner' in n for n in by_id['dijkstra_planner']['notes'])
    # The same stack for different drive types is not a duplicate.
    assert by_id['native_holonomic_controller']['name_math'] == 'consistent'


def test_corrected_dispatch_entries():
    _, routes, _ = module.load()
    assert routes['laser_scan_to_pointcloud'][1]['node']['executable'] == 'laserscan_to_pointcloud_node'
    assert 'unavailable' in routes['mavros_offboard_controller'][1]


def test_installed_checks_the_executable(tmp_path):
    prefix = tmp_path / 'prefix'
    (prefix / 'share/ament_index/resource_index/packages').mkdir(parents=True)
    (prefix / 'share/ament_index/resource_index/packages/demo_pkg').write_text('')
    entry = dict(node=dict(package='demo_pkg', executable='demo_node'))
    assert module.installed(entry, None, [prefix])[0] is False
    (prefix / 'lib/demo_pkg').mkdir(parents=True)
    (prefix / 'lib/demo_pkg/demo_node').write_text('')
    assert module.installed(entry, None, [prefix]) == (True, 'demo_pkg/demo_node')
    assert module.installed(dict(plugin='demo_pkg::Thing'), None, [prefix])[0] is True
    assert module.installed(dict(plugin='other_pkg/Thing'), None, [prefix])[0] is False
    assert module.installed(dict(unavailable='retired'), None, [prefix])[0] is False


def test_ledger_block_is_replaced_not_duplicated(tmp_path):
    ledger = tmp_path / 'ledger.yaml'
    ledger.write_text('groups:\n  R6:\n    state: partial\n  R7:\n    owner: x\n    state: partial\n    tasks:\n'
                      '      R7.1:\n        acceptance_ref: ROADMAP.md\n        state: partial\n  R8:\n    state: open\n')
    records = rows()[:3]
    module.write_ledger(ledger, records, '2026-11-14')
    module.write_ledger(ledger, records, '2026-11-15')
    data = yaml.safe_load(ledger.read_text())['groups']
    assert ledger.read_text().count('method_records:') == 1
    assert data['R7']['method_records']['generated'] == '2026-11-15'
    assert set(data['R7']['method_records']['records']) == {r['id'] for r in records}
    assert data['R7']['tasks']['R7.1']['state'] == 'partial' and data['R8']['state'] == 'open'
