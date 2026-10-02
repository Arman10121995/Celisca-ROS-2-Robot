#!/usr/bin/env python3
"""Read-only demonstrations of why the old generated reports are not missions."""
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]

def module(relative, name):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec)
    sys.modules[name] = obj
    spec.loader.exec_module(obj)
    return obj

report = {'audit_date': '2026-10-02', 'baseline_revision': '2a0aae2',
          'probes': {}, 'qualification': 'Source/demo probes only; no simulator or flight launched.'}
with redirect_stdout(io.StringIO()):
    reset = module('src/robot_lab_maps/tools/r6_reset_validation.py', 'audit_reset')
    example = reset.create_reset_test_configs()[0]
    result = reset.ResetValidator()._validate_single_reset(example, 'audit_only')
    report['probes']['reset'] = {'status': result.status.value, 'findings': result.findings,
                                'actual_backend_integration': False}
    provenance = module('scripts/r9_1_provenance_framework.py', 'audit_provenance')
    classes = [v for v in vars(provenance).values() if isinstance(v, type) and hasattr(v, 'run_clean_host_test')]
    obj = classes[0]()
    report['probes']['clean_host'] = {'returned_success': obj.run_clean_host_test(),
                                    'results': [r.to_dict() for r in obj.clean_host_results],
                                    'logs_exist': [Path(r.log_file).exists() for r in obj.clean_host_results],
                                    'actual_build_or_mission': False}
    support = module('scripts/r9_3_support_matrix.py', 'audit_support')
    obj = support.SupportMatrixFramework()
    cell = support.SupportCell('bumperbot', 'aerial_course', 'isaac', 'coverage')
    obj._determine_support_level(cell)
    report['probes']['unqualified_isaac_cell'] = cell.to_dict()
paths = ['src/robot_lab_maps/tools/r6_reset_validation.py',
         'src/robot_lab_maps/tools/r6_geometry_map_alignment.py',
         'src/robot_lab_bringup/tools/r8_backend_qualification.py',
         'src/robot_lab_benchmark/tools/r8_resource_bounded_experiments.py',
         'scripts/r9_1_provenance_framework.py', 'scripts/r9_3_support_matrix.py',
         'docs/tutorials/r9_2_tutorials_framework.py']
report['source_sha256'] = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
print(json.dumps(report, indent=2))
