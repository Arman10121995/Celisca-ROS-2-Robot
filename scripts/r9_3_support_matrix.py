#!/usr/bin/env python3
"""Build scoped support rows from hashed measurements and gates from the ledger.

Historical October 1 reports are preserved. A short successful screen is
partial support, not full mission qualification. Unindexed cells are untested.
"""
import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum
import hashlib
import json
import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class SupportLevel(str, Enum):
    FULLY_SUPPORTED = 'fully_supported'
    PARTIALLY_SUPPORTED = 'partially_supported'
    EXPERIMENTAL = 'experimental'
    UNSUPPORTED = 'unsupported'
    NOT_TESTED = 'not_tested'


class EvidenceLevel(str, Enum):
    RUNTIME_EVIDENCE = 'runtime_evidence'
    STATIC_CHECK = 'static_check'
    CATALOG_ONLY = 'catalog_only'
    NONE = 'none'


class ReleaseStatus(str, Enum):
    READY = 'ready'
    CONDITIONAL = 'conditional'
    BLOCKED = 'blocked'
    DEFERRED = 'deferred'


@dataclass
class SupportCell:
    robot_id: str
    environment_id: str
    simulator: str
    task_type: str
    support_level: SupportLevel = SupportLevel.NOT_TESTED
    evidence_level: EvidenceLevel = EvidenceLevel.NONE
    last_tested: str = ''
    revision: str = ''
    test_result: str = 'not_tested'
    artifacts: list = field(default_factory=list)
    limitations: list = field(default_factory=list)
    notes: str = ''
    steering_mode: str = ''

    def to_dict(self):
        return asdict(self)


@dataclass
class ReleaseGate:
    condition: str
    status: ReleaseStatus = ReleaseStatus.BLOCKED
    required_for: list = field(default_factory=list)
    blocking_issues: list = field(default_factory=list)
    evidence_required: list = field(default_factory=list)
    decision: str = ''

    def to_dict(self):
        return asdict(self)


def flatten_tasks(tasks):
    result = {}
    for key,task in tasks.items():
        if isinstance(task,dict):
            result[key] = task
            result.update(flatten_tasks(task.get('tasks',{})))
    return result


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def valid_position(value):
    return isinstance(value,list) and len(value)==3 and all(finite(v) for v in value)


def measured_result(report,kind):
    """Recompute declared screen checks; never accept a bare PASS marker."""
    if kind == 'navigation_screen':
        limits = report.get('acceptance',{}).get('limits',{})
        position,heading,age = (report.get(key) for key in (
            'final_error_truth_m','final_yaw_error_truth_deg','truth_age_wall_s'))
        passed = (report.get('outcome')=='succeeded' and report.get('settled_sim_second') is True
                  and finite(position) and finite(heading) and finite(age) and 0<=age<1
                  and finite(limits.get('position_m')) and 0<=position<=limits['position_m']
                  and finite(limits.get('yaw_deg')) and abs(heading)<=limits['yaw_deg'])
        route = report.get('route_acceptance')
        if route is not None:
            passed = passed and finite(route.get('min_swept_clearance_m')) and (
                route['min_swept_clearance_m']>=route['min_clearance_limit_m'])
            if 'straight_route_blocked' in route.get('checks',{}):
                passed = passed and finite(route.get('direct_route_clearance_m')) and route['direct_route_clearance_m']<0
        return bool(passed)
    if kind == 'workflow_screen':
        checks,phases = report.get('checks',{}),report.get('phases',{})
        return (bool(checks) and all(value is True for value in checks.values())
                and report.get('truth_samples',0)>1 and report.get('clock_backward_jumps')==0
                and finite(phases.get('forward',{}).get('distance_m'))
                and phases['forward']['distance_m']>.25
                and finite(phases.get('reverse',{}).get('distance_m'))
                and phases['reverse']['distance_m']>.25
                and finite(report.get('localization_error_m')) and report['localization_error_m']<.25)
    if kind == 'flight_screen':
        phases = report.get('phases',{})
        idle,goal,drive = (phases.get(k,{}) for k in ('idle','goal','drive'))
        vectors = [idle.get('start_truth'),idle.get('end_truth'),phases.get('hover_truth'),
                   goal.get('target_enu'),goal.get('final_truth'),drive.get('start_truth'),
                   drive.get('stop_truth'),drive.get('held_truth'),phases.get('land_truth')]
        if not all(valid_position(p) for p in vectors):
            return False
        start,end,hover,target,arrived,moved,stopped,held,landed = vectors
        passed = (not report.get('error') and idle.get('state',{}).get('armed') is False
            and math.dist(start,end)<.08 and abs(hover[2]-start[2]-3)<.35
            and math.dist(target,arrived)<.25 and math.dist(moved[:2],stopped[:2])>1
            and math.dist(stopped,held)<.2 and abs(landed[2]-start[2])<.1
            and phases.get('land_state',{}).get('armed') is False
            and phases.get('land_state',{}).get('phase')=='idle'
            and finite(report.get('rotor_max_measured_rad_s')) and report['rotor_max_measured_rad_s']>10
            and report.get('truth_messages',0)>100 and finite(report.get('truth_age_wall_s'))
            and 0<=report['truth_age_wall_s']<1)
        if 'gui_altitude' in phases:
            for label,lo,hi in [('up',.5,1.6),('down',-1.6,-.5)]:
                event = phases['gui_altitude'].get(label,{})
                a,b,c = (event.get(k) for k in ('start_truth','stopped_truth','held_truth'))
                if not all(valid_position(p) for p in (a,b,c)):
                    return False
                passed = passed and lo<b[2]-a[2]<hi and math.dist(b,c)<.2
        return bool(passed)
    raise ValueError(f'Unrecognized measured report kind: {kind}')


class SupportMatrixFramework:
    def __init__(self,evidence_index=None,ledger=None,root=ROOT):
        self.root = Path(root)
        self.evidence_index = Path(evidence_index) if evidence_index else self.root/'docs/status/runtime-evidence-index.yaml'
        self.ledger_path = Path(ledger) if ledger else self.root/'docs/status/platform-status.yaml'
        self.records = []
        if self.evidence_index.is_file():
            self.records = (yaml.safe_load(self.evidence_index.read_text()) or {}).get('records',[])
        self.ledger = yaml.safe_load(self.ledger_path.read_text()) if self.ledger_path.is_file() else {}
        self.support_matrix = {}
        self.release_gates = {}

    def path(self,value):
        path = Path(value)
        return path if path.is_absolute() else self.root/path

    def _determine_support_level(self,cell):
        key = (cell.robot_id,cell.environment_id,cell.simulator,cell.task_type,cell.steering_mode)
        matches = [r for r in self.records if tuple(r.get(k,'') for k in (
            'robot_id','environment_id','simulator','task_type','steering_mode'))==key]
        cell.support_level,cell.evidence_level = SupportLevel.NOT_TESTED,EvidenceLevel.NONE
        cell.limitations = ['No verified measurement for this exact robot/map/backend/task/pattern']
        if not matches:
            return
        # The index keeps individual repeats/negative results. A failing latest
        # record stays visible rather than being hidden by an earlier success.
        record = matches[-1]
        try:
            if not record.get('revision') or not record.get('source_hashes') or not record.get('scope'):
                raise ValueError('Missing source provenance or measured scope')
            sources = self.path(record['source_hashes'])
            if not sources.is_file() or not json.loads(sources.read_text()):
                raise ValueError('Missing source-hash manifest')
            report_file = self.path(record['report'])
            payload = report_file.read_bytes()
            if hashlib.sha256(payload).hexdigest()!=record.get('sha256'):
                raise ValueError('Report hash mismatch')
            report = json.loads(payload)
            observed = report.get('drive_configuration')
            if cell.steering_mode and observed is not None:
                configuration = observed.get('configuration', {})
                if (observed.get('passed') is not True
                    or observed.get('expected_mode') != cell.steering_mode
                    or configuration.get('steering_mode') != cell.steering_mode
                    or configuration.get('type') != 'four_wheel_steer'):
                    raise ValueError('Measured drive configuration does not match the indexed steering mode')
            passed = measured_result(report,record['report_kind'])
        except (OSError,ValueError,KeyError,TypeError) as exc:
            cell.limitations = [f'Unverified artifact: {exc}']
            return
        cell.evidence_level = EvidenceLevel.RUNTIME_EVIDENCE
        cell.support_level = SupportLevel.PARTIALLY_SUPPORTED if passed else SupportLevel.EXPERIMENTAL
        cell.test_result = 'passed_screen' if passed else 'failed_screen'
        cell.revision,cell.last_tested = record['revision'],record.get('date','')
        cell.artifacts = [record['report'],record['source_hashes']]
        cell.notes = record['scope']
        cell.limitations = record.get('limitations',[])+['Screen evidence; full ROADMAP mission acceptance remains separate']

    def generate_support_matrix(self):
        for record in self.records:
            cell = SupportCell(*(record[key] for key in ('robot_id','environment_id','simulator','task_type')),
                               steering_mode=record.get('steering_mode',''))
            self._determine_support_level(cell)
            key = '/'.join([cell.robot_id,cell.environment_id,cell.simulator,cell.task_type,cell.steering_mode])
            self.support_matrix[key] = cell
        return len(self.support_matrix)

    def generate_release_gates(self):
        tasks = flatten_tasks(self.ledger.get('tasks',{}))
        # Newly requested robot/world work must also block a full-scope claim.
        groups = {'robot_tasks':sorted(set(f'R5.{i}' for i in range(1,7)) |
                                      {k for k in tasks if k.startswith('R5.')}),
                  'environment_tasks':sorted(set(f'R6.{i}' for i in range(1,5)) |
                                            {k for k in tasks if k.startswith('R6.')}),
                  'algorithm_breadth':[f'R7.{i}' for i in range(1,9)],
                  'backend_qualification':[f'R8.{i}' for i in range(1,4)],
                  'reproduction_and_comparisons':['R3.6','R4.3','R9.1','R9.2']}
        for name,ids in groups.items():
            blockers = [f"{key}: {tasks.get(key,{}).get('state','missing')}" for key in ids
                        if tasks.get(key,{}).get('state')!='done' or not tasks[key].get('evidence')]
            self.release_gates[name] = ReleaseGate(name,
                status=ReleaseStatus.BLOCKED if blockers else ReleaseStatus.READY,
                required_for=['full_release'],blocking_issues=blockers,evidence_required=ids,
                decision='Derived from current task states and evidence references; no scope waiver recorded')
        return self.release_gates

    def report(self):
        self.generate_support_matrix()
        self.generate_release_gates()
        required_pass = all(g.status==ReleaseStatus.READY for g in self.release_gates.values())
        return {'generated_utc':datetime.now(timezone.utc).isoformat(),'task':'R9.3',
                'status':'ready' if required_pass else 'partial',
                'qualification':'Exact hashed screens only; artifact validation does not rerun a mission',
                'unlisted_cells':'not_tested',
                'support_cells':[c.to_dict() for c in self.support_matrix.values()],
                'release_gates':{k:g.to_dict() for k,g in self.release_gates.items()},
                'acceptance_criteria':{'required_tasks_pass':required_pass}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-index',type=Path)
    parser.add_argument('--ledger',type=Path)
    parser.add_argument('--out-directory',type=Path,default=ROOT/'log/status')
    parser.add_argument('--require-release-ready',action='store_true')
    args = parser.parse_args()
    report = SupportMatrixFramework(args.evidence_index,args.ledger).report()
    args.out_directory.mkdir(parents=True,exist_ok=True)
    output = args.out_directory/'support-matrix-current.json'
    output.write_text(json.dumps(report,indent=2)+'\n')
    lines = ['# Measured support screens','',report['qualification'],'',
             'Unlisted combinations are untested. No row establishes full mission support.','',
             '| Robot | Map | Backend | Task / pattern | Result |','|---|---|---|---|---|']
    for cell in report['support_cells']:
        lines.append(f"| {cell['robot_id']} | {cell['environment_id']} | {cell['simulator']} | "
                     f"{cell['task_type']} / {cell['steering_mode']} | {cell['test_result']} |")
    lines.extend(['','Full release blockers:',''])
    lines.extend(f'- {issue}' for gate in report['release_gates'].values() for issue in gate['blocking_issues'])
    (args.out_directory/'support-matrix-current.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'output':str(output),'status':report['status'],
                      'measured_cells':len(report['support_cells']),'unlisted_cells':'not_tested'}))
    return 1 if args.require_release_ready and report['status']!='ready' else 0


if __name__=='__main__':
    raise SystemExit(main())
