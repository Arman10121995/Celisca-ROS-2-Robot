#!/usr/bin/env python3
"""Summarise each coordinator run's jobs in the fields of the October 15
summary_rounds33.json: state, measured goals, startup rejections, wall time,
real-time factor, peak RSS/CPU and cleanup.

usage: summarize.py <run-root> <run-name>... > summary.json
"""
import json
from pathlib import Path
import sys


def job_summary(folder):
    run = json.loads((folder/'run.json').read_text())
    result = run.get('mission_result') or {}
    if not result and (folder/'mission-result.json').is_file():
        result = json.loads((folder/'mission-result.json').read_text())
    telemetry = run.get('telemetry') or {}
    samples = [json.loads(line) for line in (folder/'resources.jsonl').read_text().splitlines() if line.strip()] \
        if (folder/'resources.jsonl').is_file() else []
    goals = [dict(requested=g['requested'], position_error_m=round(g['position_error_m'], 3),
                  heading_error_deg=round(abs(g['heading_error_rad'])*57.29578, 2),
                  nav2_status=g.get('nav2_status'), sim_time=g.get('sim_time'))
             for g in result.get('goals', []) if 'position_error_m' in g]
    return dict(id=run['id'], backend=run['selection']['backend'], ros_domain_id=run['ros_domain_id'],
                partition=run.get('partition'), state=run['state'],
                mission_success=result.get('success'), mission_state=result.get('state'),
                mission_reason=result.get('reason') or None, mission_result_error=run.get('mission_result_error'),
                goals=goals, goal_rejections_before_nav2_active=result.get('goal_rejections_before_nav2_active'),
                path_length_m=result.get('physical_path_length_m'), wall_duration_s=run.get('wall_duration_s'),
                real_time_factor=round(telemetry['real_time_factor'], 3) if telemetry.get('real_time_factor') else None,
                peak_rss_mb=round(max((s['rss_mb'] for s in samples), default=0)),
                peak_cpu_percent=round(max((s['cpu_percent'] for s in samples), default=0)), resource_samples=len(samples),
                budget_rss_mb=(run.get('requested_budgets') or {}).get('max_rss_mb'),
                cleanup_complete=run.get('cleanup_complete'),
                observed_surviving_pids=run.get('observed_surviving_pids'), revision=(run.get('source') or {}).get('base_revision'))


def main():
    root = Path(sys.argv[1])
    out = {}
    for name in sys.argv[2:]:
        folders = sorted(p.parent for p in (root/name).glob('*/run.json'))
        out[name] = [job_summary(folder) for folder in folders]
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
