#!/usr/bin/env python3
"""R8.3: summarize robot-lab-concurrent campaign runs.

Reads each run directory written by `ros2 run robot_lab_benchmark
robot-lab-concurrent ... --output <run>` and prints, per job: backend, ROS
domain, coordinator state, the measured Nav2 workload result (per-goal truth
error and heading error, Nav2 statuses, goal rejections before Nav2 was
active), wall duration, real-time factor, peak memory and CPU from the
coordinator's resource trace, and cleanup.

  python3 scripts/r8_3_campaign_summary.py --out summary.json <run_dir> ...
"""
import argparse
import json
import math
import os


def job_summary(directory):
    run = json.load(open(os.path.join(directory, 'run.json')))
    mission = run.get('mission_result') or {}
    peak_rss = peak_cpu = 0.0
    samples = 0
    path = os.path.join(directory, 'resources.jsonl')
    if os.path.exists(path):
        for line in open(path):
            try:
                row = json.loads(line)
            except ValueError:
                continue
            samples += 1
            peak_rss = max(peak_rss, float(row.get('rss_mb') or 0))
            peak_cpu = max(peak_cpu, float(row.get('cpu_percent') or 0))
    telemetry = {}
    path = os.path.join(directory, 'telemetry-summary.json')
    if os.path.exists(path):
        telemetry = json.load(open(path))
    goals = [{'requested': g.get('requested'),
              'position_error_m': round(g['position_error_m'], 3),
              'heading_error_deg': round(math.degrees(g['heading_error_rad']), 2),
              'nav2_status': g.get('nav2_status'), 'sim_time': round(g.get('sim_time') or 0, 1)}
             for g in mission.get('goals') or []]
    return {
        'id': run['id'], 'backend': (run.get('selection') or {}).get('backend'),
        'ros_domain_id': run.get('ros_domain_id'), 'state': run.get('state'),
        'mission_success': mission.get('success'), 'mission_state': mission.get('state'),
        'mission_reason': mission.get('reason') or None, 'mission_result_error': run.get('mission_result_error'),
        'goals': goals,
        'goal_rejections_before_nav2_active': mission.get('goal_rejections_before_nav2_active'),
        'path_length_m': round(mission['physical_path_length_m'], 2) if 'physical_path_length_m' in mission else None,
        'wall_duration_s': round(run.get('wall_duration_s') or 0, 1),
        'real_time_factor': round(telemetry['real_time_factor'], 3) if telemetry.get('real_time_factor') else None,
        'peak_rss_mb': round(peak_rss), 'peak_cpu_percent': round(peak_cpu), 'resource_samples': samples,
        'budget_rss_mb': (run.get('requested_budgets') or {}).get('max_rss_mb'),
        'cleanup_complete': run.get('cleanup_complete'),
        'observed_surviving_pids': run.get('observed_surviving_pids'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    summary = {}
    for run in args.runs:
        jobs = [os.path.join(run, name) for name in sorted(os.listdir(run))
                if os.path.exists(os.path.join(run, name, 'run.json'))]
        summary[os.path.basename(os.path.normpath(run))] = [job_summary(job) for job in jobs]
    with open(args.out, 'w') as handle:
        json.dump(summary, handle, indent=1)
    for run, jobs in summary.items():
        print('==', run)
        for j in jobs:
            print('  %-8s domain %s state %-24s success %-5s rss %5s MB cpu %4s%% rtf %s wall %ss cleanup %s %s' % (
                j['backend'], j['ros_domain_id'], j['state'], j['mission_success'], j['peak_rss_mb'],
                j['peak_cpu_percent'], j['real_time_factor'], j['wall_duration_s'], j['cleanup_complete'],
                [(g['position_error_m'], g['heading_error_deg']) for g in j['goals']]))


if __name__ == '__main__':
    main()
