from __future__ import annotations

from typing import Any, Dict, Iterable, List


def aggregate_results(results: Iterable[Any]) -> Dict[str, Any]:
    """Compute a minimal aggregate summary for a collection of benchmark results."""
    rows = list(results)
    if not rows:
        return {
            'total_runs': 0,
            'success_count': 0,
            'failure_count': 0,
            'avg_elapsed_seconds': 0.0,
            'avg_path_length_m': 0.0,
            'avg_collision_count': 0.0,
            'avg_min_clearance_m': 0.0,
        }

    success_count = sum(1 for row in rows if getattr(row, 'success', False))
    failure_count = len(rows) - success_count
    avg_elapsed = sum(float(getattr(row, 'elapsed_seconds', 0.0)) for row in rows) / len(rows)
    avg_path = sum(float(getattr(row, 'path_length_m', 0.0)) for row in rows) / len(rows)
    avg_collision = sum(float(getattr(row, 'collision_count', 0.0)) for row in rows) / len(rows)
    avg_clearance = sum(float(getattr(row, 'min_clearance_m', 0.0)) for row in rows) / len(rows)

    return {
        'total_runs': len(rows),
        'success_count': success_count,
        'failure_count': failure_count,
        'avg_elapsed_seconds': avg_elapsed,
        'avg_path_length_m': avg_path,
        'avg_collision_count': avg_collision,
        'avg_min_clearance_m': avg_clearance,
    }


def compare_results(results: Iterable[Any]) -> Dict[str, Any]:
    """Rank runs by successful completion, time, path length, and collisions."""
    rows = list(results)
    if not rows:
        return {'ranking': [], 'best_run': None}

    ranked = sorted(
        rows,
        key=lambda row: (
            0 if getattr(row, 'success', False) else 1,
            float(getattr(row, 'elapsed_seconds', 0.0)),
            float(getattr(row, 'path_length_m', 0.0)),
            int(getattr(row, 'collision_count', 0)),
        ),
    )

    ranking = []
    for index, row in enumerate(ranked, start=1):
        ranking.append({
            'rank': index,
            'seed': getattr(row, 'seed', None),
            'success': getattr(row, 'success', False),
            'elapsed_seconds': getattr(row, 'elapsed_seconds', 0.0),
            'path_length_m': getattr(row, 'path_length_m', 0.0),
            'collision_count': getattr(row, 'collision_count', 0),
        })

    return {
        'ranking': ranking,
        'best_run': ranking[0] if ranking else None,
        'summary': aggregate_results(rows),
    }
