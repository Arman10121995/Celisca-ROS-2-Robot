from __future__ import annotations

from typing import Any, Dict, Iterable, List

from .aggregator import aggregate_results, compare_results


def generate_report(results: Iterable[Any]) -> Dict[str, Any]:
    """Generate a machine-readable benchmark comparison report."""
    rows = list(results)
    comparison = compare_results(rows)
    summary = aggregate_results(rows)

    report = {
        'summary': summary,
        'ranking': comparison['ranking'],
        'best_run': comparison['best_run'],
        'total_runs': summary['total_runs'],
        'success_count': summary['success_count'],
        'failure_count': summary['failure_count'],
    }
    return report
