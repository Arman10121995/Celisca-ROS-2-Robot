from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, TextIO

try:
    import matplotlib.pyplot as plt
    import numpy as np
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class OutputGenerator:
    """Generate benchmark reports in multiple formats."""

    def __init__(self) -> None:
        pass

    @staticmethod
    def write_json_report(
        results: Iterable[Any],
        output_path: str,
        summary: Optional[Dict[str, Any]] = None,
        comparison: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Write benchmark results as JSON."""
        try:
            payload = {
                'schema_version': '1.0',
                'summary': summary or {},
                'comparison': comparison or {},
                'results': [],
            }

            for result in results:
                if isinstance(result, dict):
                    payload['results'].append(result)
                else:
                    payload['results'].append(
                        result.to_dict() if hasattr(result, 'to_dict') else vars(result)
                    )

            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, indent=2)

            return True
        except Exception as e:
            print(f"Failed to write JSON report: {e}")
            return False

    @staticmethod
    def write_csv_report(
        results: Iterable[Any],
        output_path: str,
    ) -> bool:
        """Write benchmark results as CSV table."""
        try:
            rows = []
            for result in results:
                if isinstance(result, dict):
                    rows.append(result)
                else:
                    rows.append(
                        result.to_dict() if hasattr(result, 'to_dict') else vars(result)
                    )

            if not rows:
                return False

            fieldnames = list(rows[0].keys())

            with open(output_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)

            return True
        except Exception as e:
            print(f"Failed to write CSV report: {e}")
            return False

    @staticmethod
    def write_markdown_table(
        results: Iterable[Any],
        output_path: str,
        title: str = 'Benchmark Results',
    ) -> bool:
        """Write benchmark results as Markdown table."""
        try:
            rows = []
            for result in results:
                if isinstance(result, dict):
                    rows.append(result)
                else:
                    rows.append(
                        result.to_dict() if hasattr(result, 'to_dict') else vars(result)
                    )

            if not rows:
                return False

            fieldnames = [
                'seed',
                'success',
                'elapsed_seconds',
                'path_length_m',
                'collision_count',
                'min_clearance_m',
            ]

            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(f"# {title}\n\n")
                f.write("| " + " | ".join(fieldnames) + " |\n")
                f.write("|" + "|".join(["-" * 15] * len(fieldnames)) + "|\n")

                for row in rows:
                    values = [str(row.get(f, '')) for f in fieldnames]
                    f.write("| " + " | ".join(values) + " |\n")

            return True
        except Exception as e:
            print(f"Failed to write Markdown table: {e}")
            return False

    @staticmethod
    def write_plot(
        results: Iterable[Any],
        output_path: str,
        metric_x: str = 'seed',
        metric_y: str = 'elapsed_seconds',
        title: str = 'Benchmark Performance',
    ) -> bool:
        """Write benchmark results as plot (requires matplotlib)."""
        if not HAS_MATPLOTLIB:
            print("matplotlib not available; skipping plot generation")
            return False

        try:
            rows = []
            for result in results:
                if isinstance(result, dict):
                    rows.append(result)
                else:
                    rows.append(
                        result.to_dict() if hasattr(result, 'to_dict') else vars(result)
                    )

            if not rows:
                return False

            x_values = [float(row.get(metric_x, 0)) for row in rows]
            y_values = [float(row.get(metric_y, 0)) for row in rows]

            plt.figure(figsize=(10, 6))
            plt.scatter(x_values, y_values, s=100, alpha=0.6)
            plt.xlabel(metric_x)
            plt.ylabel(metric_y)
            plt.title(title)
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(output_path, dpi=100)
            plt.close()

            return True
        except Exception as e:
            print(f"Failed to write plot: {e}")
            return False

    @staticmethod
    def write_comparison_html(
        results: Iterable[Any],
        output_path: str,
        title: str = 'Benchmark Comparison',
    ) -> bool:
        """Write benchmark comparison as HTML page."""
        try:
            rows = []
            for result in results:
                if isinstance(result, dict):
                    rows.append(result)
                else:
                    rows.append(
                        result.to_dict() if hasattr(result, 'to_dict') else vars(result)
                    )

            if not rows:
                return False

            fieldnames = list(rows[0].keys())

            html_lines = [
                '<!DOCTYPE html>',
                '<html>',
                '<head>',
                '<meta charset="utf-8">',
                f'<title>{title}</title>',
                '<style>',
                'body { font-family: Arial, sans-serif; margin: 20px; }',
                'table { border-collapse: collapse; width: 100%; }',
                'th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }',
                'th { background-color: #4CAF50; color: white; }',
                'tr:nth-child(even) { background-color: #f2f2f2; }',
                '</style>',
                '</head>',
                '<body>',
                f'<h1>{title}</h1>',
                '<table>',
                '<tr>' + ''.join(f'<th>{field}</th>' for field in fieldnames) + '</tr>',
            ]

            for row in rows:
                html_lines.append(
                    '<tr>'
                    + ''.join(f'<td>{row.get(field, "")}</td>' for field in fieldnames)
                    + '</tr>'
                )

            html_lines.extend(['</table>', '</body>', '</html>'])

            with open(output_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(html_lines))

            return True
        except Exception as e:
            print(f"Failed to write HTML comparison: {e}")
            return False
