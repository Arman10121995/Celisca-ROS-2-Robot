from __future__ import annotations

from typing import Any, Dict, Iterable, Optional


class MetricNormalizer:
    """Normalize benchmark metrics for fair comparison."""

    def __init__(
        self,
        path_length_reference: float = 25.0,
        elapsed_seconds_reference: float = 60.0,
        collision_penalty: float = 1.0,
        clearance_target: float = 0.5,
    ) -> None:
        self.path_length_ref = path_length_reference
        self.elapsed_seconds_ref = elapsed_seconds_reference
        self.collision_penalty = collision_penalty
        self.clearance_target = clearance_target

    def normalize_path_efficiency(self, path_length_m: float, elapsed_seconds: float) -> float:
        """Normalize path efficiency (lower is better)."""
        if elapsed_seconds <= 0:
            return 0.0

        efficiency = path_length_m / elapsed_seconds
        normalized = efficiency / (self.path_length_ref / self.elapsed_seconds_ref)
        return min(normalized, 2.0)  # Cap at 2.0

    def normalize_collision_penalty(self, collision_count: int) -> float:
        """Normalize collision penalty (lower is better)."""
        return min(float(collision_count) * self.collision_penalty, 2.0)

    def normalize_clearance(self, min_clearance_m: float) -> float:
        """Normalize clearance metric (higher is better, inverted for ranking)."""
        if min_clearance_m <= 0:
            return 2.0  # Worst case

        penalty = (self.clearance_target - min_clearance_m) / self.clearance_target
        return min(max(penalty, 0.0), 2.0)

    def compute_composite_score(
        self,
        path_length_m: float,
        elapsed_seconds: float,
        collision_count: int,
        min_clearance_m: float,
        weights: Optional[Dict[str, float]] = None,
    ) -> float:
        """Compute composite score from normalized metrics (lower is better)."""
        if weights is None:
            weights = {
                'efficiency': 0.4,
                'collision': 0.3,
                'clearance': 0.3,
            }

        efficiency_score = self.normalize_path_efficiency(path_length_m, elapsed_seconds)
        collision_score = self.normalize_collision_penalty(collision_count)
        clearance_score = self.normalize_clearance(min_clearance_m)

        composite = (
            weights['efficiency'] * efficiency_score
            + weights['collision'] * collision_score
            + weights['clearance'] * clearance_score
        )

        return composite

    def normalize_batch(
        self,
        results: Iterable[Any],
    ) -> list[Dict[str, Any]]:
        """Normalize a batch of benchmark results."""
        normalized_list = []

        for result in results:
            if isinstance(result, dict):
                payload = result
            else:
                payload = result.to_dict() if hasattr(result, 'to_dict') else vars(result)

            success = payload.get('success', False)

            if not success:
                # Failed runs get worst scores
                normalized = {
                    'original': payload,
                    'normalized': {
                        'efficiency': 2.0,
                        'collision': 2.0,
                        'clearance': 2.0,
                        'composite': 2.0,
                    },
                }
            else:
                composite = self.compute_composite_score(
                    path_length_m=payload.get('path_length_m', 0.0),
                    elapsed_seconds=payload.get('elapsed_seconds', 1.0),
                    collision_count=payload.get('collision_count', 0),
                    min_clearance_m=payload.get('min_clearance_m', 0.0),
                )

                normalized = {
                    'original': payload,
                    'normalized': {
                        'efficiency': self.normalize_path_efficiency(
                            payload.get('path_length_m', 0.0),
                            payload.get('elapsed_seconds', 1.0),
                        ),
                        'collision': self.normalize_collision_penalty(
                            payload.get('collision_count', 0),
                        ),
                        'clearance': self.normalize_clearance(
                            payload.get('min_clearance_m', 0.0),
                        ),
                        'composite': composite,
                    },
                }

            normalized_list.append(normalized)

        return normalized_list
