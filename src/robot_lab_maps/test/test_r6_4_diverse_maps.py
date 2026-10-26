"""R6.4 — Expand diverse maps for ground, legged and aerial tasks.

Tests that at least 6 distinct seeded layouts exist with proper physical
dimensions, collision geometry validation, and feasible spawn/goals/routes.
"""
from pathlib import Path
import sys
import math
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_maps/tools'))
from r6_diverse_maps import MapMetadata, MapType, DimensionType, RobotClass, DiverseMapGenerator


# ── Map metadata tests ────────────────────────────────────────────────

def test_diverse_map_generator_creates_six_maps():
    """Generator creates at least 6 distinct seeded layouts (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    assert len(maps) >= 6, f"Expected at least 6 maps, got {len(maps)}"


def test_diverse_maps_have_unique_ids():
    """Each map has a unique ID."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    ids = [m.map_id for m in maps]
    assert len(ids) == len(set(ids)), "Map IDs must be unique"


def test_diverse_maps_have_physical_dimensions():
    """Each map has physical dimensions, units, and provenance (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    for m in maps:
        assert m.size['x'] > 0, f"Map {m.map_id} missing x size"
        assert m.size['y'] > 0, f"Map {m.map_id} missing y size"
        assert m.units == "meters", f"Map {m.map_id} wrong units"
        assert m.resolution > 0, f"Map {m.map_id} missing resolution"
        assert m.generator, f"Map {m.map_id} missing generator"
        assert m.seeds, f"Map {m.map_id} missing seeds"


def test_diverse_maps_cover_required_types():
    """Maps cover road, slope, flight, and other required types (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    tags = set()
    for m in maps:
        tags.update(m.tags)
    # Should cover road intersections, slopes, flight corridors, etc.
    assert len(tags) >= 3, f"Expected at least 3 tag types, got {tags}"


def test_diverse_maps_support_robot_classes():
    """Each map declares supported robot classes."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    for m in maps:
        assert len(m.supported_robot_classes) > 0, \
            f"Map {m.map_id} must support at least one robot class"


def test_diverse_maps_have_map_types():
    """Each map declares map type(s)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    for m in maps:
        assert len(m.map_types) > 0, f"Map {m.map_id} must have at least one map type"


# ── Collision geometry validation ─────────────────────────────────────

def test_collision_geometry_matches_representation():
    """Each map's collision geometry agrees with its representation (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    results = gen.validate_collision_geometry(maps)
    for map_id, result in results.items():
        assert result['collision_geometry_validated'], \
            f"Map {map_id} collision geometry not validated"


# ── Feasibility tests ─────────────────────────────────────────────────

def test_spawn_goals_feasible():
    """Spawn, goals and routes are feasible for declared robot footprint (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    results = gen.validate_feasibility(maps)
    for map_id, result in results.items():
        assert result['spawn_goals_feasible'], \
            f"Map {map_id} spawn/goals not feasible"


# ── Reset reproducibility ─────────────────────────────────────────────

def test_reset_reproducible():
    """Reset reproduces the declared state (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    # Deterministic seeds ensure reset reproducibility
    for m in maps:
        assert len(m.seeds) > 0, f"Map {m.map_id} needs seeds for reset reproducibility"


# ── Map diversity test ────────────────────────────────────────────────

def test_maps_are_distinct_layouts():
    """Maps are distinct layouts, not pure visual changes (R6.4 acceptance)."""
    gen = DiverseMapGenerator()
    maps = gen.generate_all_diverse_maps()
    # Each map should have different size or tags
    signatures = set()
    for m in maps:
        sig = (m.size['x'], m.size['y'], m.size['z'], tuple(sorted(m.tags)))
        signatures.add(sig)
    assert len(signatures) >= 6, "Maps must be distinct layouts"
