#!/usr/bin/env python3
"""Derive a small convex roller contact mesh from the MIT FUJI visual asset."""
import argparse
from pathlib import Path
import json
import numpy as np
import trimesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    assets = Path(__file__).resolve().parents[1] / 'src/robot_lab_robots/holonomic_wheels/third_party/fuji_mecanum'
    original = trimesh.load(str(assets / 'roller.stl'), process=True)
    points = []
    for a in np.linspace(-np.pi / 2, np.pi / 2, 9):
        for theta in np.arange(12) * 2 * np.pi / 12:
            direction = np.array([np.sin(a), np.cos(a) * np.cos(theta), np.cos(a) * np.sin(theta)])
            points.append(original.vertices[np.argmax(original.vertices @ direction)])
    proxy = trimesh.convex.convex_hull(np.array(points))
    destination = assets / 'roller_collision.stl'
    if args.check:
        existing = trimesh.load(str(destination), process=True)
        expected_vertices = np.unique(proxy.vertices.astype(np.float32), axis=0)
        actual_vertices = np.unique(existing.vertices.astype(np.float32), axis=0)
        if expected_vertices.shape != actual_vertices.shape or not np.allclose(expected_vertices, actual_vertices, atol=1e-8):
            raise SystemExit('Roller collision proxy differs from the upstream-derived geometry')
    else:
        proxy.export(str(destination))
    directions = np.random.default_rng(5402).normal(size=(4000, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    errors = (np.max(original.vertices @ directions.T, axis=0)
              - np.max(proxy.vertices @ directions.T, axis=0)) * 0.1 / 0.205
    if not proxy.is_watertight or errors.max() > 0.0005:
        raise SystemExit('Proxy fails the watertight / 0.5 mm support-error contract')
    print(json.dumps({'source_faces': len(original.faces), 'proxy_faces': len(proxy.faces),
                      'seed': 5402, 'directions': 4000, 'max_support_error_m': float(errors.max())}))


if __name__ == '__main__':
    main()
