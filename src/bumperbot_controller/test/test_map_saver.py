# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

from bumperbot_controller.map_saver import MapSaver, save_occupancy_map  # noqa: E402


def test_numpy_map_preserves_occupancy_semantics():
    result = MapSaver().save_map_to_numpy(
        free_counts={(0, 0): 2},
        occupied_counts={(2, 0): 3},
        min_x=0,
        max_x=2,
        min_y=0,
        max_y=0,
    )
    np.testing.assert_array_equal(result, [[0, -1, 100]])


def test_saved_map_is_valid_ros_pgm_and_yaml(tmp_path):
    pgm_path = save_occupancy_map(
        free_counts={(0, 0): 2},
        occupied_counts={(2, 0): 3},
        origin_x=-1.0,
        origin_y=-2.0,
        resolution=0.1,
        output_dir=tmp_path,
        map_name="room",
    )

    pgm_lines = [
        line
        for line in pgm_path.read_text(encoding="utf-8").splitlines()
        if not line.startswith("#")
    ]
    assert pgm_lines == ["P2", "3 1", "255", "254 205 0"]

    metadata = yaml.safe_load((tmp_path / "room.yaml").read_text(encoding="utf-8"))
    assert metadata["image"] == "room.pgm"
    assert metadata["resolution"] == 0.1
    assert metadata["origin"] == [-1.0, -2.0, 0.0]


def test_empty_map_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="No map data"):
        save_occupancy_map({}, {}, 0.0, 0.0, 0.1, tmp_path, "empty")
