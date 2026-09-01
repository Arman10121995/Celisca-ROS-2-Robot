"""
P5 - Algorithm breadth: five integrated implementations per required category.

Validates that every required algorithm category now has at least five
`integrated` implementations (P5.1-P5.6), that the P5.7 normalization contract
is satisfied (each carries implementation package + input/output contracts),
that the 13 new bumperbot_algorithms nodes are on disk and their pure-Python
logic is correct, and that cross-reference validation passes.
"""

import sys
import unittest
from pathlib import Path

import yaml

_SRC_PACKAGE_DIR = Path(__file__).resolve().parent.parent
if _SRC_PACKAGE_DIR.name == "robot_lab_registry":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_registry" in p)
    ]
    if str(_SRC_PACKAGE_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_PACKAGE_DIR))
    for _stale in [m for m in list(sys.modules)
                   if m == "robot_lab_registry" or m.startswith("robot_lab_registry.")]:
        del sys.modules[_stale]

from robot_lab_registry.catalog import Registry
from robot_lab_registry.validation import validate_cross_references

REQUIRED_CATEGORIES = [
    "perception", "localization", "state_estimation", "sensor_fusion",
    "global_planning", "local_planning", "control",
]

# The 13 new P5 algorithms backed by the bumperbot_algorithms package.
NEW_ALGORITHMS = [
    "obstacle_detector", "scan_clusterer", "pointcloud_segmenter",
    "dead_reckoning", "ekf_3d_estimator", "motion_model_estimator",
    "pose_graph_estimator", "wheel_imu_fusion", "gps_odom_fusion",
    "complementary_imu", "rrt_planner", "voronoi_planner", "follow_the_gap",
]


class CategoryCoverageTests(unittest.TestCase):
    """Every required category has >=5 integrated implementations."""

    @classmethod
    def setUpClass(cls):
        cfg = Path(__file__).parent.parent / "config"
        cls.registry = Registry(cfg)
        cls.registry.load(cfg)

    def test_five_integrated_per_category(self):
        algos = self.registry.algorithms.get_all()
        counts = {}
        for a in algos.values():
            if a.get("status") == "integrated":
                counts[a["category"]] = counts.get(a["category"], 0) + 1
        for cat in REQUIRED_CATEGORIES:
            self.assertGreaterEqual(
                counts.get(cat, 0), 5,
                f"category '{cat}' has {counts.get(cat, 0)} integrated (<5)",
            )

    def test_each_integrated_has_contracts(self):
        algos = self.registry.algorithms.get_all()
        for a in algos.values():
            if a.get("status") != "integrated":
                continue
            self.assertTrue(a.get("implementation", {}).get("package"),
                            f"{a['id']} missing implementation.package")
            self.assertTrue(a.get("input_contract"), f"{a['id']} missing input_contract")
            self.assertTrue(a.get("output_contract"), f"{a['id']} missing output_contract")

    def test_new_algorithms_registered_integrated(self):
        algos = self.registry.algorithms.get_all()
        for aid in NEW_ALGORITHMS:
            self.assertIn(aid, algos, f"algorithm '{aid}' missing")
            self.assertEqual(algos[aid]["status"], "integrated")
            self.assertEqual(
                algos[aid]["implementation"]["package"], "bumperbot_algorithms",
                f"algorithm '{aid}' should use bumperbot_algorithms",
            )


class NodeAssetTests(unittest.TestCase):
    """The new algorithm node modules exist on disk."""

    def setUp(self):
        self.pkg = Path(__file__).resolve().parents[4] / "src/bumperbot_algorithms/bumperbot_algorithms"

    def test_node_modules_exist(self):
        modules = ["perception", "localization", "state_estimation",
                   "sensor_fusion", "global_planning", "local_planning"]
        for m in modules:
            self.assertTrue((self.pkg / f"{m}.py").is_file(), f"missing module {m}.py")


class AlgorithmLogicTests(unittest.TestCase):
    """Pure-Python algorithm logic behaves correctly."""

    def test_follow_the_gap_steers_toward_gap(self):
        from bumperbot_algorithms.local_planning import FollowTheGap
        ftg = FollowTheGap()
        # scan with a clear gap on the left, obstacle on the right
        ranges = [10.0] * 60 + [0.2] * 30 + [10.0] * 30
        linear, angular = ftg.steer(-1.5708, 0.01745, ranges)
        self.assertGreater(linear, 0.0)
        self.assertLess(angular, 0.0)  # steers toward the left gap (negative theta region)

    def test_rrt_planner_finds_path(self):
        from bumperbot_algorithms.global_planning import RRTPlanner
        def is_free(x, y):
            return abs(x) > 0.6 or abs(y) > 0.6  # central square obstacle
        planner = RRTPlanner(step=0.8, max_iter=2000, goal_tol=0.5)
        path = planner.plan((-5.0, -5.0), (5.0, 5.0), is_free,
                            (-6.0, -6.0, 6.0, 6.0), seed=1)
        self.assertTrue(len(path) >= 2, "RRT should find a path")
        self.assertAlmostEqual(path[0][0], -5.0, delta=0.5)
        self.assertAlmostEqual(path[-1][0], 5.0, delta=0.6)

    def test_dead_reckoning_integrates(self):
        from bumperbot_algorithms.localization import DeadReckoning
        dr = DeadReckoning()
        # move forward 1 m/s for 2 s and 0.5 rad/s for 2 s
        state = dr.integrate(1.0, 0.5, 2.0)
        self.assertAlmostEqual(state[2], 1.0, delta=1e-6)
        self.assertGreater(state[0], 0.5)  # moved in x

    def test_ekf_update_converges(self):
        from bumperbot_algorithms.state_estimation import EKF3DEstimator
        ekf = EKF3DEstimator()
        ekf.predict(0.1)
        z = [1.0, 0.0, 0.0, 0.5, 0.0, 0.0]
        for _ in range(10):
            ekf.predict(0.1)
            ekf.update(z)
        s = ekf.state()
        self.assertAlmostEqual(s[0], 1.0, delta=0.5)

    def test_complementary_fusion_blends(self):
        from bumperbot_algorithms.sensor_fusion import WheelImuFusion
        f = WheelImuFusion(alpha=0.5)
        for _ in range(10):
            f.fuse(1.0, 0.5, 0.1)
        vx, yaw = f.fuse(1.0, 0.5, 0.1)
        self.assertGreater(yaw, 0.0)

    def test_obstacle_detector_clusters(self):
        from bumperbot_algorithms.perception import ObstacleDetector
        od = ObstacleDetector()
        points = [(0.0, 0.0), (0.1, 0.1), (5.0, 5.0)]
        clusters = od.detect(points)
        self.assertEqual(len(clusters), 2)

    def test_follow_the_gap_handles_full_obstacle(self):
        from bumperbot_algorithms.local_planning import FollowTheGap
        ftg = FollowTheGap()
        linear, angular = ftg.steer(-1.5708, 0.01745, [0.2] * 120)
        self.assertEqual(angular, 0.0)


class CrossReferenceTests(unittest.TestCase):
    def test_cross_reference_passes(self):
        cfg = Path(__file__).parent.parent / "config"
        reg = Registry(cfg)
        reg.load(cfg)
        result = validate_cross_references(reg)
        self.assertTrue(result.valid, f"errors: {result.errors}")


if __name__ == "__main__":
    unittest.main()
