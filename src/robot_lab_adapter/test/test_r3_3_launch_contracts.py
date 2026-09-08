"""
R3.3 Launch-Contract Tests (integration tier)

These tests import ROS launch modules (`launch`, `launch_ros`,
`ament_index_python`) so they run in the integration tier where a sourced
ROS 2 environment is available, not in the plain-Python fast tier.

They verify:
- the navigation stack switches planner plugins via launch-provided values,
- the fixed select_components.launch.py (string + LaunchConfiguration
  concatenation bug) constructs a LaunchDescription.
"""

import importlib.util
import sys
import unittest
from pathlib import Path

_SRC_ADAPTER_DIR = Path(__file__).resolve().parent.parent
if _SRC_ADAPTER_DIR.name == "robot_lab_adapter":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_adapter" in p)
    ]
    if str(_SRC_ADAPTER_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_ADAPTER_DIR))

ADAPTER_LAUNCH_DIR = _SRC_ADAPTER_DIR / "launch"


def load_launch_module(name):
    """Load a launch file module by filename from the adapter launch dir."""
    spec = importlib.util.spec_from_file_location(
        name, str(ADAPTER_LAUNCH_DIR / f"{name}.launch.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class R33LaunchContractTests(unittest.TestCase):
    """R3.3: launch files accept and switch planner plugins."""

    def test_navigation_launch_planner_overrides(self):
        """The navigation stack switches plugins via launch-provided values."""
        nav_launch = _SRC_ADAPTER_DIR.parent / "robot_lab_navigation" / "launch" / "navigation.launch.py"
        self.assertTrue(nav_launch.is_file(), nav_launch)
        spec = importlib.util.spec_from_file_location("r33_navigation_launch",
                                                      str(nav_launch))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        # Defaults preserve the current YAML behavior.
        planner_default = module._planner_parameter_overrides(
            "planner_server", "nav2_smac_planner/SmacPlanner2D",
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
        self.assertEqual(planner_default,
                         [{"GridBased.plugin": "nav2_smac_planner/SmacPlanner2D"}])

        # Switching planners changes the active plugin parameter.
        switched = module._planner_parameter_overrides(
            "planner_server", "nav2_navfn_planner/NavfnPlanner",
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
        self.assertEqual(switched,
                         [{"GridBased.plugin": "nav2_navfn_planner/NavfnPlanner"}])

        # The DWB local planner also carries its supporting critics params.
        controller_default = module._planner_parameter_overrides(
            "controller_server", "nav2_smac_planner/SmacPlanner2D",
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
        self.assertEqual(
            controller_default,
            [{"FollowPath.plugin":
               "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"}])
        controller_dwb = module._planner_parameter_overrides(
            "controller_server", "nav2_smac_planner/SmacPlanner2D",
            "dwb_core::DWBLocalPlanner")
        self.assertEqual(controller_dwb[0]["FollowPath.plugin"],
                         "dwb_core::DWBLocalPlanner")
        self.assertIn("FollowPath.critics", controller_dwb[0])

    def test_select_components_launch_constructs(self):
        """Regression: string + LaunchConfiguration concatenation raised TypeError."""
        module = load_launch_module("select_components")
        description = module.generate_launch_description()
        self.assertIsNotNone(description)

    def test_no_duplicate_navigation_includes(self):
        """The bringup starts the navigation stack exactly once."""
        bringup_launch = (_SRC_ADAPTER_DIR.parent / "robot_lab_bringup" /
                          "launch" / "simulated_robot.launch.py")
        text = bringup_launch.read_text()
        self.assertEqual(text.count('"navigation.launch.py"'), 1)


if __name__ == "__main__":
    unittest.main()