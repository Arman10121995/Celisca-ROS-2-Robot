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

        # Direct CLI launches of the three cars resolve auto to a
        # curvature-aware pair. Explicit incompatible choices are rejected.
        car_global, car_local = module._compatible_planners(
            "ackermann", "auto", "auto")
        self.assertEqual(car_global, "nav2_smac_planner/SmacPlannerHybrid")
        self.assertEqual(car_local,
                         "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
        with self.assertRaisesRegex(ValueError, "curvature-aware"):
            module._compatible_planners("ackermann", "nav2_smac_planner/SmacPlanner2D",
                                        "dwb_core::DWBLocalPlanner")
        self.assertEqual(module._compatible_planners("diff", "nav2_smac_planner/SmacPlanner2D",
                                                     "dwb_core::DWBLocalPlanner"),
                         ("nav2_smac_planner/SmacPlanner2D", "dwb_core::DWBLocalPlanner"))

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

    def test_rviz_goals_go_through_the_relay(self):
        """RViz goals are restamped, not sent straight to the navigator.

        nav2's RViz panel stamps goals with the system clock
        (``Nav2Panel::onNewGoal``: ``rclcpp::Clock().now()``), which a
        simulated planner rejects with "extrapolation into the future" before
        planning.  The navigation launch must start the relay, and the nav
        RViz config must offer the plain SetGoal tool on /goal_pose instead
        of the panel's GoalTool.
        """
        nav_launch = (_SRC_ADAPTER_DIR.parent / "robot_lab_navigation" /
                      "launch" / "navigation.launch.py")
        text = nav_launch.read_text()
        self.assertIn("nav2_goal_relay", text)
        self.assertIn('executable="nav2_goal_relay.py"', text)

        rviz = (_SRC_ADAPTER_DIR.parent / "robot_lab_localization" / "rviz" /
                "nav2_default_view.rviz")
        rviz_text = rviz.read_text()
        self.assertIn("Class: rviz_default_plugins/SetGoal", rviz_text)
        self.assertIn("Value: /robot_lab/goal_pose", rviz_text)
        self.assertNotIn("Value: /goal_pose\n", rviz_text)
        self.assertNotIn("Class: nav2_rviz_plugins/GoalTool", rviz_text)
        self.assertNotIn("Class: nav2_rviz_plugins/Navigation 2", rviz_text)


if __name__ == "__main__":
    unittest.main()
