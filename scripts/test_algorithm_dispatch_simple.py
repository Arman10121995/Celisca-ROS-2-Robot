#!/usr/bin/env python3
"""
Simple Algorithm Dispatch Test

Tests the core algorithm dispatch validation without requiring ROS to be fully initialized.
"""

import yaml
import os
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_BRINGUP = os.path.join(_HERE, "src", "robot_lab_bringup")

_ALGORITHMS = os.path.join(_HERE, "src", "robot_lab", "robot_lab_registry", "config", "algorithms.yaml")
_DISPATCH = os.path.join(_BRINGUP, "config", "algorithm_dispatch.yaml")
_MODES = os.path.join(_BRINGUP, "config", "sim_modes.yaml")

_REQUIRED_BREADTH = 5


def _load(path):
    """Load YAML file safely."""
    try:
        with open(path) as handle:
            return yaml.safe_load(handle)
    except FileNotFoundError:
        print(f"WARNING: File not found: {path}")
        return None
    except yaml.YAMLError as e:
        print(f"WARNING: YAML error in {path}: {e}")
        return None


class AlgorithmDispatchTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.algorithms = _load(_ALGORITHMS) or {}
        cls.dispatch = _load(_DISPATCH) or {}
        cls.modes = _load(_MODES) or {}
        
        # Extract algorithms from dispatch if it exists
        cls.dispatch_algorithms = cls.dispatch.get("algorithms", {}) if cls.dispatch else {}

    def test_dispatch_yaml_exists_and_parsable(self):
        """Dispatch YAML file should exist and be parsable."""
        self.assertIsNotNone(self.dispatch, "algorithm_dispatch.yaml should be parsable")
        self.assertIn("algorithms", self.dispatch, "Dispatch should have 'algorithms' key")

    def test_each_category_has_five_runnable_implementations(self):
        """The breadth target: each category must have ≥5 runnable implementations."""
        if not self.dispatch_algorithms:
            self.skipTest("Dispatch algorithms not available")
            
        for category, entries in self.dispatch_algorithms.items():
            runnable = []
            unavailable = []
            
            for algorithm_id, entry in entries.items():
                if isinstance(entry, dict):
                    if any(key in entry for key in ["node", "plugin", "stack"]):
                        runnable.append(algorithm_id)
                    elif "unavailable" in entry:
                        unavailable.append(algorithm_id)
                else:
                    # Direct entries
                    runnable.append(algorithm_id)
            
            with self.subTest(category=category):
                self.assertGreaterEqual(
                    len(runnable), _REQUIRED_BREADTH,
                    f"{category} has only {len(runnable)} runnable implementation(s): {sorted(runnable)}"
                )

    def test_every_dispatch_entry_names_one_mechanism(self):
        """An entry should declare exactly one mechanism: node, plugin, stack or unavailable."""
        if not self.dispatch_algorithms:
            self.skipTest("Dispatch algorithms not available")
            
        for category, entries in self.dispatch_algorithms.items():
            for algorithm_id, entry in entries.items():
                if not isinstance(entry, dict):
                    continue  # Skip non-dict entries
                    
                with self.subTest(category=category, algorithm=algorithm_id):
                    mechanisms = [key for key in ["node", "plugin", "stack", "unavailable"]
                                  if entry.get(key)]
                    self.assertEqual(
                        1, len(mechanisms),
                        f"{category}/{algorithm_id} declares {mechanisms or 'nothing'}"
                    )
                    
                    if entry.get("node"):
                        node_entry = entry["node"]
                        if isinstance(node_entry, dict):
                            self.assertIn("package", node_entry,
                                        f"{category}/{algorithm_id} node entry missing 'package'")
                            self.assertIn("executable", node_entry,
                                        f"{category}/{algorithm_id} node entry missing 'executable'")

    def test_mode_defaults_are_runnable_algorithms(self):
        """A mode's default_algorithm must exist and be launchable."""
        if not self.dispatch_algorithms or not self.modes:
            self.skipTest("Dispatch or modes not available")
            
        modes = self.modes.get("modes", {})
        
        for mode_name, mode in modes.items():
            for step in mode.get("steps") or []:
                category = step.get("algorithm_category")
                default = step.get("default_algorithm")
                if not category or not default or default == "none":
                    continue
                    
                with self.subTest(mode=mode_name, step=step.get("id")):
                    category_entries = self.dispatch_algorithms.get(category, {})
                    entry = category_entries.get(default)
                    self.assertIsNotNone(
                        entry, f"{category} default '{default}' is not dispatchable"
                    )
                    
                    # Check if it's runnable (not unavailable)
                    if isinstance(entry, dict):
                        self.assertFalse(
                            entry.get("unavailable"),
                            f"{category} default '{default}' is not runnable: {entry.get('unavailable')}"
                        )

    def test_each_category_has_enough_runnable_for_breadth(self):
        """Comprehensive test: verify all categories meet the R7.1 breadth requirement."""
        if not self.dispatch_algorithms:
            self.skipTest("Dispatch algorithms not available")
            
        results = {}
        all_passed = True
        
        for category, entries in self.dispatch_algorithms.items():
            runnable = []
            for algorithm_id, entry in entries.items():
                if isinstance(entry, dict):
                    if any(key in entry for key in ["node", "plugin", "stack"]):
                        runnable.append(algorithm_id)
                else:
                    runnable.append(algorithm_id)
            
            passed = len(runnable) >= _REQUIRED_BREADTH
            results[category] = {
                'runnable_count': len(runnable),
                'passed': passed,
                'runnable': runnable
            }
            
            if not passed:
                all_passed = False
        
        # Print summary
        print("\nAlgorithm Dispatch Breadth Results:")
        for category, result in results.items():
            status = "✅" if result['passed'] else "❌"
            print(f"{status} {category}: {result['runnable_count']} runnable implementations")
        
        self.assertTrue(all_passed, "Not all categories have ≥5 runnable implementations")


if __name__ == "__main__":
    unittest.main()