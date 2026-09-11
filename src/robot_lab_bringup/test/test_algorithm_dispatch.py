"""The algorithm selection contract of simulated_robot.launch.py.

Every algorithm category the GUI/CLI can select must resolve into something
concrete at launch time - a node, a plugin switch, or a stack that already
provides it - otherwise a selection is silently ignored and the simulation
does not run what the panel shows.
"""
import importlib.util
import os
import sys
import unittest

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_BRINGUP = os.path.dirname(_HERE)
_SRC = os.path.dirname(_BRINGUP)

_ALGORITHMS = os.path.join(
    _SRC, "robot_lab", "robot_lab_registry", "config", "algorithms.yaml")
_DISPATCH = os.path.join(_BRINGUP, "config", "algorithm_dispatch.yaml")
_MODES = os.path.join(_BRINGUP, "config", "sim_modes.yaml")
_LAUNCH = os.path.join(_BRINGUP, "launch", "simulated_robot.launch.py")

# Categories that must each reach five runnable implementations - the
# platform's stated breadth target.
_REQUIRED_BREADTH = 5


def _load(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


def _launch_module():
    spec = importlib.util.spec_from_file_location(
        "robot_lab_simulated_robot_launch", _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AlgorithmDispatchTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.algorithms = _load(_ALGORITHMS)
        cls.dispatch = _load(_DISPATCH)["algorithms"]
        cls.modes = _load(_MODES)["modes"]
        cls.launch = _launch_module()

    def test_every_registry_algorithm_has_a_dispatch_entry(self):
        """No cataloged algorithm can be selected without a launch decision."""
        missing = []
        for algorithm in self.algorithms:
            category = algorithm.get("category")
            entry = (self.dispatch.get(category) or {}).get(algorithm["id"])
            if entry is None:
                missing.append("%s/%s" % (category, algorithm["id"]))
        self.assertEqual([], missing,
                         "algorithms with no entry in algorithm_dispatch.yaml")

    def test_every_dispatch_entry_names_one_mechanism(self):
        """An entry says exactly how it applies: node, plugin, stack or why not."""
        for category, entries in self.dispatch.items():
            for algorithm_id, entry in entries.items():
                with self.subTest(category=category, algorithm=algorithm_id):
                    mechanisms = [key for key in
                                  ("node", "plugin", "stack", "unavailable")
                                  if entry.get(key)]
                    self.assertEqual(
                        1, len(mechanisms),
                        "%s/%s declares %s" % (category, algorithm_id,
                                               mechanisms or "nothing"))
                    if entry.get("node"):
                        self.assertIn("package", entry["node"])
                        self.assertIn("executable", entry["node"])

    def test_each_category_has_five_runnable_implementations(self):
        """The breadth target counts only algorithms that can actually run."""
        for category, entries in self.dispatch.items():
            runnable = [algorithm_id for algorithm_id, entry in entries.items()
                        if not entry.get("unavailable")]
            with self.subTest(category=category):
                self.assertGreaterEqual(
                    len(runnable), _REQUIRED_BREADTH,
                    "%s has only %d runnable implementation(s): %s"
                    % (category, len(runnable), sorted(runnable)))

    def test_mode_defaults_are_runnable_algorithms(self):
        """A mode's default_algorithm must exist and be launchable."""
        for mode_name, mode in self.modes.items():
            for step in mode.get("steps") or []:
                category = step.get("algorithm_category")
                default = step.get("default_algorithm")
                if not category or not default or default == "none":
                    continue
                with self.subTest(mode=mode_name, step=step.get("id")):
                    entry = (self.dispatch.get(category) or {}).get(default)
                    self.assertIsNotNone(
                        entry, "%s default '%s' is not dispatchable"
                        % (category, default))
                    self.assertFalse(
                        entry.get("unavailable"),
                        "%s default '%s' is not runnable: %s"
                        % (category, default, entry.get("unavailable")))

    def test_mode_steps_only_use_canonical_categories(self):
        """Mode categories and algorithm categories share one taxonomy."""
        for mode_name, mode in self.modes.items():
            for step in mode.get("steps") or []:
                category = step.get("algorithm_category")
                if category is None:
                    continue
                with self.subTest(mode=mode_name, step=step.get("id")):
                    self.assertIn(category, self.launch.ALGORITHM_CATEGORIES)

    def test_launch_declares_one_argument_per_category(self):
        """Each category is selectable on the command line."""
        description = self.launch.generate_launch_description()
        declared = {action.name for action in description.entities
                    if hasattr(action, "name")}
        for category in self.launch.ALGORITHM_CATEGORIES:
            self.assertIn(category, declared)

    def test_selecting_a_category_the_mode_does_not_run_is_refused(self):
        """A selection the mode cannot honour fails instead of being dropped."""
        context = _FakeContext({"global_planning": "a_star_planner",
                                "algorithm": "auto"})
        with self.assertRaises(RuntimeError) as caught:
            self.launch._resolve_algorithm_selection(
                context, self.modes["display"], self.dispatch)
        self.assertIn("does not run that category", str(caught.exception))

    def test_unavailable_algorithm_is_refused_with_its_reason(self):
        """A cataloged-only algorithm cannot start a half-working graph."""
        context = _FakeContext({"localization": "hector_slam"})
        with self.assertRaises(RuntimeError) as caught:
            self.launch._resolve_algorithm_selection(
                context, self.modes["loc"], self.dispatch)
        self.assertIn("hector_slam", str(caught.exception))
        self.assertIn("not built", str(caught.exception))

    def test_auto_resolves_to_the_mode_default(self):
        """'auto' means the mode's declared default, and is reported as such."""
        selection, plugins, nodes, notes = \
            self.launch._resolve_algorithm_selection(
                _FakeContext({}), self.modes["nav"], self.dispatch)
        self.assertEqual("a_star_planner", selection["global_planning"])
        self.assertEqual("nav2_smac_planner/SmacPlanner2D",
                         plugins["global_planning"])
        self.assertTrue(notes)

    def test_none_switches_a_stage_off(self):
        """'none' leaves the stage out rather than falling back to a default."""
        selection, _, nodes, _ = self.launch._resolve_algorithm_selection(
            _FakeContext({"sensor_fusion": "none"}), self.modes["nav"],
            self.dispatch)
        self.assertEqual("", selection["sensor_fusion"])
        self.assertNotIn("sensor_fusion",
                         [category for category, _, _ in nodes])

    def test_selected_node_algorithms_become_launch_nodes(self):
        """A selection wired as a node really starts that node."""
        _, _, nodes, _ = self.launch._resolve_algorithm_selection(
            _FakeContext({"global_planning": "rrt_planner"}),
            self.modes["nav"], self.dispatch)
        actions = self.launch._algorithm_nodes(nodes, "true")
        self.assertTrue(actions, "no Node action produced for rrt_planner")


def _FakeContext(values):
    """A real LaunchContext with the algorithm arguments pre-set.

    Using the real context (rather than a stub) means the tests exercise the
    same LaunchConfiguration resolution the launch file uses at run time.
    """
    from launch import LaunchContext
    context = LaunchContext()
    module = _launch_module()
    defaults = {category: "auto" for category in module.ALGORITHM_CATEGORIES}
    defaults["algorithm"] = "auto"
    defaults.update(values)
    for name, value in defaults.items():
        context.launch_configurations[name] = value
    return context


if __name__ == "__main__":
    unittest.main()
