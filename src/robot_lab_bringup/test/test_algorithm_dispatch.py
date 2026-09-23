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
import pytest

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


def test_resolver_package_relative_assets_resolve_at_launch(monkeypatch, tmp_path):
    launch = _launch_module()
    share = tmp_path / 'installed maps'
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda package: str(share))
    for relative in ('maps/nav_obstacle/worlds/nav_obstacle.world', 'maps/nav_obstacle/maps/map.yaml'):
        assert launch._resolve_asset_override(relative, 'robot_lab_maps') == str(share / relative)
    local = tmp_path / 'custom.world'
    local.write_text('<sdf/>')
    assert launch._resolve_asset_override(str(local), 'robot_lab_maps') == str(local)
    assert launch._resolve_asset_override('', 'robot_lab_maps') == ''


def _scene_actions(**selection):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    module = _launch_module()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(gui='false', start_rviz='false', **selection)
    return context, module._build_simulation_actions(context)


@pytest.mark.parametrize('backend', ['gazebo', 'mujoco', 'isaac', 'pybullet'])
@pytest.mark.parametrize('robot', ['bumperbot', 'labbot'])
@pytest.mark.parametrize('arena', ['celisca_floor_1', 'celisca_floor_2',
    'celisca_floor_1_furniture', 'celisca_floor_2_furniture', 'celisca_f1_actor', 'celisca_f2_actor'])
def test_selected_celisca_world_and_localization_share_spawn(backend, robot, arena):
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler
    _, actions = _scene_actions(simulator=backend, robot_model=robot,
        map_name=arena, mode='nav', spawn_x='5', spawn_y='2', spawn_yaw='0.7')
    # Non-Gazebo stacks start after the readiness gate; inspect those
    # conditional launch actions as well as the immediate simulator include.
    nested = [child for action in actions if isinstance(action, RegisterEventHandler)
              for _, children in action.describe_conditional_sub_entities() for child in children]
    includes = [dict(a.launch_arguments) for a in actions + nested if isinstance(a, IncludeLaunchDescription)]
    world = next(a for a in includes if 'world_path' in a)
    localization = next(a for a in includes if 'initial_pose_x' in a)
    assert world['world_path'].endswith('/maps/' + arena + '/worlds/' + arena + '.world')
    assert localization['map_yaml'].endswith('/maps/' + arena + '/maps/map.yaml')
    for axis, expected in [('x', '5.0'), ('y', '2.0'), ('yaw', '0.7')]:
        assert world['spawn_' + axis] == localization['initial_pose_' + axis] == expected


def test_gazebo_launches_isolate_transport_and_preserve_explicit_partition(monkeypatch):
    from launch.actions import SetEnvironmentVariable
    monkeypatch.delenv('IGN_PARTITION', raising=False)
    monkeypatch.delenv('GZ_PARTITION', raising=False)
    def partition():
        from unittest.mock import patch
        # LaunchContext uses os.environ. Each real ros2 launch has a fresh
        # process; restore the environment to emulate that here.
        with patch.dict(os.environ):
            context, actions = _scene_actions(simulator='gazebo', robot_model='bumperbot',
                map_name='celisca_floor_1', mode='display')
            for action in actions:
                if isinstance(action, SetEnvironmentVariable): action.execute(context)
            assert context.environment['GZ_PARTITION'] == context.environment['IGN_PARTITION']
            return context.environment['IGN_PARTITION']
    assert partition() != partition()
    monkeypatch.setenv('IGN_PARTITION', 'user_selected_partition')
    assert partition() == 'user_selected_partition'
