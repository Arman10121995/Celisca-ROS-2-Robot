"""
R3.4 GUI ↔ CLI common-resolver tests (headless).

Acceptance criteria (ROADMAP.md R3.4):
- CLI and GUI resolve identical saved profiles;
- selection changes the active component;
- stop/close only clean owned processes and never delete user artifacts;
- headless GUI-adjacent logic tests pass.

These tests exercise the headless layer (gui_composition + launch_profiles)
and the shared resolver, without a display.
"""

import sys
import tempfile
import unittest
from pathlib import Path

_SRC_GUI_DIR = Path(__file__).resolve().parent.parent
if _SRC_GUI_DIR.name == "robot_lab_gui":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_gui" in p)
    ]
    if str(_SRC_GUI_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_GUI_DIR))

_SRC_REGISTRY_DIR = _SRC_GUI_DIR.parent / "robot_lab" / "robot_lab_registry"
if _SRC_REGISTRY_DIR.name == "robot_lab_registry":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_registry" in p)
    ]
    if str(_SRC_REGISTRY_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_REGISTRY_DIR))

from robot_lab_registry.catalog import Registry  # noqa: E402
from robot_lab_adapter.resolver import (  # noqa: E402
    ExperimentRequest,
    build_ros2_launch_command,
    resolve_experiment,
)
from robot_lab_gui import gui_composition  # noqa: E402
from robot_lab_gui import launch_profiles  # noqa: E402
from robot_lab_gui.gui_composition import (  # noqa: E402
    GuiCompositionSelection,
    command_for_selection,
    environment_id_for_map_name,
    manifest_from_yaml,
    manifest_running_summary,
    manifest_to_yaml,
    migrate_legacy_selection,
    resolve_selection,
    validation_lines,
)

CONFIG_DIR = str(_SRC_REGISTRY_DIR / "config")
MODE_TO_CATEGORY = {
    "display": "perception",
    "loc": "localization",
    "slam": "localization",
    "3d_slam": "state_estimation",
    "nav": "global_planning",
}


class R34GuiCompositionTests(unittest.TestCase):
    """R3.4: headless GUI-adjacent composition logic."""

    @classmethod
    def setUpClass(cls):
        cls.registry = Registry(CONFIG_DIR)
        cls.registry.load(CONFIG_DIR)
        cls.profile_tmp = tempfile.TemporaryDirectory()
        # Isolate profile storage for the duration of the tests.
        import robot_lab_gui.launch_profiles as lp
        cls._orig_profiles_dir = lp._profiles_dir
        lp._profiles_dir = staticmethod(
            lambda: Path(cls.profile_tmp.name))
        cls.lp = lp

    @classmethod
    def tearDownClass(cls):
        cls.lp._profiles_dir = cls._orig_profiles_dir
        cls.profile_tmp.cleanup()

    def selection(self, **overrides):
        base = dict(
            robot_id="bumperbot",
            simulator="gazebo",
            environment_id="small_office",
        )
        base.update(overrides)
        return GuiCompositionSelection(**base)

    def test_cli_and_gui_resolve_identical_manifest(self):
        """The same selection resolved via GUI and CLI yields one manifest."""
        selection = self.selection(algorithm_ids={"global_planning": "navfn_planner"})
        gui_ok, gui_manifest = resolve_selection(self.registry, selection)
        self.assertTrue(gui_ok)

        cli_ok, cli_manifest = resolve_experiment(
            self.registry,
            ExperimentRequest(
                robot_id="bumperbot",
                simulator="gazebo",
                environment_id="small_office",
                algorithm_ids={"global_planning": "navfn_planner"},
            ),
        )
        self.assertTrue(cli_ok)
        for key in ("robot_id", "environment_id", "simulator", "algorithm_ids",
                    "plugins"):
            self.assertEqual(gui_manifest[key], cli_manifest[key], key)
        self.assertEqual(gui_manifest["launch"], cli_manifest["launch"])
        self.assertEqual(gui_manifest["ros2_command"], cli_manifest["ros2_command"])

    def test_saved_profile_loaded_by_cli_resolves_identically(self):
        """A GUI-saved manifest (CLI --out format) resolves the same in CLI."""
        selection = self.selection(algorithm_ids={"global_planning": "a_star_planner"})
        ok, manifest = resolve_selection(self.registry, selection)
        self.assertTrue(ok)

        # Save via the GUI profile layer (same format as CLI --out).
        path = self.lp.save_manifest("shared_profile", manifest)
        self.assertTrue(Path(path).exists())

        # Load it back and re-resolve through the CLI-facing resolver.
        reloaded = self.lp.load_profile("shared_profile")
        self.assertTrue(self.lp.is_manifest(reloaded))
        from robot_lab_adapter.resolver import manifest_to_yaml as cli_yaml
        self.assertEqual(cli_yaml(reloaded), manifest_to_yaml(manifest))

        # Re-resolving the loaded manifest's request reproduces it.
        request = ExperimentRequest(
            robot_id=reloaded["robot_id"],
            simulator=reloaded["simulator"],
            environment_id=reloaded["environment_id"],
            algorithm_ids=reloaded["algorithm_ids"],
        )
        ok2, manifest2 = resolve_experiment(self.registry, request)
        self.assertTrue(ok2)
        self.assertEqual(manifest2["ros2_command"], manifest["ros2_command"])

    def test_saved_profile_roundtrips_through_yaml(self):
        """manifest_to_yaml -> manifest_from_yaml preserves the manifest."""
        ok, manifest = resolve_selection(self.registry, self.selection())
        self.assertTrue(ok)
        reloaded = manifest_from_yaml(manifest_to_yaml(manifest))
        self.assertEqual(reloaded["robot_id"], "bumperbot")
        self.assertEqual(reloaded["launch"]["arguments"],
                         manifest["launch"]["arguments"])
        self.assertEqual(reloaded["ros2_command"], manifest["ros2_command"])

    # ------------------------------------------------------------------
    # Selection changes the active component
    # ------------------------------------------------------------------

    def test_selection_changes_active_planner_component(self):
        """Changing the global_planning slot changes the active nav2 plugin."""
        a_star = self.selection(algorithm_ids={"global_planning": "a_star_planner"})
        navfn = self.selection(algorithm_ids={"global_planning": "navfn_planner"})
        cmd_a = command_for_selection(self.registry, a_star)
        cmd_n = command_for_selection(self.registry, navfn)
        self.assertIn("global_planner_plugin:=nav2_smac_planner/SmacPlanner2D",
                      cmd_a)
        self.assertIn("global_planner_plugin:=nav2_navfn_planner/NavfnPlanner",
                      cmd_n)
        self.assertNotEqual(cmd_a, cmd_n)
        # All other arguments stay the same.
        arg_a = {p.split(":=", 1)[0]: p for p in cmd_a[4:]}
        arg_n = {p.split(":=", 1)[0]: p for p in cmd_n[4:]}
        arg_a.pop("global_planner_plugin")
        arg_n.pop("global_planner_plugin")
        self.assertEqual(arg_a, arg_n)

    # ------------------------------------------------------------------
    # Validator filtering
    # ------------------------------------------------------------------

    def test_validation_filters_invalid_selection(self):
        """Validator-driven reasons for unsupported components are surfaced."""
        go2_no_lidar = self.selection(
            robot_id="go2", environment_id="empty",
            algorithm_ids={"localization": "amcl"})
        errors, _warnings = validation_lines(self.registry, go2_no_lidar)
        self.assertTrue(errors)
        self.assertTrue(any("lidar" in e for e in errors))

        valid = self.selection(
            algorithm_ids={"localization": "amcl",
                           "global_planning": "navfn_planner"})
        errors2, _warnings2 = validation_lines(self.registry, valid)
        self.assertEqual(errors2, [])
        # Warnings may exist (topic-name variants) but the selection is valid.

    def test_environment_id_for_map_name(self):
        """Legacy map names map to registry environments (exact then key)."""
        self.assertEqual(
            environment_id_for_map_name("small_office", self.registry),
            "small_office")
        # small_office's world file key is 'simple_office'.
        self.assertEqual(
            environment_id_for_map_name("simple_office", self.registry),
            "small_office")
        # celisca_floor_1 is now a real environment id in the catalog.
        self.assertEqual(
            environment_id_for_map_name("celisca_floor_1", self.registry),
            "celisca_floor_1")

    def test_legacy_profile_migration(self):
        """Old (mode/robot/map_name/algorithm) profiles migrate to selection."""
        legacy = {
            "mode": "loc",
            "simulator": "gazebo",
            "robot": "bumperbot",
            "map_name": "small_office",
            "algorithm": "amcl",
            "gui": False,
        }
        migrated = migrate_legacy_selection(
            legacy, self.registry, MODE_TO_CATEGORY)
        self.assertEqual(migrated.robot_id, "bumperbot")
        self.assertEqual(migrated.environment_id, "small_office")
        self.assertEqual(migrated.algorithm_ids, {"localization": "amcl"})

        legacy_nav = {
            "mode": "nav",
            "robot": "quadrotor_sitl",
            "map_name": "aerial_course",
            "algorithm": "navfn_planner",
        }
        migrated_nav = migrate_legacy_selection(
            legacy_nav, self.registry, MODE_TO_CATEGORY)
        self.assertEqual(migrated_nav.algorithm_ids,
                         {"global_planning": "navfn_planner"})

    def test_manifest_running_summary(self):
        """The running summary exposes what the manifest will actually run."""
        ok, manifest = resolve_selection(
            self.registry,
            self.selection(algorithm_ids={"global_planning": "navfn_planner"}))
        self.assertTrue(ok)
        summary = manifest_running_summary(manifest)
        self.assertEqual(summary["robot_id"], "bumperbot")
        self.assertEqual(summary["environment_id"], "small_office")
        self.assertEqual(summary["plugins"]["global_planning"],
                         "nav2_navfn_planner/NavfnPlanner")
        self.assertEqual(summary["launch_file"], "simulated_robot.launch.py")

    # ------------------------------------------------------------------
    # Ownership: delete/close only owned things, keep user artifacts
    # ------------------------------------------------------------------

    def test_profile_delete_never_touches_user_artifacts(self):
        """Deleting a profile removes only that profile file."""
        profiles_dir = Path(self.lp._profiles_dir())
        artifact = profiles_dir / "user_log.txt"
        artifact.write_text("precious user data")
        self.lp.save_profile("temp_profile",
                             {"mode": "nav", "robot": "bumperbot"})
        self.assertTrue(self.lp.delete_profile("temp_profile"))
        self.assertFalse((profiles_dir / "temp_profile.json").exists())
        # The unrelated user artifact is untouched.
        self.assertTrue(artifact.exists())
        self.assertEqual(artifact.read_text(), "precious user data")
        artifact.unlink()

        # Deleting a non-existent profile reports False.
        self.assertFalse(self.lp.delete_profile("does_not_exist"))

    def test_ensure_defaults_never_overwrites_user_profile(self):
        """Built-in defaults cannot clobber an existing profile."""
        self.lp.save_profile("Navigation (Isaac)", {"mode": "nav",
                                                    "robot": "custom_bot"})
        before = self.lp.load_profile("Navigation (Isaac)")
        self.lp.ensure_defaults()
        after = self.lp.load_profile("Navigation (Isaac)")
        self.assertEqual(before, after)
        self.assertEqual(after.get("robot"), "custom_bot")

    def test_cli_out_manifest_loadable_by_gui(self):
        """A CLI-written --out manifest loads and resolves in the GUI layer."""
        import io
        from contextlib import redirect_stdout

        import robot_lab_registry.cli as cli

        with tempfile.TemporaryDirectory() as tmp:
            out_file = Path(tmp) / "cli_manifest.yaml"
            captured = io.StringIO()
            with redirect_stdout(captured):
                rc = cli.main(["launch", "-c", CONFIG_DIR,
                               "--robot", "bumperbot",
                               "--world", "small_office",
                               "--global-planning", "navfn_planner",
                               "--out", str(out_file)])
            self.assertEqual(rc, 0)
            loaded = manifest_from_yaml(out_file.read_text())
            self.assertIn("manifest_version", loaded)

            # The GUI layer re-resolves the same profile to the same command.
            selection = GuiCompositionSelection(
                robot_id=loaded["robot_id"],
                simulator=loaded["simulator"],
                environment_id=loaded["environment_id"],
                algorithm_ids=loaded["algorithm_ids"],
            )
            ok, gui_manifest = resolve_selection(self.registry, selection)
            self.assertTrue(ok, gui_manifest)
            self.assertEqual(gui_manifest["ros2_command"],
                             loaded["ros2_command"])


if __name__ == "__main__":
    unittest.main()