"""
Simulator availability + compatibility gating tests (headless).

Covers the R3.4+ GUI gating layer:
- host-level simulator detection (installed binaries / configured runtimes);
- per-simulator mode gating from the documented sensor feature gaps;
- selection cascade corrections;
- the resolver applying the selected bringup mode and GUI choice to the
  concrete launch command (previously silently defaulted to mode=nav).
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

_SRC_ADAPTER_PKG = _SRC_GUI_DIR.parent / "robot_lab_adapter"
if (_SRC_ADAPTER_PKG / "robot_lab_adapter").is_dir():
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_adapter" in p)
    ]
    if str(_SRC_ADAPTER_PKG) not in sys.path:
        sys.path.insert(0, str(_SRC_ADAPTER_PKG))

from robot_lab_registry.catalog import Registry  # noqa: E402
from robot_lab_adapter.resolver import (  # noqa: E402
    ExperimentRequest,
    resolve_experiment,
)
from robot_lab_gui import simulator_compat as sc  # noqa: E402
from robot_lab_gui.gui_composition import (  # noqa: E402
    GuiCompositionSelection,
    build_request,
)

CONFIG_DIR = str(_SRC_REGISTRY_DIR / "config")

MODE_PROFILES = {
    "display": {"simulators": ["gazebo", "isaac", "pybullet", "mujoco"]},
    "loc": {"simulators": ["gazebo", "isaac", "pybullet", "mujoco"]},
    "slam": {"simulators": ["gazebo", "isaac", "pybullet", "mujoco"]},
    "3d_slam": {"simulators": ["gazebo", "isaac", "pybullet", "mujoco"],
                "required_features": ["rgbd_camera"]},
    "nav": {"simulators": ["gazebo", "isaac", "pybullet", "mujoco"]},
}


class SimulatorAvailabilityTests(unittest.TestCase):
    """Host-level detection of simulator backends."""

    def test_isaac_unavailable_without_runtime(self):
        """No ISAAC_PYTHON and no isaacsim wheel -> Isaac not selectable."""
        with tempfile.TemporaryDirectory() as tmp:
            missing = str(Path(tmp) / "no-such-python")
            installed, reason = sc.simulator_status(
                "isaac", env={"ISAAC_PYTHON": missing})
        self.assertFalse(installed)
        self.assertIn("ISAAC_PYTHON", reason)

    def test_isaac_available_with_runtime_env(self):
        """A configured ISAAC_PYTHON interpreter makes Isaac selectable."""
        with tempfile.NamedTemporaryFile(suffix="python3.12") as fake:
            installed, reason = sc.simulator_status(
                "isaac", env={"ISAAC_PYTHON": fake.name})
        self.assertTrue(installed)
        self.assertEqual(reason, "")

    def test_unknown_simulator_reports_reason(self):
        installed, reason = sc.simulator_status("warp10", env={})
        self.assertFalse(installed)
        self.assertIn("Unknown simulator", reason)

    def test_available_simulators_covers_all_backends(self):
        statuses = sc.available_simulators(env={})
        self.assertEqual(sorted(statuses), sorted(sc.SIMULATOR_ORDER))
        for installed, reason in statuses.values():
            self.assertIsInstance(installed, bool)
            self.assertIsInstance(reason, str)


class SimulatorModeGatingTests(unittest.TestCase):
    """Feature-gap gating: which modes a backend can physically run."""

    def test_gazebo_supports_every_mode(self):
        for mode in MODE_PROFILES:
            ok, why = sc.simulator_supports_mode("gazebo", mode, MODE_PROFILES)
            self.assertTrue(ok, (mode, why))

    def test_pybullet_mujoco_cannot_run_3d_slam(self):
        for sim in ("pybullet", "mujoco"):
            ok, why = sc.simulator_supports_mode(sim, "3d_slam", MODE_PROFILES)
            self.assertFalse(ok)
            self.assertIn("rgbd_camera", why)
            for mode in ("display", "loc", "slam", "nav"):
                ok, why = sc.simulator_supports_mode(sim, mode, MODE_PROFILES)
                self.assertTrue(ok, (sim, mode, why))

    def test_isaac_lacks_scan_and_rgbd(self):
        self.assertFalse(
            sc.simulator_supports_mode("isaac", "nav", MODE_PROFILES)[0])
        self.assertFalse(
            sc.simulator_supports_mode("isaac", "3d_slam", MODE_PROFILES)[0])
        self.assertTrue(
            sc.simulator_supports_mode("isaac", "display", MODE_PROFILES)[0])

    def test_allowed_simulators_for_3d_slam(self):
        allowed = sc.allowed_simulators("3d_slam", MODE_PROFILES, env={})
        self.assertFalse(allowed["pybullet"][0])
        self.assertFalse(allowed["mujoco"][0])
        self.assertFalse(allowed["isaac"][0])
        if allowed["gazebo"][0]:
            self.assertEqual(allowed["gazebo"][1], "")

    def test_allowed_modes_preserves_order(self):
        modes = sc.allowed_modes("mujoco", list(MODE_PROFILES), MODE_PROFILES)
        self.assertNotIn("3d_slam", modes)
        self.assertEqual(modes, [m for m in MODE_PROFILES if m != "3d_slam"])


class CorrectionTests(unittest.TestCase):
    """Selection cascade corrections."""

    def test_keeps_valid_current(self):
        value, note = sc.correction_for("mujoco", ["gazebo", "mujoco"])
        self.assertEqual(value, "mujoco")
        self.assertEqual(note, "")

    def test_falls_back_by_preference(self):
        value, note = sc.correction_for("isaac", ["gazebo"], ["gazebo"])
        self.assertEqual(value, "gazebo")
        self.assertIn("isaac", note)

    def test_no_allowed_option(self):
        value, note = sc.correction_for("isaac", [])
        self.assertIsNone(value)
        self.assertIn("no compatible", note)


class ResolverAppliesSelectionTests(unittest.TestCase):
    """The resolved launch command must honour the GUI's selections."""

    @classmethod
    def setUpClass(cls):
        cls.registry = Registry(CONFIG_DIR)
        cls.registry.load(CONFIG_DIR)

    def resolve(self, **overrides):
        request = ExperimentRequest(
            robot_id="bumperbot",
            simulator="gazebo",
            environment_id="small_office",
        )
        for key, value in overrides.items():
            setattr(request, key, value)
        return resolve_experiment(self.registry, request)

    def test_mode_and_gui_applied_to_command(self):
        """Selected mode/gui become concrete launch arguments."""
        ok, manifest = self.resolve(mode="slam", gui="false")
        self.assertTrue(ok, manifest)
        args = manifest["launch"]["arguments"]
        self.assertEqual(args["mode"], "slam")
        self.assertEqual(args["gui"], "false")
        self.assertIn("mode:=slam", manifest["ros2_command"])
        self.assertIn("gui:=false", manifest["ros2_command"])

    def test_default_selection_omits_optional_args(self):
        """Without mode/gui the manifest stays backward compatible."""
        ok, manifest = self.resolve()
        self.assertTrue(ok, manifest)
        self.assertNotIn("mode", manifest["launch"]["arguments"])
        self.assertNotIn("gui", manifest["launch"]["arguments"])

    def test_invalid_gui_value_rejected(self):
        ok, outcome = self.resolve(gui="always")
        self.assertFalse(ok)
        self.assertTrue(any("gui" in error for error in outcome["errors"]))

    def test_gui_selection_round_trips_through_selection_dataclass(self):
        selection = GuiCompositionSelection(
            robot_id="bumperbot",
            simulator="gazebo",
            environment_id="small_office",
            mode="3d_slam",
            gui="true",
        )
        request = build_request(selection)
        self.assertEqual(request.mode, "3d_slam")
        self.assertEqual(request.gui, "true")
        ok, manifest = resolve_experiment(self.registry, request)
        self.assertTrue(ok, manifest)
        self.assertEqual(manifest["launch"]["arguments"]["mode"], "3d_slam")
        self.assertEqual(manifest["launch"]["arguments"]["gui"], "true")
        self.assertEqual(manifest["resolved_from"]["mode"], "3d_slam")


if __name__ == "__main__":
    unittest.main()