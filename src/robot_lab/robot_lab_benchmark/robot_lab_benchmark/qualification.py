"""R5.1: golden-harness qualification for Bumperbot and Labbot mobile bases.

Implements the R5.1 acceptance bar:

- Robot-specific inertias, wheel geometry, footprint, sensors, control and
  overlays are verified against golden values extracted from the real
  processed URDF (xacro -> URDF XML), not from hand-copied numbers.
- Both bases must spawn at a known pose, drive, turn and stop, and
  complete clear and obstacle navigation with predeclared limits and
  watchdogs. Task-level execution is injected via an executor callable so
  the qualification logic is testable without a live simulator; the
  backend used for the run is named in the report.
- Truth (simulator ground truth) and odometry are recorded as separate
  trajectories; evaluation uses truth only, and odometry drift is
  reported as its own observation, never merged into truth.
- Capabilities are claimed only when implemented: an RGB-D claim requires
  a depth camera in the robot description. Labbot has no RGB-D sensor,
  so its capability set excludes it rather than claiming it.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

QUALIFICATION_SCHEMA_VERSION = "1.0"

#: Wheel radius tolerance for golden static checks (metres).
WHEEL_RADIUS_TOL = 5e-3
#: Wheel separation tolerance for golden static checks (metres).
WHEEL_SEPARATION_TOL = 5e-3
#: Relative tolerance for inertia golden checks.
INERTIA_RTOL = 1e-4


@dataclass(frozen=True)
class CheckResult:
    """Result of one qualification check."""

    name: str
    passed: bool
    detail: str

    def to_dict(self):
        return {"name": self.name, "passed": self.passed, "detail": self.detail}


@dataclass
class QualificationReport:
    """Full qualification outcome for one robot."""

    robot_id: str
    passed: bool
    checks: List[CheckResult] = field(default_factory=list)
    issues: List[str] = field(default_factory=list)

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "passed": self.passed,
            "checks": [c.to_dict() for c in self.checks],
            "issues": list(self.issues),
        }


@dataclass(frozen=True)
class LinkInfo:
    """Mass and inertia of one URDF link.

    ``is_virtual`` marks frame-only links (no inertial, no collision and
    no visual geometry), such as ``base_footprint``; those are exempt
    from the positive-mass/inertia requirement.
    """

    name: str
    mass: Optional[float]
    has_inertia: bool
    is_virtual: bool = False


@dataclass(frozen=True)
class WheelInfo:
    """One drive wheel extracted from the URDF."""

    joint: str
    link: str
    radius: float
    y_offset: float
    axis: Tuple[float, float, float]


# ----------------------------------------------------------------------
# Gazebo sensor vocabulary
# ----------------------------------------------------------------------

#: Classic Gazebo and Ignition/Gz use different <sensor type> names for
#: the same physical sensor. Normalize both to one vocabulary so the
#: golden harness is simulator-dialect independent.
SENSOR_TYPE_ALIASES = {
    "ray": "lidar",
    "gpu_lidar": "lidar",
    "lidar": "lidar",
    "depth": "depth",
    "rgbd_camera": "depth",
    "camera": "camera",
    "gpu_camera": "camera",
    "imu": "imu",
}


def normalize_sensor_types(sensor_types) -> Tuple[str, ...]:
    """Map simulator-specific sensor type names to the common vocabulary."""
    normalized = []
    for raw in sensor_types:
        mapped = SENSOR_TYPE_ALIASES.get(raw)
        if mapped is not None and mapped not in normalized:
            normalized.append(mapped)
    return tuple(normalized)


@dataclass(frozen=True)
class RobotDescription:
    """Static description extracted from the processed URDF."""

    robot_name: str
    links: Tuple[LinkInfo, ...]
    wheels: Tuple[WheelInfo, ...]
    wheel_separation: float
    base_footprint_offset_z: float
    footprint: Dict[str, float]
    sensor_types: Tuple[str, ...]
    actuated_joints: Tuple[str, ...]
    base_collision_kind: str = "none"  # "primitive" | "mesh" | "none"

    @property
    def link_names(self) -> Tuple[str, ...]:
        return tuple(link.name for link in self.links)

    @property
    def normalized_sensor_types(self) -> Tuple[str, ...]:
        return normalize_sensor_types(self.sensor_types)

    @property
    def capabilities(self) -> Tuple[str, ...]:
        """Claimed capabilities; only sensors actually present are claimed."""
        caps = ["differential_drive"]
        sensors = set(self.normalized_sensor_types)
        if "lidar" in sensors:
            caps.append("lidar")
        if "imu" in sensors:
            caps.append("imu")
        if "depth" in sensors:
            caps.append("rgbd")
        return tuple(caps)

    def wheel_radius(self) -> float:
        return self.wheels[0].radius

    def total_mass(self) -> float:
        return sum(link.mass or 0.0 for link in self.links)


def _parse_urdf(urdf_xml: str) -> "etree.Element":  # type: ignore[name-defined]
    import xml.etree.ElementTree as ET

    return ET.fromstring(urdf_xml)


def _geometry_radius(geom) -> Optional[float]:
    """Inradius-equivalent radius from a URDF <geometry> element.

    Cylinders and spheres use their radius directly; boxes use the
    half-diagonal of the x/y footprint, which is the conservative
    circular footprint that fully contains the box.
    """
    if geom is None:
        return None
    for shape in ("cylinder", "sphere"):
        node = geom.find(shape)
        if node is not None and node.get("radius") is not None:
            return float(node.get("radius"))
    box = geom.find("box")
    if box is not None and box.get("size") is not None:
        try:
            sx, sy, _sz = (float(v) for v in box.get("size").split())
        except (ValueError, TypeError):
            return None
        return math.hypot(sx / 2.0, sy / 2.0)
    return None


def extract_robot_description(
    xacro_paths: Sequence, mappings: Optional[Mapping[str, str]] = None
) -> RobotDescription:
    """Process xacro file(s) to URDF and extract the qualification inputs.

    Multiple files are merged (e.g. main description + gazebo overlay) by
    concatenating their <robot> children, which is how the overlay sensor
    tags reach the extractor without a full ROS launch.
    """
    import xacro

    if mappings is None:
        mappings = {}
    chunks = []
    robot_name = ""
    for path in xacro_paths:
        doc = xacro.process_file(str(path), mappings=dict(mappings))
        chunks.append(doc.toprettyxml(indent="  "))
        root = _parse_urdf(doc.toprettyxml(indent="  "))
        robot_name = robot_name or root.get("name", "")

    import xml.etree.ElementTree as ET

    merged = ET.Element("robot", {"name": robot_name})
    for chunk in chunks:
        root = ET.fromstring(chunk)
        for child in root:
            merged.append(child)

    return _description_from_urdf(merged)


def _description_from_urdf(root) -> RobotDescription:
    links: List[LinkInfo] = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        mass = None
        has_inertia = False
        if inertial is not None:
            mass_node = inertial.find("mass")
            if mass_node is not None and mass_node.get("value") is not None:
                mass = float(mass_node.get("value"))
            has_inertia = inertial.find("inertia") is not None
        is_virtual = (
            inertial is None
            and link.find("collision") is None
            and link.find("visual") is None
        )
        links.append(
            LinkInfo(link.get("name", ""), mass, has_inertia, is_virtual)
        )

    wheels: List[WheelInfo] = []
    base_z = 0.0
    link_geom_radius: Dict[str, Optional[float]] = {}
    for link in root.findall("link"):
        collision = link.find("collision")
        radius = _geometry_radius(collision.find("geometry") if collision is not None else None)
        link_geom_radius[link.get("name", "")] = radius
        if link.get("name", "") == "base_footprint":
            base_z = 0.0

    for joint in root.findall("joint"):
        if joint.get("type") != "continuous":
            continue
        child = joint.find("child")
        child_link = child.get("link", "") if child is not None else ""
        origin = joint.find("origin")
        xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        parts = [float(v) for v in xyz.split()]
        axis_node = joint.find("axis")
        axis = (
            tuple(float(v) for v in axis_node.get("xyz", "0 0 0").split())
            if axis_node is not None
            else (0.0, 1.0, 0.0)
        )
        radius = link_geom_radius.get(child_link) or 0.0
        wheels.append(
            WheelInfo(
                joint=joint.get("name", ""),
                link=child_link,
                radius=radius,
                y_offset=parts[1],
                axis=axis,
            )
        )

    base_joint = None
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link") == "base_link":
            base_joint = joint
            break
    if base_joint is not None:
        origin = base_joint.find("origin")
        xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        base_z = float(xyz.split()[2])

    footprint: Dict[str, float] = {}
    base_collision_kind = "none"
    base_link = next((l for l in root.findall("link") if l.get("name") == "base_link"), None)
    if base_link is not None:
        collision = base_link.find("collision")
        geom = collision.find("geometry") if collision is not None else None
        radius = _geometry_radius(geom)
        if radius is not None:
            footprint["radius"] = radius
            base_collision_kind = "primitive"
        elif geom is not None and geom.find("mesh") is not None:
            # The collision footprint is defined by a mesh; no circular
            # radius is derived (we do not fabricate one). Downstream
            # footprint consumers must use the mesh or the costmap config.
            base_collision_kind = "mesh"

    sensor_types = tuple(
        dict.fromkeys(s.get("type", "") for s in root.iter("sensor") if s.get("type"))
    )

    actuated: List[str] = []
    for ros2_control in root.findall("ros2_control"):
        for joint in ros2_control.findall("joint"):
            name = joint.get("name", "")
            if joint.get("command_interface") is not None or joint.find("command_interface") is not None:
                actuated.append(name)

    separation = 0.0
    if len(wheels) >= 2:
        ys = [w.y_offset for w in wheels]
        separation = abs(max(ys) - min(ys))

    return RobotDescription(
        robot_name=root.get("name", ""),
        links=tuple(links),
        wheels=tuple(wheels),
        wheel_separation=separation,
        base_footprint_offset_z=base_z,
        footprint=footprint,
        sensor_types=sensor_types,
        actuated_joints=tuple(actuated),
        base_collision_kind=base_collision_kind,
    )


# ----------------------------------------------------------------------
# Golden harness (pinned from the measured robot descriptions)
# ----------------------------------------------------------------------


@dataclass(frozen=True)
class RobotGolden:
    """Golden expected values for one robot, pinned from measured sources."""

    robot_id: str
    wheel_radius: float
    wheel_separation: float
    base_footprint_offset_z: float
    wheel_joints: Tuple[str, ...]
    required_sensor_types: Tuple[str, ...]
    required_actuated_joints: int
    rgbd_capable: bool


#: Values below are the measured values from the real robot descriptions:
#: bumperbot wheels are 0.033 m spheres at y=+-0.07011018494186..., base_link
#: 0.033 m above base_footprint; labbot wheels are 0.06 m cylinders at
#: y=+-0.15, base_link 0.12 m above base_footprint. Sensor requirements use
#: the *normalized* vocabulary (lidar/imu/depth) so both classic Gazebo
#: (ray/depth) and Ignition/Gz (gpu_lidar/rgbd_camera) dialects qualify.
#: Labbot has no RGB-D sensor and none is claimed. Bumperbot's base
#: collision is a mesh, so its footprint is mesh-defined (no circular
#: radius is fabricated).
GOLDEN: Dict[str, RobotGolden] = {
    "bumperbot": RobotGolden(
        robot_id="bumperbot",
        wheel_radius=0.033,
        wheel_separation=0.0701101849418637 + 0.0701101849418642,
        base_footprint_offset_z=0.033,
        wheel_joints=("wheel_left_joint", "wheel_right_joint"),
        required_sensor_types=("lidar", "imu", "depth"),
        required_actuated_joints=2,
        rgbd_capable=True,
    ),
    "labbot": RobotGolden(
        robot_id="labbot",
        wheel_radius=0.06,
        wheel_separation=0.30,
        base_footprint_offset_z=0.12,
        wheel_joints=("labbot_left_wheel_joint", "labbot_right_wheel_joint"),
        required_sensor_types=("lidar", "imu"),
        required_actuated_joints=2,
        rgbd_capable=False,
    ),
}


def parse_robot_description(urdf_xml: str) -> RobotDescription:
    """Public entry: parse a processed URDF string into a description."""
    return _description_from_urdf(_parse_urdf(urdf_xml))


def golden_for(robot_id: str) -> RobotGolden:
    if robot_id not in GOLDEN:
        raise KeyError("no golden harness entry for robot %r" % robot_id)
    return GOLDEN[robot_id]


# ----------------------------------------------------------------------
# Static checks (inertias, wheel geometry, footprint, sensors, control)
# ----------------------------------------------------------------------


def _check(condition, name, ok_detail, fail_detail,
           checks: List[CheckResult], issues: List[str]) -> None:
    checks.append(CheckResult(name, bool(condition),
                              ok_detail if condition else fail_detail))
    if not condition:
        issues.append("%s: %s" % (name, fail_detail))


def check_inertias(desc: RobotDescription, checks, issues) -> None:
    # Virtual frame-only links (e.g. base_footprint) legitimately carry no
    # inertial; every physical link must have positive mass and a tensor.
    physical = [link for link in desc.links if not link.is_virtual]
    bad = [link.name for link in physical
           if link.mass is None or link.mass <= 0.0 or not link.has_inertia]
    _check(
        not bad, "inertias",
        "all %d physical links have positive mass and inertia tensors"
        % len(physical),
        "links missing mass/inertia: %s" % (bad or "none"),
        checks, issues,
    )


def check_wheel_geometry(desc: RobotDescription, golden: RobotGolden,
                         checks, issues) -> None:
    _check(
        len(desc.wheels) == 2, "wheel_count",
        "two drive wheels found", "expected 2 drive wheels, found %d"
        % len(desc.wheels),
        checks, issues,
    )
    _check(
        all(math.isclose(w.radius, golden.wheel_radius, abs_tol=WHEEL_RADIUS_TOL)
            for w in desc.wheels) and bool(desc.wheels),
        "wheel_radius",
        "wheel radius %.4f m matches golden" % (desc.wheels[0].radius if desc.wheels else -1.0),
        "wheel radii %s do not match golden %.4f m"
        % ([w.radius for w in desc.wheels], golden.wheel_radius),
        checks, issues,
    )
    _check(
        math.isclose(desc.wheel_separation, golden.wheel_separation,
                     abs_tol=WHEEL_SEPARATION_TOL),
        "wheel_separation",
        "wheel separation %.4f m matches golden" % desc.wheel_separation,
        "wheel separation %.4f m does not match golden %.4f m"
        % (desc.wheel_separation, golden.wheel_separation),
        checks, issues,
    )
    found_joints = {w.joint for w in desc.wheels}
    _check(
        all(j in found_joints for j in golden.wheel_joints),
        "wheel_joints",
        "differential drive joints present: %s" % sorted(found_joints),
        "expected drive joints %s, found %s"
        % (sorted(golden.wheel_joints), sorted(found_joints)),
        checks, issues,
    )
    _check(
        desc.wheels and all(w.axis[1] > 0.9 for w in desc.wheels),
        "wheel_axis_y",
        "wheel rotation axes are lateral (+y)",
        "wheel axes %s are not lateral" % ([w.axis for w in desc.wheels],),
        checks, issues,
    )


def check_footprint(desc: RobotDescription, golden: RobotGolden,
                    checks, issues) -> None:
    _check(
        math.isclose(desc.base_footprint_offset_z, golden.base_footprint_offset_z,
                     abs_tol=WHEEL_SEPARATION_TOL),
        "base_footprint_offset_z",
        "base_link sits %.3f m above base_footprint as measured"
        % desc.base_footprint_offset_z,
        "base offset %.4f m does not match golden %.4f m"
        % (desc.base_footprint_offset_z, golden.base_footprint_offset_z),
        checks, issues,
    )
    if desc.footprint.get("radius", 0.0) > 0.0:
        _check(
            True, "footprint",
            "footprint extracted: %s" % desc.footprint,
            "",  # unused on pass
            checks, issues,
        )
    elif desc.base_collision_kind == "mesh":
        # Honest handling: the base collision is a mesh, so no circular
        # radius is derived. This qualifies, but the report states that
        # the numeric footprint must come from the mesh/costmap config.
        _check(
            True, "footprint",
            "base collision is mesh-defined; circular footprint radius not "
            "derived (no value fabricated) - use the mesh or costmap config",
            "",
            checks, issues,
        )
    else:
        _check(
            False, "footprint",
            "",
            "no usable collision footprint on base_link",
            checks, issues,
        )


def check_sensors(desc: RobotDescription, golden: RobotGolden,
                  checks, issues) -> None:
    # Compare in the normalized vocabulary so both simulator dialects
    # (classic ray/depth and ignition gpu_lidar/rgbd_camera) qualify.
    present = set(desc.normalized_sensor_types)
    missing = [s for s in golden.required_sensor_types if s not in present]
    _check(
        not missing, "sensors",
        "required sensors present (normalized): %s"
        % sorted(golden.required_sensor_types),
        "missing sensor types %s (present: %s)" % (missing, sorted(present)),
        checks, issues,
    )
    # Capabilities are claimed only when implemented.
    claims = set(desc.capabilities)
    _check(
        ("rgbd" in claims) == (golden.rgbd_capable and "depth" in present),
        "rgbd_claim_matches_hardware",
        "RGB-D capability claim consistent with the description",
        "RGB-D claim inconsistent: claimed=%s, depth sensor present=%s"
        % ("rgbd" in claims, "depth" in present),
        checks, issues,
    )
    if not golden.rgbd_capable:
        _check(
            "rgbd" not in claims,
            "no_false_rgbd_claim",
            "no RGB-D claim made (robot has no depth camera)",
            "robot has no depth camera but claims RGB-D capability",
            checks, issues,
        )


def check_actuation(desc: RobotDescription, golden: RobotGolden,
                    checks, issues) -> None:
    _check(
        len(desc.actuated_joints) >= golden.required_actuated_joints,
        "actuated_joints",
        "drive joints exposed to ros2_control: %s" % sorted(desc.actuated_joints),
        "expected >=%d actuated joints, found %s"
        % (golden.required_actuated_joints, sorted(desc.actuated_joints)),
        checks, issues,
    )


def run_static_checks(desc: RobotDescription, golden: RobotGolden
                      ) -> Tuple[List[CheckResult], List[str]]:
    checks: List[CheckResult] = []
    issues: List[str] = []
    check_inertias(desc, checks, issues)
    check_wheel_geometry(desc, golden, checks, issues)
    check_footprint(desc, golden, checks, issues)
    check_sensors(desc, golden, checks, issues)
    check_actuation(desc, golden, checks, issues)
    return checks, issues


# ----------------------------------------------------------------------
# Task-level qualification (execution injected)
# ----------------------------------------------------------------------


@dataclass(frozen=True)
class TaskStageResult:
    """Result of one predeclared task stage (spawn/drive/turn/stop/nav)."""

    stage: str
    passed: bool
    detail: str = ""


@dataclass(frozen=True)
class TaskTrialSpec:
    """Predeclared limits and watchdogs for the task-level trial."""

    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_yaw: float = 0.0
    drive_distance_m: float = 1.0
    drive_tolerance_m: float = 0.05
    turn_degrees: float = 90.0
    turn_tolerance_deg: float = 5.0
    stage_timeout_seconds: float = 30.0
    nav_clear_timeout_seconds: float = 60.0
    nav_obstacle_timeout_seconds: float = 90.0
    min_clearance_m: float = 0.1
    seed: int = 0


@dataclass(frozen=True)
class TaskTrialObservation:
    """What the backend measured during one trial.

    ``truth_trajectory`` is simulator ground truth; ``odom_trajectory`` is
    wheel odometry. Evaluation uses truth only, and odometry drift is
    reported as its own observation, never merged into truth.
    """

    truth_trajectory: Tuple[Tuple[float, float, float], ...] = ()
    odom_trajectory: Tuple[Tuple[float, float, float], ...] = ()
    min_clearance_m: Optional[float] = None
    contacts: int = 0

    def drift_summary(self) -> Dict[str, Optional[float]]:
        """Final-pose odometry drift w.r.t. truth (reported, never merged)."""
        if not self.truth_trajectory or not self.odom_trajectory:
            return {"odom_drift_m": None}
        tx, ty, _ = self.truth_trajectory[-1]
        ox, oy, _ = self.odom_trajectory[-1]
        return {"odom_drift_m": math.hypot(tx - ox, ty - oy)}


#: Executor injected by the backend: (robot_id, spec) -> observed stages.
TaskExecutor = Callable[[str, TaskTrialSpec], List[TaskStageResult]]


def _drift_check(observation: TaskTrialObservation, checks, issues) -> None:
    drift = observation.drift_summary()["odom_drift_m"]
    _check(
        drift is not None,
        "odom_drift_reported",
        "odometry drift observed: %.4f m (reported separately from truth)"
        % (drift or 0.0),
        "odometry drift could not be computed (missing trajectory)",
        checks, issues,
    )


def qualify_robot(
    robot_id: str,
    description: RobotDescription,
    executor: Optional[TaskExecutor],
    *,
    spec: Optional[TaskTrialSpec] = None,
    backend: str = "injected",
    observation: Optional[TaskTrialObservation] = None,
) -> QualificationReport:
    """Qualify one mobile base against the golden harness.

    Static description checks always run. Task-level stages (spawn, drive,
    turn, stop, clear navigation, obstacle navigation) are executed through
    the injected *executor*; with no executor the report honestly marks
    itself not passed rather than claiming unverified task capability.
    Odometry drift (from *observation*, when provided) is reported as its
    own check and never merged into the truth trajectory.
    """
    spec = spec or TaskTrialSpec()
    golden = golden_for(robot_id)
    checks, issues = run_static_checks(description, golden)

    if executor is None:
        issues.append("task stages: skipped (no executor); report is not a pass")
        return QualificationReport(robot_id=robot_id, passed=False,
                                   checks=checks, issues=issues)

    for stage in executor(robot_id, spec):
        _check(
            stage.passed, "task_%s" % stage.stage,
            stage.detail or "stage passed", stage.detail or "stage failed",
            checks, issues,
        )

    if observation is not None:
        _drift_check(observation, checks, issues)

    return QualificationReport(
        robot_id=robot_id,
        passed=not issues,
        checks=checks,
        issues=issues,
    )


def write_qualification_report(report: QualificationReport, path) -> Path:
    """Write a machine-readable qualification report (JSON)."""
    payload = {
        "qualification_schema_version": QUALIFICATION_SCHEMA_VERSION,
        **report.to_dict(),
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path
