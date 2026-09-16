"""Offline Isaac Sim wheel-drive diagnostic (runs under Isaac's python.sh).

Usage: python.sh isaac_drive_diag.py <urdf> <collision_from_visuals:0|1>
       [wheel_armature] [merge_fixed_joints:0|1]
"""
import sys

URDF, CFV = sys.argv[1], sys.argv[2] == "1"
ARMATURE = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
MERGE = len(sys.argv) > 4 and sys.argv[4] == "1"
DAMPING = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0e4
PHYSICS_HZ = float(sys.argv[6]) if len(sys.argv) > 6 else 60.0
print("VARIANT cfv=%s armature=%s merge_fixed_joints=%s damping=%s hz=%s"
      % (CFV, ARMATURE, MERGE, DAMPING, PHYSICS_HZ), flush=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

sys.path.insert(0, "/workspace/molar/ros_ws/bumperbot_ws/src/robot_lab_isaac/python/robot_lab_isaac")
import isaac_runtime as rt  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

stage_utils.create_new_stage()
imp = URDFImporterConfig(urdf_path=URDF)
imp.merge_fixed_joints = MERGE
imp.fix_base = False
imp.collision_from_visuals = CFV
imp.joint_drive_type = "force"
usd = URDFImporter(imp).import_urdf()
stage_utils.add_reference_to_stage(usd, "/World")
stage = stage_utils.get_current_stage()
world = World(physics_dt=1.0 / PHYSICS_HZ, stage_units_in_meters=1.0)
world.scene.add_ground_plane()
driven = rt._author_wheel_velocity_drives(stage, ("wheel_left_joint", "wheel_right_joint"), DAMPING)
print("DRIVEN", driven)
if ARMATURE > 0.0:
    from pxr import PhysxSchema
    for path in driven:
        api = PhysxSchema.PhysxJointAPI.Apply(stage.GetPrimAtPath(path))
        (api.GetArmatureAttr() or api.CreateArmatureAttr()).Set(ARMATURE)
    print("ARMATURE set", ARMATURE)

for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    if prim.IsA(UsdPhysics.Joint) and "wheel" in prim.GetName():
        attrs = {a.GetName(): a.Get() for a in prim.GetAttributes()
                 if any(k in a.GetName() for k in ("drive", "physxJoint", "Enabled", "axis", "body"))}
        print("JOINT", prim.GetPath(), prim.GetTypeName(), attrs,
              [str(t) for t in prim.GetRelationship("physics:body0").GetTargets()],
              [str(t) for t in prim.GetRelationship("physics:body1").GetTargets()])
    if prim.HasAPI(UsdPhysics.CollisionAPI) and "caster" in str(prim.GetPath()) or (
            prim.HasAPI(UsdPhysics.CollisionAPI) and "wheel" in str(prim.GetPath())):
        approx = prim.GetAttribute("physics:approximation")
        print("COLLIDER", prim.GetPath(), prim.GetTypeName(), approx.Get() if approx else None)
    if prim.HasAPI(UsdPhysics.MassAPI):
        mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if mass:
            print("MASS", prim.GetPath(), mass)

root = None
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World")):
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        root = prim
        break
print("ROOT", root.GetPath() if root else None)
robot = Robot(prim_path=str(root.GetPath()), name="bumperbot",
              position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0))
world.scene.add(robot)
world.reset()
robot.initialize()
names = list(robot.dof_names)
print("DOFS", names, "ROOT BODY", rt._articulation_root_body(robot))
view = robot._articulation_view
for fn in ("get_gains", "get_max_efforts", "get_joint_max_velocities",
           "get_friction_coefficients", "get_armatures", "get_body_masses"):
    try:
        print(fn, getattr(view, fn)())
    except Exception as exc:
        print(fn, "ERR", exc)

li, ri = names.index("wheel_left_joint"), names.index("wheel_right_joint")
for label, (vl, vr) in (("settle", (0.0, 0.0)), ("straight", (9.09, 9.09)), ("turn", (3.0, 6.09))):
    steps = int(3 * PHYSICS_HZ)
    import math as _m
    def _pose():
        p, q = robot.get_world_pose()
        w, x, y, z = (float(v) for v in q)
        return float(p[0]), float(p[1]), _m.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    start = None
    for step in range(steps):
        if step == int(PHYSICS_HZ):  # average over the last 2 s of the phase
            start = _pose()
        robot.apply_action(ArticulationAction(joint_velocities=[vl, vr], joint_indices=[li, ri]))
        world.step(render=False)
        if step % int(PHYSICS_HZ) == int(PHYSICS_HZ) - 1:
            jv = robot.get_joint_velocities()
            lin = robot.get_linear_velocity()
            ang = robot.get_angular_velocity()
            pos, _ = robot.get_world_pose()
            print(label, step, "jvel L/R %.2f %.2f" % (float(jv[li]), float(jv[ri])),
                  "speed %.3f" % float((lin[0] ** 2 + lin[1] ** 2) ** 0.5),
                  "wz %.3f" % float(ang[2]), "z %.4f" % float(pos[2]), flush=True)
    end = _pose()
    if start is not None:
        print("AVG %s speed %.3f m/s yaw_rate %.3f rad/s" % (label, _m.hypot(end[0] - start[0], end[1] - start[1]) / 2.0,
              _m.remainder(end[2] - start[2], 2 * _m.pi) / 2.0), flush=True)
    try:
        print("velocity targets", view._physics_view.get_dof_velocity_targets())
    except Exception as exc:
        print("velocity targets ERR", exc)
print("expected: straight 0.300 m/s wz 0; turn 0.150 m/s wz 0.600 (0.17 m contact track)")
print("DIAG_DONE", flush=True)
app.close()
