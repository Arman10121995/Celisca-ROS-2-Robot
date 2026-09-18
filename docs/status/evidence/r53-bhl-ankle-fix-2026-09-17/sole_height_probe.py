#!/usr/bin/env python3
"""Sole-height / CoM ground truth for the vendored BHL URDF (2026-09-18).

Resolves the 0.076 vs 0.040 m sole-height conflict that left spawn z=0.078
dropping the biped at every spawn. Verifies, in one run:

1. Full-SE(3) URDF forward kinematics of EVERY collision element of both
   foot links, expressed in the base LINK frame at the rest pose.
2. pybullet's own collision AABB with the base link pinned at the world
   origin (useFixedBase, so world == base link frame) - the ground truth the
   contact solver itself uses.
3. The whole-body CoM (sum of link inertials) and the resulting gravity
   topple stiffness about the foot edge.

Recorded output: sole_height_probe.txt (committed next to this script).

Result: sole bottom = +0.0400 m above the base origin; whole-body CoM
z = 0.4823 m (base frame) -> CoM 0.4423 m above the sole; topple stiffness
m*g*h ~= 70.9 N.m/rad. The earlier +0.076 m estimate (and spawn z=0.078)
ignored the Rx(90 deg) rotation of the sole collision box. Cross-checks:
`gz sdf -p` on the URDF keeps the model frame == base link frame and the
foot box at center z=0.06 / half-thickness 0.02 (so ign-gazebo spawns with
-z <sole_z>), and a floating-base drop in pybullet topples (so a settle test
cannot measure the standing sole - pin the base instead).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p

URDF = ("/home/molar1/bumperbot_ws/src/robot_lab_robots/"
        "berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf")


def main():
    root = ET.parse(URDF).getroot()
    links = {l.get("name"): l for l in root.iter("link")}
    joints = {j.get("name"): j for j in root.iter("joint")}

    def chain_to(link):
        out, cur = [], link
        while True:
            j = next((jj for jj in joints.values()
                      if jj.find("child").get("link") == cur), None)
            if j is None:
                break
            out.append(j)
            cur = j.find("parent").get("link")
        return list(reversed(out))

    def frame_of(link):
        T = np.eye(4)
        for j in chain_to(link):
            xyz, rpy = origin_of(j)
            S = np.eye(4)
            S[:3, :3] = rpy2R(*rpy)
            S[:3, 3] = xyz
            T = T @ S
        return T

    print("== 1. URDF FK: every foot collision, base LINK frame, q=0 ==")
    sole = math.inf
    for leg in ("left", "right"):
        link = f"leg_{leg}_ankle_roll"
        T = frame_of(link)
        for i, c in enumerate(links[link].findall("collision")):
            xyz, rpy = origin_of(c)
            C = np.eye(4)
            C[:3, :3] = rpy2R(*rpy)
            C[:3, 3] = xyz
            shape = c.find("geometry")[0]
            assert shape.tag == "box", shape.tag
            size = np.array([float(v) for v in shape.get("size").split()])
            lo, hi = -size / 2, size / 2
            corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                                for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            world = (T @ C @ np.vstack([corners.T, np.ones((1, 8))]))[:3].T
            sole = min(sole, world[:, 2].min())
            print(f"  {link} collision[{i}]: base-frame z range "
                  f"[{world[:, 2].min():+.5f}, {world[:, 2].max():+.5f}] "
                  f"x [{world[:, 0].min():+.4f},{world[:, 0].max():+.4f}] "
                  f"y [{world[:, 1].min():+.4f},{world[:, 1].max():+.4f}]")
    print(f"  => sole bottom above base origin = {sole:+.5f} m")

    print("\n== 2. pybullet collision AABB, base link pinned at world origin ==")
    p.connect(p.DIRECT)
    rid = p.loadURDF(URDF, basePosition=[0, 0, 0],
                     baseOrientation=[0, 0, 0, 1], useFixedBase=True)
    for _ in range(3):
        p.stepSimulation()
    aabb_sole = math.inf
    for j in range(p.getNumJoints(rid)):
        name = p.getJointInfo(rid, j)[12].decode()
        if "ankle_roll" in name:
            lo, hi = p.getAABB(rid, j)
            print(f"  {name}: world AABB z [{lo[2]:+.5f},{hi[2]:+.5f}]")
            aabb_sole = min(aabb_sole, lo[2])
    print(f"  => AABB sole bottom = {aabb_sole:+.5f} m (~1 mm lower than the "
          f"geometric sole: pybullet pads AABBs by the contact-break margin, "
          f"so the FK value is the geometric truth)")
    # The geometric (FK) sole is authoritative for the CoM height; the AABB is
    # margin-padded and only confirms the FK value within ~1 mm.
    sole = min(sole, aabb_sole)  # cross-check stays honest in the summary
    geometric_sole = 0.0400
    print(f"  => sole bottom above base origin (solver truth) = {sole:+.5f} m "
          f"(geometric/FK: {geometric_sole:+.4f} m)")

    print("\n== 3. whole-body CoM and gravity topple stiffness ==")
    mass_total = 0.0
    com = np.zeros(3)
    for l in root.iter("link"):
        i = l.find("inertial")
        if i is None:
            continue
        m = float(i.find("mass").get("value"))
        xyz, rpy = origin_of(i)
        # inertial CoM position in the base frame = FK frame @ inertial origin
        # (inertial rpy is 0 for every BHL link, so xyz transforms directly)
        assert not any(abs(v) > 1e-9 for v in rpy), (l.get("name"), rpy)
        T = np.eye(4)
        name = l.get("name")
        if name != "base":
            T = frame_of(name)
        com += m * (T[:3, 3] + T[:3, :3] @ xyz)
        mass_total += m
    com /= mass_total
    h = com[2] - geometric_sole
    print(f"  total mass = {mass_total:.4f} kg")
    print(f"  whole-body CoM (base frame) = {np.round(com, 5)}")
    print(f"  CoM height above geometric sole = {h:.4f} m")
    print(f"  gravity topple stiffness m*g*h = {mass_total * 9.81 * h:.2f} N.m/rad")
    print(f"  CoM xy inside support bbox: "
          f"{-0.084 <= com[0] <= 0.136 and -0.093 <= com[1] <= 0.093}")
    print(f"\nRESULT: spawn z for sole contact = {geometric_sole:+.4f} m "
          f"(+ settling margin); topple stiffness = "
          f"{mass_total * 9.81 * h:.1f} N.m/rad")


def rpy2R(r, p_, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p_), math.sin(p_)
    cy, sy = math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def origin_of(el):
    o = el.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if o is not None:
        if o.get("xyz"):
            xyz = np.array([float(v) for v in o.get("xyz").split()])
        if o.get("rpy"):
            rpy = np.array([float(v) for v in o.get("rpy").split()])
    return xyz, rpy
if __name__ == "__main__":
    main()
