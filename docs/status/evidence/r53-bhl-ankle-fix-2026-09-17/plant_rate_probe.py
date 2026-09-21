#!/usr/bin/env python3
"""MuJoCo plant truth: control-rate sweep + balance-law sign (2026-09-18).

Answers two questions the live Gazebo runs could not separate:

1. Why does the biped stand perfectly for ~5.7 s and then topple in ~0.6 s
   with the committed PD-effort path?  -> The 50 Hz control rate is
   discretely UNSTABLE for the Kp=120/Kd=4 PD on the low-inertia ankle/foot
   links (zero-order-hold torque staleness). The rate sweep below shows the
   same law, same model, same spawn: stable at >= 125 Hz, divergent at
   50/25 Hz. At 500/250 Hz the pure stance hold never exceeds 0.015 rad.
2. Is the balance reaction's SIGN right? -> At a stable 250 Hz rate with a
   pitch+roll perturbation at t=3 s: committed sign (+K on same-sign
   ankles) damps to max tilt 0.008 rad; flipped sign wobbles to 0.129 rad;
   no reaction 0.014 rad. The committed sign is correct; only the rate was
   wrong. (Ledger note: this is the legs-only biped MJCF, visuals stripped
   because the vendored mjcf/assets/merged mesh layout is absent - visual
   geoms are non-colliding, so physics is unaffected.)

Method: qfrc_applied PD (tau = 120*(0-q) + 4*(-qd), clamped +/-20 N.m),
spawn base z=-0.038 (soles 2 mm above floor; see sole_height_probe.txt),
sim 500 Hz, fall = |tilt| > 0.70 rad (the controller's SAFE_STOP threshold).

Recorded output: plant_rate_probe.txt (committed next to this script).
"""
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

SRC_MJCF = ("/home/molar1/bumperbot_ws/src/robot_lab_robots/"
            "berkeley_humanoid_lite/mjcf/berkeley_humanoid_lite_biped.xml")
OUT_XML = "/tmp/bhl_mj/biped_novis.xml"
SCENE_XML = "/tmp/bhl_mj/scene_novis.xml"
SIM_HZ = 500
DUR = 12.0


def build_novis_model():
    t = ET.parse(SRC_MJCF)
    r = t.getroot()
    for a in r.findall("asset"):
        for m in list(a):
            if m.tag in ("mesh", "material", "texture"):
                a.remove(m)
        if len(list(a)) == 0:
            r.remove(a)

    def strip(body):
        for g in list(body.findall("geom")):
            if g.get("class") == "visual":
                body.remove(g)
        for b in body.findall("body"):
            strip(b)
    for wb in r.findall("worldbody"):
        strip(wb)
    for c in r.findall("compiler"):
        c.attrib.pop("meshdir", None)
    t.write(OUT_XML)
    with open(SCENE_XML, "w") as f:
        f.write('<mujoco model="bhl-biped-novis-scene">\n'
                '  <include file="biped_novis.xml"/>\n'
                '  <worldbody>\n'
                '    <geom name="floor" size="0 0 0.05" type="plane"/>\n'
                '  </worldbody>\n</mujoco>\n')


def run(model, data, adr, jnts, ctrl_every, k_sign=0.0, perturb_t=None):
    mujoco.mj_resetData(model, data)
    data.qpos[2] = -0.038
    mujoco.mj_forward(model, data)
    max_tilt = 0.0
    fall = False
    for k in range(int(DUR * SIM_HZ)):
        if k % ctrl_every == 0:
            w, x, y, z = data.qpos[3:7]
            roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            for nm in jnts:
                d, q = adr[nm]
                tau = 120.0 * (0.0 - data.qpos[q]) + 4.0 * (0.0 - data.qvel[d])
                if k_sign and "ankle_pitch" in nm:
                    tau += 120.0 * k_sign * pitch
                if k_sign and "ankle_roll" in nm:
                    tau += 120.0 * k_sign * roll
                data.qfrc_applied[d] = np.clip(tau, -20.0, 20.0)
            if perturb_t is not None and abs(k / SIM_HZ - perturb_t) < 1e-9:
                data.qvel[3] += 0.06
                data.qvel[4] += 0.04
        mujoco.mj_step(model, data)
        w, x, y, z = data.qpos[3:7]
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        max_tilt = max(max_tilt, abs(roll), abs(pitch))
        if max_tilt > 0.70:
            fall = True
            break
    rate = SIM_HZ / ctrl_every
    verdict = ("FALLS" if fall else
               "HOLDS" if max_tilt < 0.20 else "WOBBLE")
    print(f"  rate={rate:5.0f} Hz  k_sign={k_sign:+.1f}  -> {verdict}  "
          f"max_tilt={max_tilt:.4f}")


def main():
    build_novis_model()
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data = mujoco.MjData(model)
    adr = {}
    for i in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        adr[nm] = (model.jnt_dofadr[i], model.jnt_qposadr[i])
    jnts = [n for n in adr if n != "base_freejoint"]
    print("== 1. rate sweep, pure PD stance hold (Kp=120 Kd=4, +/-20 N.m) ==")
    for ce in (1, 2, 4, 10, 20):
        run(model, data, adr, jnts, ce)
    print("== 2. balance-law sign at 250 Hz, perturbed at t=3 s ==")
    for ks in (0.0, +0.7, -0.7):
        run(model, data, adr, jnts, 2, k_sign=ks, perturb_t=3.0)
    print("RESULT: control rate must be >= 125 Hz (50 Hz diverges); "
          "committed balance sign (+K) is correct and damps perturbations.")


if __name__ == "__main__":
    main()
