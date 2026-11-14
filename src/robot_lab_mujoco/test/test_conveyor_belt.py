"""MuJoCo deck belt: a resting payload follows the roller surface speed (R11)."""
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robot_lab_mujoco.conveyor_belt import ConveyorBelt  # noqa: E402

XML = """<mujoco><option timestep="0.002"/><worldbody>
  <body name="robot"><geom type="box" size=".05 .05 .05" pos="0 0 .05"/>
    <body name="deck" pos="0 0 .5"><geom name="plate" type="box" size=".6 .2 .01"/>
      <body name="roller" pos="-.55 0 -.02"><joint name="roller_joint" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size=".012 .2" euler="90 0 0" contype="0" conaffinity="0"/></body></body></body>
  <body name="payload" pos="-.3 0 .56"><freejoint/><geom type="box" size=".1 .075 .05" mass=".5" friction=".8"/></body>
</worldbody></mujoco>"""


def run(roller_speed, seconds=2.0):
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    belt = ConveyorBelt(model, data, 'deck', 'roller_joint', roller_radius=.01, mu=.8)
    payload = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'payload')
    for _ in range(int(.5 / model.opt.timestep)):          # settle on the deck
        mujoco.mj_step(model, data)
    start = data.xpos[payload].copy()
    for _ in range(int(seconds / model.opt.timestep)):
        data.qvel[model.jnt_dofadr[0]] = roller_speed
        belt.step()
        mujoco.mj_step(model, data)
    return data.xpos[payload] - start, data.cvel[payload][3:].copy(), belt


def test_payload_moves_at_belt_speed_and_stays_on_the_deck():
    moved, velocity, belt = run(10.0)                      # 0.10 m/s belt
    assert moved[0] == pytest.approx(.2, abs=.03)
    assert abs(moved[1]) < .01 and abs(moved[2]) < .01
    assert belt.speed() == pytest.approx(.1)


def test_stopped_or_reversed_belt():
    moved, _, _ = run(0.0)
    assert np.abs(moved).max() < .005
    moved, _, _ = run(-10.0, 1.0)
    assert moved[0] == pytest.approx(-.1, abs=.02)


def test_forces_are_replaced_not_accumulated():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    belt = ConveyorBelt(model, data, 'deck', 'roller_joint', roller_radius=.01)
    for _ in range(300):
        mujoco.mj_step(model, data)
    data.qvel[model.jnt_dofadr[0]] = 10.
    belt.step()
    once = data.xfrc_applied.copy()
    belt.step()
    np.testing.assert_allclose(data.xfrc_applied, once)    # same state, same force: nothing piles up
