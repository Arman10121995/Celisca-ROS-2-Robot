"""PyBullet deck belt (pybullet_belt): a loose box rides the belt; a stopped belt holds it."""
import pytest

p = pytest.importorskip('pybullet')
from robot_lab_pybullet.pybullet_belt import PyBulletBelt  # noqa: E402

URDF = '''<robot name="deck"><link name="base"/>
<link name="telescope_stage2"><inertial><mass value="5"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
  <collision><geometry><box size="1.0 0.4 0.05"/></geometry></collision></link>
<link name="roller"><inertial><mass value="0.1"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial></link>
<joint name="deck_fix" type="fixed"><parent link="base"/><child link="telescope_stage2"/><origin xyz="0 0 0.3"/></joint>
<joint name="transfer_roller_joint" type="continuous"><parent link="telescope_stage2"/><child link="roller"/>
  <axis xyz="0 1 0"/><limit effort="5" velocity="30"/></joint></robot>'''


def _world(tmp_path):
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    path = tmp_path / 'deck.urdf'
    path.write_text(URDF)
    robot = p.loadURDF(str(path), useFixedBase=True)
    box = p.createMultiBody(0.5, p.createCollisionShape(p.GEOM_BOX, halfExtents=[.05, .05, .05]),
                            basePosition=[0, 0, .3 + .025 + .05])
    links = {p.getJointInfo(robot, i)[12].decode(): i for i in range(p.getNumJoints(robot))}
    joints = {p.getJointInfo(robot, i)[1].decode(): i for i in range(p.getNumJoints(robot))}
    belt = PyBulletBelt(robot, links['telescope_stage2'], joints['transfer_roller_joint'], .012, {'box': box})
    return client, robot, box, belt, joints['transfer_roller_joint']


def _run(robot, roller, belt, rate, steps):
    for _ in range(steps):
        p.setJointMotorControl2(robot, roller, p.VELOCITY_CONTROL, targetVelocity=rate, force=5)
        belt.step()
        p.stepSimulation()


def test_box_rides_the_running_belt(tmp_path):
    client, robot, box, belt, roller = _world(tmp_path)
    try:
        _run(robot, roller, belt, 0., 120)               # settle on the deck
        start = p.getBasePositionAndOrientation(box)[0]
        _run(robot, roller, belt, 10., 240)              # 0.12 m/s belt for 1 s
        end = p.getBasePositionAndOrientation(box)[0]
        assert end[0] - start[0] == pytest.approx(.12, abs=.03)
        assert abs(end[1] - start[1]) < .01
    finally:
        p.disconnect(client)


def test_stopped_belt_holds_the_box(tmp_path):
    client, robot, box, belt, roller = _world(tmp_path)
    try:
        _run(robot, roller, belt, 0., 360)
        position = p.getBasePositionAndOrientation(box)[0]
        assert abs(position[0]) < .005 and abs(position[1]) < .005
    finally:
        p.disconnect(client)
