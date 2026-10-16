#!/usr/bin/env python3
"""Generate the camera calibration room and its chessboard model (R7.2).

The Gazebo calibration check moves one board with Gazebo services; MuJoCo,
PyBullet and Isaac have no such services.  This world instead stands
chessboards around the nav_empty spawn, at seeded distances, heights and
tilts, and the robot turns on the spot in front of them
(scripts/r7_2_calibration_room_check.py).  Every simulator draws the board
from the same textured Collada visual (Gazebo natively, the others through
robot_lab_utils.textured_mesh).

Writes:
* robot_lab_models/models/robot_lab_calibration_board/: model.config,
  model.sdf, meshes/board.dae, materials/textures/board.png - 8 x 6 inner
  corners, 0.04 m squares, a one-square white margin;
* robot_lab_maps/maps/calibration_room/worlds/calibration_room.world;
* robot_lab_maps/mjcf/calibration_room.xml (gen_mjcf_worlds.convert_world).

  python3 gen_calibration_room.py [--seed 7]
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAPS = os.path.dirname(_HERE)
_SRC = os.path.dirname(_MAPS)
PATTERN = (8, 6)          # inner corners
SQUARE = 0.04             # metres
PIXELS = 40               # texture pixels per square
SPAWN = (-0.5, -0.5)      # nav_empty spawn (sim_maps.yaml)
CAMERA_HEIGHT = 0.20


def board_png(path):
    """Square texture: the board's (columns + 1) x (rows + 1) squares centred
    in white, as scripts/r7_2_calibration_check.py draws it."""
    from PIL import Image
    columns, rows = PATTERN
    side = max(columns, rows) + 3
    image = np.full((side * PIXELS, side * PIXELS), 255, np.uint8)
    left, top = (side - columns - 1) // 2, (side - rows - 1) // 2
    for j in range(rows + 1):
        for i in range(columns + 1):
            if (i + j) % 2 == 0:
                image[(top + j) * PIXELS:(top + j + 1) * PIXELS, (left + i) * PIXELS:(left + i + 1) * PIXELS] = 0
    Image.fromarray(image).save(path)
    return side * SQUARE


def board_dae(size):
    """A square facing +x in the y-z plane, texture u along -y, v along +z."""
    h = size / 2.0
    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="board-png"><init_from>../materials/textures/board.png</init_from></image></library_images>
  <library_effects><effect id="board-fx"><profile_COMMON>
    <newparam sid="surface"><surface type="2D"><init_from>board-png</init_from></surface></newparam>
    <newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
    <technique sid="common"><lambert>
      <emission><color>0.25 0.25 0.25 1</color></emission>
      <diffuse><texture texture="sampler" texcoord="UVSET0"/></diffuse>
    </lambert></technique>
  </profile_COMMON></effect></library_effects>
  <library_materials><material id="board"><instance_effect url="#board-fx"/></material></library_materials>
  <library_geometries><geometry id="board-mesh"><mesh>
    <source id="board-positions"><float_array id="board-positions-array" count="12">0 {h} -{h} 0 -{h} -{h} 0 -{h} {h} 0 {h} {h}</float_array>
      <technique_common><accessor source="#board-positions-array" count="4" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="board-uv"><float_array id="board-uv-array" count="8">0 0 1 0 1 1 0 1</float_array>
      <technique_common><accessor source="#board-uv-array" count="4" stride="2">
        <param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="board-vertices"><input semantic="POSITION" source="#board-positions"/></vertices>
    <triangles material="board-material" count="2">
      <input semantic="VERTEX" source="#board-vertices" offset="0"/>
      <input semantic="TEXCOORD" source="#board-uv" offset="1" set="0"/>
      <p>0 0 1 1 2 2 0 0 2 2 3 3</p>
    </triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="board-node">
    <instance_geometry url="#board-mesh"><bind_material><technique_common>
      <instance_material symbol="board-material" target="#board"/>
    </technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""


def board_model(size):
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="robot_lab_calibration_board">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>-0.003 0 0 0 0 0</pose>
        <geometry><box><size>0.005 {size:.4f} {size:.4f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://robot_lab_calibration_board/meshes/board.dae</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>robot_lab_calibration_board</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Chessboard with 8 x 6 inner corners and 0.04 m squares, facing +x
  (generated by robot_lab_maps/tools/gen_calibration_room.py).</description>
</model>
"""


def board_poses(rng, size, count=14):
    """Boards on a ring around the spawn, each facing the camera position
    with a seeded tilt; centres high and low so they cover the image rows."""
    poses = []
    for index in range(count):
        azimuth = 2 * math.pi * index / count + rng.uniform(-0.1, 0.1)
        distance = rng.uniform(0.55, 0.95)
        # Alternate low, middle and high boards; the lowest edge stays above the floor.
        height = [CAMERA_HEIGHT, CAMERA_HEIGHT + 0.22, size / 2 + 0.04][index % 3] + rng.uniform(-0.03, 0.03)
        height = max(height, size / 2 + 0.03)
        x = SPAWN[0] + distance * math.cos(azimuth)
        y = SPAWN[1] + distance * math.sin(azimuth)
        facing = azimuth + math.pi                       # +x toward the spawn
        dz = CAMERA_HEIGHT - height
        pitch = -math.atan2(dz, distance)                # face the camera height
        poses.append((x, y, height, rng.uniform(-0.3, 0.3),
                      pitch + rng.uniform(-0.35, 0.35),
                      facing + rng.uniform(-0.45, 0.45)))
    return poses


def world(poses):
    boards = "\n".join(f"""    <include>
      <name>calibration_board_{i}</name>
      <uri>model://robot_lab_calibration_board</uri>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
    </include>""" for i, (x, y, z, roll, pitch, yaw) in enumerate(poses))
    return f"""<?xml version="1.0" ?>
<!-- Generated by robot_lab_maps/tools/gen_calibration_room.py - do not edit by hand.
     Chessboards around the nav_empty spawn for camera calibration (R7.2):
     launch with map_name:=nav_empty world_name:=calibration_room
     world_path:=<this file>. -->
<sdf version="1.7">
  <world name="calibration_room">
    <physics type="ode"><max_step_size>0.002</max_step_size></physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.7 0.7 1</background></scene>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
{boards}
  </world>
</sdf>
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)
    model = os.path.join(_SRC, "robot_lab_models", "models", "robot_lab_calibration_board")
    os.makedirs(os.path.join(model, "meshes"), exist_ok=True)
    os.makedirs(os.path.join(model, "materials", "textures"), exist_ok=True)
    size = board_png(os.path.join(model, "materials", "textures", "board.png"))
    with open(os.path.join(model, "meshes", "board.dae"), "w") as handle:
        handle.write(board_dae(size))
    with open(os.path.join(model, "model.sdf"), "w") as handle:
        handle.write(board_model(size))
    with open(os.path.join(model, "model.config"), "w") as handle:
        handle.write(MODEL_CONFIG)
    world_dir = os.path.join(_MAPS, "maps", "calibration_room", "worlds")
    os.makedirs(world_dir, exist_ok=True)
    world_path = os.path.join(world_dir, "calibration_room.world")
    with open(world_path, "w") as handle:
        handle.write(world(board_poses(np.random.default_rng(args.seed), size)))
    sys.path.insert(0, _HERE)
    import gen_mjcf_worlds
    mjcf, skipped = gen_mjcf_worlds.convert_world(world_path, "calibration_room")
    with open(os.path.join(_MAPS, "mjcf", "calibration_room.xml"), "w") as handle:
        handle.write(mjcf)
    print("board %.2f m, %d boards, mjcf %d geoms%s" % (
        size, mjcf.count("calibration_board"), mjcf.count("<geom"),
        (" skipped: %s" % skipped) if skipped else ""))


if __name__ == "__main__":
    main()
