# R7.2 camera_calibration: a chessboard in front of the Gazebo camera (2026-10-15)

Status: **implemented, experimental**. The perception id
`camera_calibration` named ROS's interactive cameracalibrator.py, which is
not installed here, so it was cataloged only.

`robot_lab_algorithms.camera_calibration` is the same method, Zhang's
planar-target calibration, with OpenCV (`a691fae`, `eae5809`, `f550534`):

1. find the board's inner corners in a frame: the classic detector refined
   to sub-pixel, then OpenCV's sector-based detector if that misses;
2. keep the view if it differs from every kept view in board position,
   size or tilt;
3. once `min_views` (15) are kept, solve the camera matrix with plumb_bob
   distortion, or none with `model:=pinhole`.

`perception:=camera_calibration` starts the node through the dispatch. It
publishes the result, latched, on `/perception/calibrated_camera_info` and
can write a ROS camera-info YAML (`output_path`).

**Unit check.** On 27 ray-cast board views from a known camera, it
recovers fx within 0.15% and the principal point within 0.15 px:
(159.48, 119.36) for (159.5, 119.5).

## The Gazebo check

`scripts/r7_2_calibration_check.sh bumperbot nav_empty <out.json>` launches
Bumperbot in Gazebo with `perception:=camera_calibration`. Then
`scripts/r7_2_calibration_check.py`:

1. draws a chessboard with 8 x 6 inner corners into a PNG;
2. spawns it as a floating, self-lit plane;
3. moves it with `set_pose` to 100 seeded poses: 0.45-0.85 m from the
   OAK-D, centred on a random pixel of the middle 60% of the image, tilted
   up to 30°, and entirely above the floor;
4. calibrates from the frames rendered after each move, with both models;
5. reads the node's own result.

Final run ([v4.json](v4.json), `f550534`, 17:2x):

| | fx | fy | cx | cy | RMS |
|---|---|---|---|---|---|
| Camera info (gz-sensors) | 221.77 | 221.77 | 160.00 | 120.00 | |
| Pinhole at the pixel centres | 221.77 | 221.77 | **159.50** | **119.50** | |
| Script, 73 views, plumb_bob | 220.16 | 220.13 | 159.37 | 119.53 | 0.43 px |
| Script, 73 views, pinhole | 220.57 | 220.58 | 159.36 | 118.97 | 0.43 px |
| Node, 15 views, plumb_bob ([log](v4_node_log.txt)) | 218.62 | 218.35 | 159.37 | 119.34 | 0.34 px |

- 73 of 100 board views were kept; 13 frames had no board found, and 14
  were too similar to kept views.
- **The principal point agrees with the depth check.** The 73-view
  plumb_bob fit lands 0.13 px and 0.03 px from the pixel centres
  (159.5, 119.5), and 0.6 px and 0.5 px from the camera info's
  (160, 120). The depth images showed the same independently
  ([depth-node](../depth-node/README.md)).
- **The focal length comes out 0.7% short** (1.4% for the node's 15
  views). The depth check rules that out for the camera: with the camera
  info's fy, the Gazebo floor lies at 0.0 mm at every depth, which a 0.7%
  fy error would raise by about 1.4 mm. So the shortfall is the
  calibration's. The likely cause is the aliased, unfiltered 320 x 240
  rendering (RMS 0.43 px, against 0.09 px on the anti-aliased unit-check
  renders); it was not isolated.

## Earlier runs, kept as recorded

Each run exposed a fault in the check.

- [v1](v1_stretched_board.json): 20 views, RMS 0.55 px, cy 116.1.
  Gazebo mapped the 11 x 9-square texture onto the plane transposed. The
  board appeared in portrait with stretched, non-square squares, which no
  planar calibration can fit.
- [v2](v2_stretched_small_board.json): a smaller board found in 5 of 100
  frames, for the same reason, and because FAST_CHECK rejected the small
  boards.
- [v3](v3_square_board_into_floor.json): a square texture on a square
  plane, but boards centred low reached into the floor and lost their
  lower rows ([frame](v3_board_into_floor.png)). The camera is only about
  0.2 m up, so the poses are now resampled (145 of them in v4).

## Limits

- Gazebo only: the check spawns its board through Gazebo services. The
  other simulators have no chessboard model yet.
- One seed and one camera (Bumperbot's OAK-D, 320 x 240).
