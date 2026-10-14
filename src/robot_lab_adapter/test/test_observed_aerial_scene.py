"""Observed geometry counterexamples; these checks do not launch a drone."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_utils'))
from robot_lab_utils.observed_aerial_scene import VoxelObstacles, ObservedScene, depth_points, transform_points  # noqa: E402
from robot_lab_utils.aerial_planning import plan  # noqa: E402


def test_voxel_inflation_does_not_lose_a_point_at_the_far_cell_corner():
    source = [.14999, .14999, .14999]
    observation = VoxelObstacles([source])
    assert observation.clearance(source) <= 0.
    assert observation.clearance([.6, .15, .15]) <= np.linalg.norm(np.array(source)-[.6,.15,.15])


def test_thin_observed_obstacle_between_sampling_points_blocks_a_segment():
    obstacle = VoxelObstacles([[.13, 0, 2.]])
    assert not obstacle.segment_clear([0,0,2], [.25,0,2], radius=.02)
    assert obstacle.segment_clear([0,1,2], [1,1,2], radius=.2)


def test_oversized_occupancy_rejects_instead_of_dropping_obstacles():
    with pytest.raises(ValueError, match='refusing to drop'):
        VoxelObstacles([[0,0,0],[1,1,1]], max_cells=1)


def test_capture_transform_rotates_optical_points_and_preserves_translation():
    points = transform_points([[1,0,2]], [3,4,5], [0,0,np.sqrt(.5),np.sqrt(.5)])
    np.testing.assert_allclose(points, [[3,5,7]], atol=1e-12)
    with pytest.raises(ValueError, match='transform'):
        transform_points([[0,0,0]], [0,0,0], [0,0,0,0])


def test_every_valid_depth_pixel_is_projected_with_intrinsics_and_skew():
    depth = np.full((8,8),2.)
    k = [10,1,3.5,0,20,3.5,0,0,1]
    points, counts = depth_points(depth,k)
    assert len(points)==64 and counts['valid_fraction']==1.
    np.testing.assert_allclose(points[0], [-.665,-.35,2.])
    with pytest.raises(ValueError, match='unseen space'):
        depth_points(np.full((8,8),np.nan),k)


def test_native_camera_near_clip_and_observation_range_are_explicit():
    k = [10,0,3.5,0,10,3.5,0,0,1]
    _, counts = depth_points(np.full((8,8),.2),k)
    assert counts['valid_pixels']==64 and counts['min_range_m']==.2
    for distance in (.199,8.01):
        with pytest.raises(ValueError, match='unseen space'):
            depth_points(np.full((8,8),distance),k)


def test_observed_scene_causes_real_astar_detour_around_points():
    static = SimpleNamespace(clearance=lambda _position: 20.)
    obstacles = VoxelObstacles([[1,0,2], [1,.15,2], [1,-.15,2]])
    scene = ObservedScene(static, obstacles)
    route, details = plan(scene, [0,0,2], [2,0,2], radius=.2, resolution=.2,
                          timeout=2., min_altitude=1., max_altitude=3.)
    assert details['expanded']>0 and len(route)>2
    assert all(obstacles.segment_clear(a,b,radius=.2) for a,b in zip(route[:-1],route[1:]))


def test_runtime_guard_interrupts_a_commanded_route_before_setpoint_advance():
    pytest.importorskip('pymavlink', reason='PX4 SDK: run this check in the existing SSD PX4 venv')
    from robot_lab_adapter.px4_ros_controller import PX4Controller
    controller = SimpleNamespace(route=[[1.,0.,2.]], target=[0.,0.,2.],
        command_generation=0, route_status='tracking', position=lambda:[0.,0.,2.],
        route_guard=lambda _position,_route:'Observed obstacle', route_best_distance=10., route_progress=0.)
    controller.clear_path = lambda reason: PX4Controller.clear_path(controller,reason)
    PX4Controller.advance_route(controller,1.,.05)
    assert not controller.route and controller.command_generation==1
    assert controller.target==[0.,0.,2.] and controller.route_status=='Observed obstacle'
