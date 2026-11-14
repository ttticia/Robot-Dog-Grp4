"""Tests for maze_explorer.coverage_utils on synthetic occupancy grids."""

import math

import numpy as np
import pytest

from maze_explorer.coverage_utils import (angle_diff, apply_fence, CoverageMap,
                                          entrance_segments, GridInfo, line_of_sight,
                                          parse_edges, parse_polygon, plan_view_goal,
                                          reachable_area, SpinPlanner, start_to_world,
                                          travel_distance, wall_faces, yaw_from_quaternion)

RES = 0.05


def corridor_map():
    """
    3 m x 1 m corridor along +x (walls all round), 0.05 m cells, origin (0, 0).

    Returns (grid, info); free inside x 0.1..2.9, y 0.1..0.9.
    """
    h, w = 20, 60
    grid = np.full((h, w), 100, np.int8)
    grid[2:18, 2:58] = 0
    return grid, GridInfo(RES, 0.0, 0.0, w, h)


def fresh(info):
    """Return a CoverageMap sized for ``info``."""
    cov = CoverageMap()
    cov.update_grid(info)
    return cov


def test_wall_faces_are_free_cells_touching_walls():
    grid, _ = corridor_map()
    free = grid == 0
    faces = wall_faces(free, grid == 100)
    assert faces[2, 10] and faces[17, 10] and faces[10, 2]
    assert not faces[10, 10]          # middle of the corridor
    assert not faces[0, 10]           # inside the wall itself


def test_mark_view_sees_the_end_wall_in_front_only():
    grid, info = corridor_map()
    cov = fresh(info)
    # Camera 1.6 m from the +x end wall, looking at it.
    n = cov.mark_view(grid, 1.3, 0.5, 0.0, math.radians(40), 2.5)
    assert n > 0
    assert cov.seen[10, 57]           # face in front of the end wall at x = 2.9
    assert not cov.seen[10, 2]        # the wall behind the camera


def test_mark_view_respects_max_range():
    grid, info = corridor_map()
    cov = fresh(info)
    cov.mark_view(grid, 0.3, 0.5, 0.0, math.radians(20), 1.5)   # end wall is 2.6 m away
    assert not cov.seen[8:12, 55:58].any()


def test_mark_view_ignores_grazing_views():
    grid, info = corridor_map()
    cov = fresh(info)
    # Looking straight down the corridor: the side walls are seen at grazing angles.
    cov.mark_view(grid, 0.3, 0.5, 0.0, math.radians(60), 2.5, max_incidence=math.radians(60))
    assert not cov.seen[2, 40] and not cov.seen[17, 40]
    # Facing the side wall (+y) from 0.4 m away sees it.
    cov.mark_view(grid, 1.5, 0.5, math.pi / 2, math.radians(40), 2.5, min_range=0.2)
    assert cov.seen[17, 30]


def test_walls_block_the_view():
    grid, info = corridor_map()
    grid[2:18, 30] = 100              # a cross wall at x = 1.5 m
    cov = fresh(info)
    cov.mark_view(grid, 0.6, 0.5, 0.0, math.radians(40), 2.5)
    assert cov.seen[10, 29]           # front of the cross wall
    assert not cov.seen[10, 57]       # end wall behind it


def test_update_grid_keeps_seen_cells_when_the_map_grows():
    cov = CoverageMap()
    cov.update_grid(GridInfo(RES, 0.0, 0.0, 10, 10))
    cov.seen[3, 4] = True
    # Map grows by 1 m to the left and 0.5 m down: origin moves to (-1, -0.5).
    cov.update_grid(GridInfo(RES, -1.0, -0.5, 40, 30))
    assert cov.seen.shape == (30, 40)
    assert cov.seen[3 + 10, 4 + 20]
    assert np.count_nonzero(cov.seen) == 1


def test_travel_distance_goes_round_walls():
    passable = np.ones((10, 10), bool)
    passable[0:8, 5] = False          # wall with a gap at the bottom rows 8..9
    d = travel_distance(passable, (0, 0))
    assert d[0, 9] > 9                # must go round
    assert np.isfinite(d[0, 9])
    passable[8:, 5] = False
    assert np.isinf(travel_distance(passable, (0, 0))[0, 9])


def test_line_of_sight():
    grid, info = corridor_map()
    occ = grid == 100
    assert line_of_sight(occ, info, 0.5, 0.5, 2.5, 0.5)
    occ[:, 30] = True
    assert not line_of_sight(occ, info, 0.5, 0.5, 2.5, 0.5)
    # Ending on a wall counts as visible.
    assert line_of_sight(occ, info, 0.5, 0.5, 1.5, 0.5)


def l_shaped_map():
    """Return an L: corridor along +x (y 0..1), then a dead end going up at x 2..3 (y 1..3)."""
    h, w = 64, 64
    grid = np.full((h, w), 100, np.int8)
    grid[2:18, 2:58] = 0              # x 0.1..2.9, y 0.1..0.9
    grid[2:58, 40:58] = 0             # x 2.0..2.9, y 0.1..2.9
    return grid, GridInfo(RES, 0.0, 0.0, w, h)


def test_plan_view_goal_picks_a_reachable_pose_facing_unseen_walls():
    grid, info = l_shaped_map()
    cov = fresh(info)
    # Pretend everything near the start was seen; the dead end (y > 1.5) wasn't.
    cov.seen[:30, :] = True
    goal = plan_view_goal(grid, info, cov, (0.5, 0.5), robot_radius=0.3,
                          min_cluster_cells=4)
    assert goal is not None
    assert goal.aim_y > 1.4
    r, c = info.world_to_cell(goal.x, goal.y)
    assert grid[r, c] == 0
    assert 0.6 <= math.hypot(goal.aim_x - goal.x, goal.aim_y - goal.y) <= 1.6
    # The pose looks at its aim point.
    assert abs(angle_diff(math.atan2(goal.aim_y - goal.y, goal.aim_x - goal.x),
                          goal.yaw)) < 1e-6


def test_plan_view_goal_none_when_all_seen_or_blacklisted():
    grid, info = l_shaped_map()
    cov = fresh(info)
    cov.seen[:, :] = True
    assert plan_view_goal(grid, info, cov, (0.5, 0.5)) is None
    cov.seen[30:, :] = False
    first = plan_view_goal(grid, info, cov, (0.5, 0.5), min_cluster_cells=4)
    assert first is not None
    # Blacklisting every aim point eventually leaves nothing.
    blacklist = []
    for _ in range(20):
        g = plan_view_goal(grid, info, cov, (0.5, 0.5), min_cluster_cells=4,
                           blacklist=blacklist)
        if g is None:
            break
        blacklist.append((g.aim_x, g.aim_y))
    assert g is None


def test_plan_view_goal_ignores_unreachable_walls():
    grid, _ = corridor_map()
    big = np.full((40, 60), 100, np.int8)
    big[:20, :] = grid
    big[24:36, 10:50] = 0             # a closed room nobody can reach
    info = GridInfo(RES, 0.0, 0.0, 60, 40)
    cov = fresh(info)
    cov.seen[:20, :] = True
    assert plan_view_goal(big, info, cov, (0.5, 0.5), min_cluster_cells=4) is None


def test_spin_planner():
    sp = SpinPlanner(1.0)
    assert sp.should_spin(0, 0)
    sp.record(0, 0)
    assert not sp.should_spin(0.5, 0.5)
    assert sp.should_spin(1.0, 0.2)


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, 3.0])
def test_yaw_from_quaternion_with_pitch(yaw):
    # Rotation: yaw about z, then 20 deg pitch about the new y (like camera_link).
    pitch = math.radians(20)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    qw, qx, qy, qz = cy * cp, -sy * sp, cy * sp, sy * cp
    assert abs(angle_diff(yaw_from_quaternion(qx, qy, qz, qw), yaw)) < 1e-9


def test_angle_diff_wraps():
    assert abs(angle_diff(math.pi - 0.1, -math.pi + 0.1) - (-0.2)) < 1e-9


def test_reachable_area_leaves_out_rooms_seen_over_walls():
    big = np.full((40, 60), 100, np.int8)
    big[2:18, 2:58] = 0               # the corridor the robot is in
    big[24:36, 10:50] = 0             # a closed room the lidar saw over the wall
    info = GridInfo(RES, 0.0, 0.0, 60, 40)
    travel, swept = reachable_area(big, info, (0.5, 0.5), robot_radius=0.25)
    assert np.isfinite(travel[10, 10]) and swept[2, 10]     # wall face of the corridor
    assert not swept[24:36, 10:50].any()
    cov = fresh(info)
    seen, total = cov.stats(big, swept)
    assert seen == 0 and total == np.count_nonzero(wall_faces(big == 0, big == 100) & swept)


def test_entrance_wall_closes_the_start_cell_behind_the_robot():
    # Open floor 4 x 4 m, robot at (1, 2) facing +x: 'back' is the edge at x = 0.5.
    grid = np.zeros((80, 80), np.int8)
    info = GridInfo(RES, -1.0, 0.0, 80, 80)
    segs = [start_to_world((1.0, 2.0, 0.0), [(a, b), (c, d)])
            for a, b, c, d in entrance_segments(parse_edges('back'))]
    segs = [(p[0][0], p[0][1], p[1][0], p[1][1]) for p in segs]
    fenced, virtual = apply_fence(grid, info, segs)
    r, c = info.world_to_cell(0.5, 2.0)
    assert fenced[r, c] == 100 and virtual[r, c]
    assert fenced[info.world_to_cell(1.0, 2.0)] == 0            # the robot's cell stays free
    rows = np.nonzero(virtual[:, c])[0]
    assert rows.min() * RES + info.origin_y <= 1.5 and rows.max() * RES + info.origin_y >= 2.5
    assert not grid.any()                                        # input untouched


def test_entrance_follows_the_start_heading():
    # Facing +y, 'back' is the edge below the robot (y = -0.5 in the map).
    x0, y0, x1, y1 = entrance_segments(['back'])[0]
    (ax, ay), (bx, by) = start_to_world((0.0, 0.0, math.pi / 2), [(x0, y0), (x1, y1)])
    assert abs(ay + 0.5) < 1e-9 and abs(by + 0.5) < 1e-9
    assert abs(abs(bx - ax) - 1.1) < 1e-9


def test_fence_polygon_closes_everything_outside():
    grid = np.zeros((60, 60), np.int8)
    info = GridInfo(RES, 0.0, 0.0, 60, 60)
    poly = [(0.5, 0.5), (2.5, 0.5), (2.5, 2.5), (0.5, 2.5)]
    fenced, virtual = apply_fence(grid, info, polygon=poly)
    assert fenced[info.world_to_cell(1.5, 1.5)] == 0
    assert fenced[info.world_to_cell(0.2, 1.5)] == 100
    assert fenced[info.world_to_cell(2.8, 2.8)] == 100


def test_parse_edges_and_polygon():
    assert parse_edges('') == [] and parse_edges('none') == []
    assert parse_edges('Back, left') == ['back', 'left']
    assert parse_polygon('') is None
    assert parse_polygon('0,0, 1,0, 1,1') == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    with pytest.raises(ValueError):
        parse_edges('behind')
    with pytest.raises(ValueError):
        parse_polygon('0,0,1')


def test_fence_seals_unknown_strip_next_to_virtual_wall():
    grid = np.full((40, 40), -1, np.int8)
    grid[:, 20:] = 0                    # robot side known free from x = 1.0 m
    info = GridInfo(RES, 0.0, 0.0, 40, 40)
    fenced, virtual = apply_fence(grid, info, [(0.5, 0.0, 0.5, 2.0)], seal=0.3)
    r = 20
    assert fenced[r, info.world_to_cell(0.7, 1.0)[1]] == 0    # unknown strip -> free
    assert fenced[r, info.world_to_cell(0.5, 1.0)[1]] == 100  # the wall
    assert fenced[r, info.world_to_cell(0.05, 1.0)[1]] == -1  # beyond seal distance
    unsealed, _ = apply_fence(grid, info, [(0.5, 0.0, 0.5, 2.0)], seal=0.0)
    assert unsealed[r, info.world_to_cell(0.7, 1.0)[1]] == -1
