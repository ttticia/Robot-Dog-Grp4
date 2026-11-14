"""Tests for maze_explorer.maze_world (maze text -> Gazebo world)."""

import os
import xml.etree.ElementTree as ET

import pytest

from maze_explorer.maze_world import maze_to_sdf, parse_maze, SOLID, VOID, wall_boxes

MAZES = os.path.join(os.path.dirname(__file__), '..', 'sim', 'mazes')

SMALL = """
# comment
+---+---+
| S     |
+   +---+
|   |...|
+---+---+
"""


def test_parse_small_maze():
    m = parse_maze(SMALL)
    assert (m.rows, m.cols, m.start) == (2, 2, (0, 0))
    assert m.kinds[1][1] == VOID
    assert not m.v_walls[0][1]        # open between the two top cells
    assert not m.h_walls[1][0]        # open below the start
    assert m.h_walls[1][1]            # wall above the void cell
    assert m.v_walls[1][1]            # wall between (1, 0) and the void
    assert m.center(0, 0) == (0.0, 0.0)
    assert m.center(1, 0) == (0.0, -1.0)
    assert m.center(0, 1) == (1.0, 0.0)


def test_void_cells_are_walled_even_if_drawn_open():
    m = parse_maze('+---+---+\n| S  ...|\n+---+---+\n')   # opening drawn towards void
    assert m.v_walls[0][1]


def test_wall_boxes_cover_every_maze_edge():
    m = parse_maze(SMALL)
    boxes = wall_boxes(m)
    # 3 open cells: outer walls 2 (top) + 1 (right of start row) + ... count by hand:
    # top 2, bottom of (1,0) 1, left 2, right of (0,1) 1, below (0,1) 1, right of (1,0) 1.
    assert len(boxes) == 8
    assert all(abs(b[4] - 0.45) < 1e-9 for b in boxes)


def test_solid_cell_is_a_block():
    m = parse_maze('+---+---+\n| S  ###|\n+---+---+\n')
    assert m.kinds[0][1] == SOLID
    blocks = [b for b in wall_boxes(m) if b[2] > 0.5 and b[3] > 0.5]
    assert len(blocks) == 1 and blocks[0][:2] == (1.0, 0.0)


@pytest.mark.parametrize('bad', [
    '+---+\n|   |\n+---+\n',                        # no start
    '+---+---+\n| S | S |\n+---+---+\n',             # two starts
    '+---+\n| x |\n+---+\n',                        # unknown content
    '+---+\n| S |\n',                                # even number of lines
])
def test_bad_mazes_raise(bad):
    with pytest.raises(ValueError):
        parse_maze(bad)


@pytest.mark.parametrize('name', sorted(os.listdir(MAZES)))
def test_shipped_mazes_make_valid_worlds(name):
    with open(os.path.join(MAZES, name)) as f:
        m = parse_maze(f.read())
    sdf = maze_to_sdf(m)
    root = ET.fromstring(sdf.split('?>', 1)[1])
    links = root.findall('./world/model/link')
    assert len(links) > len(m.open_cells())


def test_example_maze_matches_the_step_layout():
    with open(os.path.join(MAZES, 'example_maze.txt')) as f:
        m = parse_maze(f.read())
    centres = sorted(m.center(r, c) for r, c in m.open_cells())
    # Cell centres of the STEP assembly (in metres, start cell at the origin).
    assert centres == sorted([(0.0, 0.0), (0.0, 1.0), (-1.0, 1.0), (-2.0, 1.0),
                              (-2.0, 0.0), (-1.0, 2.0)])


def test_border_openings_stay_open_and_void_edges_close():
    m = parse_maze('+---+---+\n  S  ...|\n+---+---+\n')
    assert not m.v_walls[0][0]        # gap in the west border: opening to the outside
    assert m.v_walls[0][1]            # start cell -> void: always a wall
    assert len([b for b in wall_boxes(m) if b[0] < -0.4]) == 0
