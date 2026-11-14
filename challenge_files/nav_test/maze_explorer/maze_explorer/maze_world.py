"""
ROS-free maze description -> Gazebo (classic) world, for testing exploration in simulation.

Maze text format (see ``sim/mazes/*.txt``)::

    +---+---+---+
    | S     |...|
    +   +---+---+
    |       ### |
    +---+---+---+

- ``+`` corners. Between two corners, ``---`` is a wall and three spaces are an opening.
- In cell rows, ``|`` is a wall and a space is an opening.
- Cell contents (the 3 characters between two wall positions):
  ``   `` normal cell, `` S `` start cell (the robot spawns here, facing +x),
  ``###`` solid block (e.g. stairs the robot shouldn't climb), ``...`` not part of the maze.
- Edges between a maze cell and a ``...`` cell are always walls. Edges on the outer border
  are as drawn, so a gap in the border is an opening to the outside (the robot could leave).
- Lines starting with ``#`` (in column 0) and blank lines are ignored.

World coordinates: the start cell's centre is (0, 0), x to the right, y up the page.
"""

from dataclasses import dataclass, field

NORMAL, START, SOLID, VOID = 'normal', 'start', 'solid', 'void'
_CONTENT = {'   ': NORMAL, ' S ': START, '###': SOLID, '...': VOID}


@dataclass
class Maze:
    """A parsed maze: cell kinds and the walls on every edge."""

    rows: int
    cols: int
    kinds: list                      # kinds[r][c], r = 0 is the top row
    h_walls: list                    # h_walls[r][c]: wall above cell (r, c); r in 0..rows
    v_walls: list                    # v_walls[r][c]: wall left of cell (r, c); c in 0..cols
    start: tuple = field(default=(0, 0))

    def is_cell(self, r, c):
        """Return True for an in-bounds cell that belongs to the maze (not void)."""
        return 0 <= r < self.rows and 0 <= c < self.cols and self.kinds[r][c] != VOID

    def center(self, r, c, cell=1.0):
        """Return the world (x, y) of the centre of cell (r, c)."""
        sr, sc = self.start
        return ((c - sc) * cell, (sr - r) * cell)

    def open_cells(self):
        """Return the cells the robot can stand in (normal and start)."""
        return [(r, c) for r in range(self.rows) for c in range(self.cols)
                if self.kinds[r][c] in (NORMAL, START)]


def parse_maze(text):
    """Parse the maze text format described in the module docstring."""
    lines = [ln.rstrip() for ln in text.splitlines()
             if ln.strip() and not ln.startswith('#')]
    if len(lines) < 3 or len(lines) % 2 == 0:
        raise ValueError('maze needs an odd number (>= 3) of non-comment lines')
    width = max(len(ln) for ln in lines)
    if (width - 1) % 4:
        raise ValueError('maze lines must be 4 * columns + 1 characters wide')
    lines = [ln.ljust(width) for ln in lines]
    rows, cols = (len(lines) - 1) // 2, (width - 1) // 4
    kinds = []
    start = None
    for r in range(rows):
        line = lines[2 * r + 1]
        row = []
        for c in range(cols):
            content = line[4 * c + 1:4 * c + 4]
            if content not in _CONTENT:
                raise ValueError(f'unknown cell content {content!r} at row {r}, column {c}')
            kind = _CONTENT[content]
            if kind == START:
                if start is not None:
                    raise ValueError('more than one start cell')
                start = (r, c)
            row.append(kind)
        kinds.append(row)
    if start is None:
        raise ValueError('no start cell (" S ")')
    h_walls = [[lines[2 * r][4 * c + 1:4 * c + 4] == '---' for c in range(cols)]
               for r in range(rows + 1)]
    v_walls = [[lines[2 * r + 1][4 * c] == '|' for c in range(cols + 1)] for r in range(rows)]
    maze = Maze(rows, cols, kinds, h_walls, v_walls, start)
    # Close every edge between a maze cell and a void cell (borders stay as drawn).
    for r in range(1, rows):
        for c in range(cols):
            if maze.is_cell(r - 1, c) != maze.is_cell(r, c):
                h_walls[r][c] = True
    for r in range(rows):
        for c in range(1, cols):
            if maze.is_cell(r, c - 1) != maze.is_cell(r, c):
                v_walls[r][c] = True
    return maze


def wall_boxes(maze, cell=1.0, thickness=0.05, height=0.45):
    """
    Return the walls and solid blocks as boxes ``(cx, cy, sx, sy, sz)`` in world metres.

    Only walls touching at least one maze cell are returned. Wall boxes are one cell plus
    one thickness long, so neighbouring walls overlap at the corners and leave no gaps.
    """
    boxes = []
    sr, sc = maze.start
    length = cell + thickness
    for r in range(maze.rows + 1):
        for c in range(maze.cols):
            if maze.h_walls[r][c] and (maze.is_cell(r - 1, c) or maze.is_cell(r, c)):
                x = (c - sc) * cell
                y = (sr - r + 0.5) * cell
                boxes.append((x, y, length, thickness, height))
    for r in range(maze.rows):
        for c in range(maze.cols + 1):
            if maze.v_walls[r][c] and (maze.is_cell(r, c - 1) or maze.is_cell(r, c)):
                x = (c - sc - 0.5) * cell
                y = (sr - r) * cell
                boxes.append((x, y, thickness, length, height))
    for r in range(maze.rows):
        for c in range(maze.cols):
            if maze.kinds[r][c] == SOLID:
                x, y = maze.center(r, c, cell)
                boxes.append((x, y, cell - thickness, cell - thickness, height))
    return boxes


def _box_xml(name, x, y, sx, sy, sz):
    geo = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    return (f'<link name="{name}"><pose>{x:.4f} {y:.4f} {sz / 2:.4f} 0 0 0</pose>'
            f'<collision name="c">{geo}</collision>'
            f'<visual name="v">{geo}<material><ambient>0.8 0.75 0.6 1</ambient>'
            f'<diffuse>0.8 0.75 0.6 1</diffuse></material></visual></link>')


def maze_to_sdf(maze, cell=1.0, thickness=0.05, height=0.45):
    """
    Return a complete Gazebo-classic SDF world string for ``maze``.

    The world has its own ground plane and light (no model downloads) and a 2 ms physics
    step, which keeps the lidar simulation close to real time on a laptop CPU.
    """
    links = '\n'.join(_box_xml(f'w{i}', *b)
                      for i, b in enumerate(wall_boxes(maze, cell, thickness, height)))
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
<world name="maze">
  <physics type="ode">
    <max_step_size>0.002</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>500</real_time_update_rate>
  </physics>
  <light name="sun" type="directional">
    <cast_shadows>false</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse>
    <direction>-0.3 0.2 -1</direction>
  </light>
  <model name="ground">
    <static>true</static>
    <link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size>
      </plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size>
      </plane></geometry><material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
    </link>
  </model>
  <model name="maze_walls">
    <static>true</static>
{links}
  </model>
</world>
</sdf>
"""
