"""
ROS-free logic for the maze explorer: camera coverage of walls and where to look next.

Everything works on a 2D occupancy grid in the layout of ``nav_msgs/OccupancyGrid``:
``grid[row, col]`` with row = y and col = x, values -1 unknown, 0..100 occupancy.

The idea:
- A *wall face* is a free cell next to an occupied cell. QR codes and posters hang on walls,
  so the camera has to see every wall face once.
- While the robot moves, :meth:`CoverageMap.mark_view` casts rays over the map inside the
  camera's horizontal field of view and marks the faces it sees (not through walls, not too
  far away, not at a grazing angle).
- When explore_lite has no frontiers left, :func:`plan_view_goal` groups the unseen faces into
  clusters and picks a pose the robot can drive to and look at the nearest cluster from.
"""

from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np

OCC_THRESHOLD = 50  # occupancy >= this is an obstacle; 0 <= value < this is free


@dataclass(frozen=True)
class GridInfo:
    """Geometry of an occupancy grid (``nav_msgs/MapMetaData`` without the rotation)."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def world_to_cell(self, x, y):
        """Return (row, col) of the cell holding world point (x, y). May be outside the grid."""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row, col):
        """Return the world (x, y) of the centre of cell (row, col)."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def inside(self, row, col):
        """Return True when (row, col) is a cell of this grid."""
        return 0 <= row < self.height and 0 <= col < self.width


@dataclass
class ViewGoal:
    """A pose to drive to (x, y, yaw) and the wall point (aim_x, aim_y) it looks at."""

    x: float
    y: float
    yaw: float
    aim_x: float
    aim_y: float
    cluster_cells: int
    travel_m: float


def classify(grid):
    """Split an occupancy grid into boolean (free, occupied) masks; the rest is unknown."""
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid < OCC_THRESHOLD)
    occupied = grid >= OCC_THRESHOLD
    return free, occupied


def wall_faces(free, occupied):
    """Free cells with an occupied 4-neighbour: the strip of floor in front of every wall."""
    occ = occupied.astype(np.uint8)
    near = cv2.dilate(occ, np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8))
    return free & (near > 0)


def wall_normals(occupied):
    """Per-cell gradient (gx, gy) of the smoothed occupied mask. Points into walls."""
    occ = cv2.GaussianBlur(occupied.astype(np.float32), (5, 5), 1.0)
    gx = cv2.Sobel(occ, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(occ, cv2.CV_32F, 0, 1, ksize=3)
    return gx, gy


class CoverageMap:
    """Which wall faces the camera has seen, kept in world coordinates as the map grows."""

    def __init__(self):
        """Start empty; call :meth:`update_grid` with the first map."""
        self.info = None
        self.seen = None

    def update_grid(self, info):
        """Follow a new map size/origin, keeping what was already seen at the same places."""
        if self.info is None or not math.isclose(info.resolution, self.info.resolution):
            self.info = info
            self.seen = np.zeros((info.height, info.width), bool)
            return
        if info == self.info:
            return
        old, old_info = self.seen, self.info
        new = np.zeros((info.height, info.width), bool)
        dc = int(round((old_info.origin_x - info.origin_x) / info.resolution))
        dr = int(round((old_info.origin_y - info.origin_y) / info.resolution))
        r0, c0 = max(0, dr), max(0, dc)
        r1 = min(info.height, dr + old_info.height)
        c1 = min(info.width, dc + old_info.width)
        if r1 > r0 and c1 > c0:
            new[r0:r1, c0:c1] = old[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
        self.info, self.seen = info, new

    def mark_view(self, grid, x, y, yaw, hfov, max_range, min_range=0.3,
                  max_incidence=math.radians(60.0), face_depth=2):
        """
        Mark the wall faces visible from a camera at (x, y) looking along ``yaw``.

        Rays are cast across ``hfov`` (rad). A ray stops at the first occupied or unknown
        cell. If it stops at a wall between ``min_range`` and ``max_range`` and hits it no
        more than ``max_incidence`` away from straight on, the last ``face_depth`` free cells
        in front of the hit are marked seen. Returns the number of newly seen cells.
        """
        info = self.info
        res = info.resolution
        free, occupied = classify(grid)
        n_rays = max(3, int(math.ceil(hfov / (0.5 * res / max_range))) + 1)
        n_steps = int(math.ceil(max_range / (0.5 * res))) + 1
        angles = yaw + np.linspace(-hfov / 2.0, hfov / 2.0, n_rays)
        dists = np.linspace(0.0, max_range, n_steps)
        dx, dy = np.cos(angles), np.sin(angles)
        px = x + np.outer(dx, dists)
        py = y + np.outer(dy, dists)
        cols = np.floor((px - info.origin_x) / res).astype(int)
        rows = np.floor((py - info.origin_y) / res).astype(int)
        inside = (rows >= 0) & (rows < info.height) & (cols >= 0) & (cols < info.width)
        rows_c = np.clip(rows, 0, info.height - 1)
        cols_c = np.clip(cols, 0, info.width - 1)
        is_free = free[rows_c, cols_c] & inside
        is_occ = occupied[rows_c, cols_c] & inside
        blocked = ~is_free
        # Index of the first non-free sample on each ray (n_steps when the ray stays free).
        first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), n_steps)
        gx, gy = wall_normals(occupied)
        cos_min = math.cos(max_incidence)
        before = int(np.count_nonzero(self.seen))
        for i in range(n_rays):
            k = first[i]
            if k >= n_steps or k == 0 or not is_occ[i, k] or dists[k] < min_range:
                continue
            r, c = rows_c[i, k], cols_c[i, k]
            g = math.hypot(gx[r, c], gy[r, c])
            if g > 1e-6 and abs(dx[i] * gx[r, c] + dy[i] * gy[r, c]) / g < cos_min:
                continue  # grazing view: a QR code would be unreadable
            j = k - 1
            marked = 0
            while j >= 0 and marked < face_depth:
                self.seen[rows_c[i, j], cols_c[i, j]] = True
                if (rows_c[i, j], cols_c[i, j]) != (rows_c[i, j - 1], cols_c[i, j - 1]):
                    marked += 1
                j -= 1
        return int(np.count_nonzero(self.seen)) - before

    def unseen_faces(self, grid):
        """Return the mask of wall faces not seen yet (any reachability)."""
        free, occupied = classify(grid)
        return wall_faces(free, occupied) & ~self.seen

    def stats(self, grid, reachable=None):
        """Return (seen, total) wall-face cell counts, optionally only inside ``reachable``."""
        free, occupied = classify(grid)
        faces = wall_faces(free, occupied)
        if reachable is not None:
            faces &= reachable
        return int(np.count_nonzero(faces & self.seen)), int(np.count_nonzero(faces))


def clearance_m(free, resolution):
    """Distance in metres from every free cell to the nearest non-free cell."""
    dist = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5)
    return dist * resolution


def travel_distance(passable, start, max_snap=10):
    """
    Breadth-first travel distance in cells from ``start`` (row, col) over ``passable``.

    8-connected, diagonal steps cost sqrt(2) (approximated by a two-queue BFS on 4-neighbours
    plus diagonals). If ``start`` itself isn't passable, the nearest passable cell within
    ``max_snap`` cells is used. Unreachable cells are ``inf``.
    """
    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    sr, sc = start
    if not (0 <= sr < h and 0 <= sc < w and passable[sr, sc]):
        best = None
        for r in range(max(0, sr - max_snap), min(h, sr + max_snap + 1)):
            for c in range(max(0, sc - max_snap), min(w, sc + max_snap + 1)):
                if passable[r, c]:
                    d = (r - sr) ** 2 + (c - sc) ** 2
                    if best is None or d < best[0]:
                        best = (d, r, c)
        if best is None:
            return dist
        sr, sc = best[1], best[2]
    dist[sr, sc] = 0.0
    queue = deque([(sr, sc)])
    steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
    while queue:
        r, c = queue.popleft()
        d0 = dist[r, c]
        for dr, dc, cost in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and passable[nr, nc] and d0 + cost < dist[nr, nc]:
                dist[nr, nc] = d0 + cost
                queue.append((nr, nc))
    return dist


def line_of_sight(occupied, info, x0, y0, x1, y1, stop_short=0.1):
    """
    Return True when the segment (x0, y0)->(x1, y1) crosses no occupied cell.

    The last ``stop_short`` metres are not checked, so a segment ending on a wall counts.
    """
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= stop_short:
        return True
    n = int(math.ceil((length - stop_short) / (0.5 * info.resolution))) + 1
    t = np.linspace(0.0, (length - stop_short) / length, n)
    cols = np.floor((x0 + t * (x1 - x0) - info.origin_x) / info.resolution).astype(int)
    rows = np.floor((y0 + t * (y1 - y0) - info.origin_y) / info.resolution).astype(int)
    ok = (rows >= 0) & (rows < info.height) & (cols >= 0) & (cols < info.width)
    return not occupied[rows[ok], cols[ok]].any()


def reachable_area(grid, info, robot_xy, robot_radius=0.3, margin=0.1):
    """
    Return (travel_m, swept): where the robot can go and the free space around that.

    ``travel_m`` is the travel distance in metres from the robot to every cell its centre
    can reach with ``robot_radius`` clearance (``inf`` elsewhere). ``swept`` is the free
    area within ``robot_radius + margin`` of those cells: the floor in front of every wall
    the robot can drive up to. It leaves out closed-off areas the lidar saw over the low
    walls, which the robot can neither enter nor see properly.
    """
    free, _ = classify(grid)
    res = info.resolution
    passable = free & (clearance_m(free, res) >= robot_radius)
    travel = travel_distance(passable, info.world_to_cell(*robot_xy)) * res
    k = 2 * int(math.ceil((robot_radius + margin) / res)) + 1
    disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    swept = cv2.dilate(np.isfinite(travel).astype(np.uint8), disk) > 0
    return travel, swept & free


def plan_view_goal(grid, info, coverage, robot_xy, robot_radius=0.3, view_min=0.6,
                   view_ideal=1.0, view_max=1.6, min_cluster_cells=6, blacklist=(),
                   blacklist_radius=0.5, candidate_stride=2):
    """
    Pick the cheapest pose from which the camera sees a cluster of unseen wall faces.

    Only faces in the robot's reachable area (:func:`reachable_area`) count. For each cluster of
    at least ``min_cluster_cells`` unseen face cells (aim point: the cluster cell nearest its
    centroid, skipped if within ``blacklist_radius`` of a ``blacklist`` point), viewpoints
    are free cells with at least ``robot_radius`` clearance, reachable from the robot, between
    ``view_min`` and ``view_max`` from the aim point and with a clear line of sight to it.
    Score = travel distance + 2 * |view distance - view_ideal|. Returns a :class:`ViewGoal`
    or None when nothing is left to look at.
    """
    grid = np.asarray(grid)
    res = info.resolution
    _, occupied = classify(grid)
    travel, swept = reachable_area(grid, info, robot_xy, robot_radius)
    targets = coverage.unseen_faces(grid) & swept
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(
        targets.astype(np.uint8), connectivity=8)
    reachable = np.isfinite(travel)
    cand_r, cand_c = np.nonzero(reachable)
    keep = (cand_r % candidate_stride == 0) & (cand_c % candidate_stride == 0)
    cand_r, cand_c = cand_r[keep], cand_c[keep]
    if cand_r.size == 0:
        return None
    cand_x = info.origin_x + (cand_c + 0.5) * res
    cand_y = info.origin_y + (cand_r + 0.5) * res
    best = None
    for label in range(1, n):
        size = int(stats[label, cv2.CC_STAT_AREA])
        if size < min_cluster_cells:
            continue
        rr, cc = np.nonzero(labels == label)
        cx, cy = centroids[label]  # (col, row)
        i = int(np.argmin((cc - cx) ** 2 + (rr - cy) ** 2))
        aim_x, aim_y = info.cell_to_world(rr[i], cc[i])
        if any(math.hypot(aim_x - bx, aim_y - by) < blacklist_radius for bx, by in blacklist):
            continue
        d = np.hypot(cand_x - aim_x, cand_y - aim_y)
        sel = np.nonzero((d >= view_min) & (d <= view_max))[0]
        if sel.size == 0:
            continue
        score = travel[cand_r[sel], cand_c[sel]] + 2.0 * np.abs(d[sel] - view_ideal)
        for j in sel[np.argsort(score)]:
            if best is not None and score_of(travel, cand_r[j], cand_c[j], d[j],
                                             view_ideal) >= best[0]:
                break
            if line_of_sight(occupied, info, cand_x[j], cand_y[j], aim_x, aim_y):
                s = score_of(travel, cand_r[j], cand_c[j], d[j], view_ideal)
                yaw = math.atan2(aim_y - cand_y[j], aim_x - cand_x[j])
                best = (s, ViewGoal(float(cand_x[j]), float(cand_y[j]), yaw, aim_x, aim_y,
                                    size, float(travel[cand_r[j], cand_c[j]])))
                break
    return None if best is None else best[1]


def score_of(travel, r, c, view_dist, view_ideal):
    """Score of one viewpoint: travel distance plus a penalty for a non-ideal view distance."""
    return float(travel[r, c]) + 2.0 * abs(float(view_dist) - view_ideal)


class SpinPlanner:
    """Decide when to spin 360°: whenever the robot is ``spacing`` metres from all past spins."""

    def __init__(self, spacing):
        """Create a planner that spins every ``spacing`` metres of new ground."""
        self.spacing = spacing
        self.points = []

    def should_spin(self, x, y):
        """Return True if no earlier spin happened within ``spacing`` of (x, y)."""
        return all(math.hypot(x - px, y - py) >= self.spacing for px, py in self.points)

    def record(self, x, y):
        """Remember that the robot spun at (x, y)."""
        self.points.append((x, y))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Heading of a frame's x axis projected on the ground plane (works for pitched cameras)."""
    # x axis of the rotation matrix
    vx = 1.0 - 2.0 * (qy * qy + qz * qz)
    vy = 2.0 * (qx * qy + qz * qw)
    return math.atan2(vy, vx)


def angle_diff(a, b):
    """Return a - b wrapped to [-pi, pi)."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


# ------------------------------------------------------------------ virtual fence
EDGES = {'front': ((0.5, -0.5), (0.5, 0.5)), 'back': ((-0.5, -0.5), (-0.5, 0.5)),
         'left': ((-0.5, 0.5), (0.5, 0.5)), 'right': ((-0.5, -0.5), (0.5, -0.5))}


def parse_edges(text):
    """Parse ``'back,left'`` into a list of start-cell edge names; '' or 'none' gives []."""
    names = [t.strip().lower() for t in text.replace(';', ',').split(',') if t.strip()]
    names = [n for n in names if n != 'none']
    bad = [n for n in names if n not in EDGES]
    if bad:
        raise ValueError(f'unknown edge(s) {bad}; use front, back, left, right')
    return names


def parse_polygon(text):
    """Parse ``'x1,y1,x2,y2,...'`` (metres) into a list of (x, y); '' gives None."""
    vals = [float(v) for v in text.replace(';', ',').split(',') if v.strip()]
    if not vals:
        return None
    if len(vals) % 2 or len(vals) < 6:
        raise ValueError('fence needs at least 3 points as x1,y1,x2,y2,x3,y3,...')
    return list(zip(vals[0::2], vals[1::2]))


def start_to_world(start, points):
    """
    Transform (x, y) points from the start frame into the map frame.

    The start frame has its origin at the robot's start pose ``start`` = (x, y, yaw), x along
    the robot's heading and y to its left.
    """
    sx, sy, syaw = start
    c, s = math.cos(syaw), math.sin(syaw)
    return [(sx + c * x - s * y, sy + s * x + c * y) for x, y in points]


def entrance_segments(edges, cell=1.0, extend=0.05):
    """
    Return the start-cell edges as segments in the start frame.

    The robot is assumed to stand at the centre of the start cell, facing along the grid.
    Each segment is ``extend`` metres longer at both ends, so it closes the gap up to the
    real walls on either side.
    """
    segs = []
    for name in edges:
        (x0, y0), (x1, y1) = EDGES[name]
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy)
        ex, ey = dx / n * extend / cell, dy / n * extend / cell
        segs.append(((x0 - ex) * cell, (y0 - ey) * cell, (x1 + ex) * cell, (y1 + ey) * cell))
    return segs


def apply_fence(grid, info, segments=(), polygon=None, thickness=0.1, seal=0.5):
    """
    Add virtual walls to an occupancy grid; return (fenced_grid, virtual_mask).

    ``segments`` are (x0, y0, x1, y1) lines in map coordinates, drawn ``thickness`` metres
    thick. With a ``polygon`` (map coordinates), every cell outside it becomes occupied.
    Fenced cells are set to 100 whatever they were (free, unknown or occupied), so no
    frontier or path can lead through them.

    Unknown cells within ``seal`` metres of a virtual wall are set free. The lidar can't see
    behind the robot, so the strip between the robot and a wall across the start cell's
    entrance would otherwise stay unknown forever: a frontier explore_lite keeps "reaching"
    without ever clearing it.
    """
    grid = np.asarray(grid, np.int8)
    mask = np.zeros(grid.shape, np.uint8)
    res = info.resolution

    def px(x, y):
        return (int(round((x - info.origin_x) / res - 0.5)),
                int(round((y - info.origin_y) / res - 0.5)))

    t = max(1, int(round(thickness / res)))
    for x0, y0, x1, y1 in segments:
        cv2.line(mask, px(x0, y0), px(x1, y1), 1, thickness=t)
    if polygon is not None:
        inside = np.zeros(grid.shape, np.uint8)
        cv2.fillPoly(inside, [np.array([px(x, y) for x, y in polygon], np.int32)], 1)
        mask |= (inside == 0).astype(np.uint8)
    virtual = mask > 0
    fenced = grid.copy()
    if seal > 0 and virtual.any():
        dist = cv2.distanceTransform((~virtual).astype(np.uint8), cv2.DIST_L2, 5) * res
        fenced[(fenced < 0) & (dist <= seal)] = 0
    fenced[virtual] = 100
    return fenced, virtual
