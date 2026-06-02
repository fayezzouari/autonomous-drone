r"""3-D path planning around obstacles, accounting for the drone's own size.

Given a start A, a goal B and an :class:`ObstacleField`, :class:`PathPlanner`
returns a collision-free polyline of waypoints A → … → B that the existing
``GotoController`` / ``Mission`` machinery can then fly.

Pipeline
--------
1. **Inflate** every obstacle by ``drone_radius + safety_margin`` so the drone
   can be treated as a point (Minkowski padding — see ``obstacles.py``).
2. **Fast path**: if the straight segment A→B is already clear, return ``[A, B]``.
3. **A\*** over a uniform 3-D lattice (26-connected) bounded to a box around A
   and B. A cell is usable when its centre is outside every inflated box and the
   edge into it doesn't cross one; step cost and heuristic are Euclidean.
4. **String-pull**: greedily shortcut the grid path with line-of-sight tests so
   the result is a handful of any-angle waypoints rather than a staircase.

Everything is pure-Python (no numpy) and deterministic, so it unit-tests cleanly.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .obstacles import ObstacleField, Vec3


@dataclass
class PlannerConfig:
    """Tuning for :class:`PathPlanner`. All distances in metres."""

    drone_radius: float = 0.30   # bounding radius of the airframe incl. props
    safety_margin: float = 0.20  # extra clearance kept from every obstacle
    resolution: float = 0.5      # lattice cell size for the A* search
    z_min: float = 0.5           # don't cruise below this altitude …
    z_max: float = 12.0          # … or above this one
    bounds_pad: float = 3.0      # expand the search box this far around A and B
    max_nodes: int = 200_000     # search expansion cap (returns failure if exceeded)
    algorithm: str = "theta"     # "theta" (any-angle) or "astar" (grid + shortcut)
    # Which box extent (w=width / t=d=depth / h=height) maps to world X, Y, Z.
    # Publisher emits verts as (±hw,±hd,±hh) → hw→X, hd→Y, hh→Z.
    obstacle_axes: str = "wdh"   # X=width, Y=depth, Z=height
    # Centre-coordinate axes to negate when parsing (source→world frame). The
    # publisher's centres are already world-frame, so no flip.
    obstacle_flip: str = ""
    # At startup, wait up to this long (s) for the first drone/obs message so the
    # initial plan already accounts for the retrieved obstacles.
    obstacle_wait: float = 3.0
    # ── path following (how the planned polyline is flown) ───────────────────
    # Horizontal speed cap while flying a planned route. Slower → the drone
    # tracks the line tighter, so it can't swing wide into a wall on turns.
    # 0 keeps the goto config's v_max_xy.
    cruise_speed: float = 1.5
    # Vertical speed cap while avoiding (gentle launch/descent; 0 = goto vz_max).
    climb_speed: float = 1.2
    # Minimum climb rate (m/s) kept when coupling climb to forward progress, so
    # takeoff / near-vertical legs aren't stalled. See GotoController.couple_climb.
    climb_floor: float = 0.25
    # Re-sample the planned polyline to this spacing (m) so consecutive targets
    # are close and the drone hugs the line instead of orbiting far waypoints.
    follow_spacing: float = 0.8
    # After planning the first route, hold position this long (s) so the path
    # (published on drone/path) can be inspected before the drone sets off.
    preview_pause: float = 3.0

    @property
    def clearance(self) -> float:
        return self.drone_radius + self.safety_margin

    @property
    def switch_radius(self) -> float:
        """Advance to the next path vertex once within this distance — kept
        well under the clearance so any corner-cut stays inside the safe band."""
        return min(self.follow_spacing, 0.5 * self.clearance)


@dataclass
class PathResult:
    waypoints: List[Vec3] = field(default_factory=list)
    found: bool = False
    direct: bool = False        # straight A→B was already clear
    expanded: int = 0           # A* nodes expanded (diagnostics)
    reason: str = ""
    length: float = 0.0

    def __bool__(self) -> bool:
        return self.found


def _dist(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


# 26-connected neighbour offsets (all combinations of -1/0/+1 except the centre).
_NEIGHBORS: List[Tuple[int, int, int]] = [
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if not (dx == 0 and dy == 0 and dz == 0)
]


class PathPlanner:
    def __init__(self, cfg: Optional[PlannerConfig] = None):
        self.cfg = cfg or PlannerConfig()

    # ── public API ───────────────────────────────────────────────────────────
    def plan(self, start: Vec3, goal: Vec3, field: ObstacleField) -> PathResult:
        cfg = self.cfg
        start = (float(start[0]), float(start[1]), float(start[2]))
        goal = (float(goal[0]), float(goal[1]), float(goal[2]))
        infl = field.inflate(cfg.clearance)

        # If the goal sits inside an obstacle there's nowhere legal to stop —
        # nudge it to the nearest free lattice cell so the run still makes sense.
        if infl.point_blocked(goal):
            free_goal = self._nearest_free(goal, infl, start)
            if free_goal is None:
                return PathResult(found=False, reason="goal is inside an obstacle")
            goal = free_goal

        # 2. straight-line fast path
        if not infl.segment_blocked(start, goal):
            return PathResult(waypoints=[start, goal], found=True, direct=True,
                              length=_dist(start, goal), reason="direct")

        # 3. search the lattice with the configured algorithm
        if cfg.algorithm == "astar":
            grid_path, expanded = self._astar(start, goal, infl)
        else:
            grid_path, expanded = self._theta(start, goal, infl)
        if grid_path is None:
            return PathResult(found=False, expanded=expanded,
                              reason="no collision-free path within bounds")

        # 4. shortcut + stitch real endpoints. (Theta* already returns an
        #    any-angle path; the shortcut only trims residual collinear points.)
        pts = [start] + grid_path + [goal]
        pts = self._shortcut(pts, infl)
        length = sum(_dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        return PathResult(waypoints=pts, found=True, expanded=expanded,
                          length=length, reason=cfg.algorithm)

    # ── lattice helpers ──────────────────────────────────────────────────────
    def _setup_grid(self, start: Vec3, goal: Vec3):
        cfg = self.cfg
        pad = cfg.bounds_pad
        lo = [min(start[i], goal[i]) - pad for i in range(3)]
        hi = [max(start[i], goal[i]) + pad for i in range(3)]
        # Constrain cruise altitude, but always keep the endpoints reachable.
        lo[2] = min(cfg.z_min, start[2], goal[2])
        hi[2] = max(cfg.z_max, start[2], goal[2])
        res = cfg.resolution
        origin = (lo[0], lo[1], lo[2])
        dims = tuple(max(1, int(math.ceil((hi[i] - lo[i]) / res)) + 1) for i in range(3))
        return origin, res, dims

    def _grid_funcs(self, start: Vec3, goal: Vec3):
        """Return (to_cell, to_pos, dims) for the search lattice over [start, goal]."""
        origin, res, dims = self._setup_grid(start, goal)

        def to_cell(p: Vec3) -> Tuple[int, int, int]:
            return tuple(
                min(dims[i] - 1, max(0, int(round((p[i] - origin[i]) / res))))
                for i in range(3)
            )

        def to_pos(c) -> Vec3:
            return (origin[0] + c[0] * res, origin[1] + c[1] * res, origin[2] + c[2] * res)

        return to_cell, to_pos, dims

    def _in_bounds(self, c, dims) -> bool:
        return 0 <= c[0] < dims[0] and 0 <= c[1] < dims[1] and 0 <= c[2] < dims[2]

    # ── A* (grid-constrained; relies on the shortcut pass for any-angle) ───────
    def _astar(self, start: Vec3, goal: Vec3, infl: ObstacleField):
        cfg = self.cfg
        to_cell, to_pos, dims = self._grid_funcs(start, goal)
        start_c, goal_c = to_cell(start), to_cell(goal)
        goal_pos = to_pos(goal_c)
        eps = 1.0 + 1e-3  # weighted A*: faster, shortcut re-optimises afterwards

        open_heap: List[Tuple[float, int, Tuple[int, int, int]]] = []
        counter = 0
        g_score = {start_c: 0.0}
        came_from = {}
        heapq.heappush(open_heap, (0.0, counter, start_c))
        closed = set()
        expanded = 0

        while open_heap:
            _, _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)
            expanded += 1
            if cur == goal_c:
                return self._reconstruct(came_from, cur, to_pos), expanded
            if expanded > cfg.max_nodes:
                return None, expanded

            cur_pos = to_pos(cur)
            base_g = g_score[cur]
            for dx, dy, dz in _NEIGHBORS:
                nc = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
                if not self._in_bounds(nc, dims) or nc in closed:
                    continue
                npos = to_pos(nc)
                if infl.point_blocked(npos) or infl.segment_blocked(cur_pos, npos):
                    continue
                tentative = base_g + _dist(cur_pos, npos)
                if tentative < g_score.get(nc, math.inf):
                    g_score[nc] = tentative
                    came_from[nc] = cur
                    counter += 1
                    heapq.heappush(open_heap,
                                   (tentative + eps * _dist(npos, goal_pos), counter, nc))

        return None, expanded

    # ── Theta* (any-angle: parent relaxes to grandparent when in line of sight) ─
    def _theta(self, start: Vec3, goal: Vec3, infl: ObstacleField):
        """Theta* — A* whose update step lets a node take its parent's parent as
        its own parent whenever there's clear line of sight, so path segments are
        not constrained to grid edges and turns only happen at obstacle corners.
        """
        cfg = self.cfg
        to_cell, to_pos, dims = self._grid_funcs(start, goal)
        start_c, goal_c = to_cell(start), to_cell(goal)
        goal_pos = to_pos(goal_c)

        def los(a, b) -> bool:  # clear line of sight between two cells
            return not infl.segment_blocked(to_pos(a), to_pos(b))

        open_heap: List[Tuple[float, int, Tuple[int, int, int]]] = []
        counter = 0
        g_score = {start_c: 0.0}
        parent = {start_c: start_c}
        heapq.heappush(open_heap, (_dist(to_pos(start_c), goal_pos), counter, start_c))
        closed = set()
        expanded = 0

        while open_heap:
            _, _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue
            closed.add(cur)
            expanded += 1
            if cur == goal_c:
                return self._reconstruct(parent, cur, to_pos, root=start_c), expanded
            if expanded > cfg.max_nodes:
                return None, expanded

            cur_pos = to_pos(cur)
            par = parent[cur]
            par_pos = to_pos(par)
            for dx, dy, dz in _NEIGHBORS:
                nc = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
                if not self._in_bounds(nc, dims) or nc in closed:
                    continue
                npos = to_pos(nc)
                if infl.point_blocked(npos):
                    continue
                # Path 2: if the neighbour can see cur's parent, hang it straight
                # off the grandparent (any-angle). Otherwise Path 1: grid step.
                if los(par, nc):
                    cand_par, base = par, g_score[par]
                    seg = _dist(par_pos, npos)
                elif not infl.segment_blocked(cur_pos, npos):
                    cand_par, base = cur, g_score[cur]
                    seg = _dist(cur_pos, npos)
                else:
                    continue
                ng = base + seg
                if ng < g_score.get(nc, math.inf):
                    g_score[nc] = ng
                    parent[nc] = cand_par
                    counter += 1
                    heapq.heappush(open_heap,
                                   (ng + _dist(npos, goal_pos), counter, nc))

        return None, expanded

    @staticmethod
    def _reconstruct(parent, cur, to_pos, root=None) -> List[Vec3]:
        cells = [cur]
        while cur in parent and parent[cur] != cur and cur != root:
            cur = parent[cur]
            cells.append(cur)
        cells.reverse()
        return [to_pos(c) for c in cells]

    # ── post-processing ─────────────────────────────────────────────────────────
    @staticmethod
    def _shortcut(pts: List[Vec3], infl: ObstacleField) -> List[Vec3]:
        """Greedy string-pulling: keep the farthest still-visible vertex."""
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        n = len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and infl.segment_blocked(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _nearest_free(self, p: Vec3, infl: ObstacleField, toward: Vec3) -> Optional[Vec3]:
        """Search outward in rings for the nearest collision-free point.

        Biased toward ``toward`` (usually the start) so we exit the obstacle on
        the side the drone is approaching from.
        """
        res = self.cfg.resolution
        dirs = []
        for r in range(1, 13):
            for dx, dy, dz in _NEIGHBORS:
                cand = (p[0] + dx * r * res, p[1] + dy * r * res, p[2] + dz * r * res)
                if not infl.point_blocked(cand):
                    dirs.append(cand)
            if dirs:
                dirs.sort(key=lambda c: _dist(c, toward))
                return dirs[0]
        return None


def densify(waypoints: List[Vec3], spacing: float) -> List[Vec3]:
    """Re-sample a polyline so no gap between consecutive points exceeds
    ``spacing`` — keeps the follower's targets close so the drone hugs the line."""
    if len(waypoints) < 2 or spacing <= 0:
        return [tuple(w) for w in waypoints]
    out: List[Vec3] = [tuple(waypoints[0])]
    for a, b in zip(waypoints, waypoints[1:]):
        n = max(1, int(math.ceil(_dist(a, b) / spacing)))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t,
                        a[2] + (b[2] - a[2]) * t))
    return out


class PathFollower:
    """Flow along a planned polyline without stopping at every vertex.

    The waypoint :class:`~drone_nav.mission.Mission` requires the drone to slow
    to ``arrival_speed`` and dwell at *each* waypoint — at cruise speed it can't,
    so it overshoots and orbits far vertices, swinging metres off the planned
    line and into walls. This follower instead advances to the next vertex as
    soon as the drone is within ``switch_radius`` (no speed/dwell gate), and only
    the final vertex uses the strict arrival test. The result tracks the line
    tightly so corner-cut stays inside the planned clearance band.
    """

    def __init__(self, waypoints, switch_radius, arrival_radius, arrival_speed, hold_time):
        self.wps = [tuple(w) for w in waypoints]
        self.switch_radius = switch_radius
        self.arrival_radius = arrival_radius
        self.arrival_speed = arrival_speed
        self.hold_time = hold_time
        self.index = 0
        self._dwell = 0.0
        self.complete = len(self.wps) == 0

    @property
    def waypoints(self):
        return self.wps

    @property
    def target(self):
        if self.complete:
            return None
        return self.wps[min(self.index, len(self.wps) - 1)]

    def update(self, tlm, dt):
        if self.complete:
            return None
        d = math.dist(tlm.pos, self.wps[self.index])
        if self.index >= len(self.wps) - 1:
            # final vertex: stop here (within radius, slow, sustained)
            if d <= self.arrival_radius and tlm.speed <= self.arrival_speed:
                self._dwell += dt
            else:
                self._dwell = 0.0
            if self._dwell >= self.hold_time:
                self.complete = True
                return None
        elif d <= self.switch_radius:
            self.index += 1
        return self.target
