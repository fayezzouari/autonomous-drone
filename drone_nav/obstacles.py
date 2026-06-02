"""Obstacle geometry parsed from the ``drone/obs`` topic.

The sim publishes ``{"obstacles": [ … ]}``, each box given by its centre and its
**half**-extents (current convention)::

    {"cx":…, "cy":…, "cz":…, "hw":<half-width>, "hd":<half-depth>, "hh":<half-height>}

The older full-extent shape ``{"c":[x,y,z], "w":…, "h":…, "t":…}`` is still
accepted as a fallback (those are halved on parse; thickness ``t`` == depth).

We model every obstacle as an **axis-aligned box** (AABB) in the world frame.
The mapping of the three extents to world axes is configurable via the ``axes``
string — 3 chars naming the extent on X, Y, Z (``w``=width, ``t``/``d``=depth,
``h``=height). The default ``"wdh"`` matches the Blender publisher, which builds
box vertices as ``(±hw, ±hd, ±hh)``:

    width  (hw) → extent along world X
    depth  (hd) → extent along world Y
    height (hh) → extent along world Z   (Z is up, matching telemetry)

If a message also carries a heading (``yaw`` / ``rot``, radians about Z) the box
is widened to the smallest axis-aligned box that still encloses the rotated
footprint — so the planner can stay AABB-only without ever *under*-approximating
a tilted wall.

The drone is not a point, so :meth:`ObstacleField.inflate` grows every box by a
clearance radius (the drone's bounding radius + a safety margin). Growing the
half-extents by ``r`` on each axis is the Minkowski sum of the box with a cube of
side ``2r`` — a conservative superset of the box ⊕ sphere, so a path that clears
the inflated boxes clears the real drone with margin to spare.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

# Default mapping of the box extents onto world (X, Y, Z): a 3-char string
# naming the extent assigned to each axis (w=width, t/d=depth, h=height).
# The Blender publisher builds box verts as (±hw, ±hd, ±hh), i.e. hw→X, hd→Y,
# hh→Z, so the matching mapping is "wdh".
DEFAULT_AXES = "wdh"   # X = width (hw), Y = depth (hd), Z = height (hh)

# Centre axes negated on parse to map the source frame onto the nav world frame.
# The publisher's centres are already in the telemetry world frame, so no flip.
DEFAULT_FLIP = ""


def _as_xyz(c) -> Vec3:
    """Accept a [x,y,z] list/tuple or a {'x':,'y':,'z':} dict as a centre."""
    if isinstance(c, dict):
        return (float(c.get("x", 0.0)), float(c.get("y", 0.0)), float(c.get("z", 0.0)))
    if isinstance(c, (list, tuple)) and len(c) >= 3:
        return (float(c[0]), float(c[1]), float(c[2]))
    raise ValueError(f"obstacle centre must be [x,y,z] or {{x,y,z}}, got {c!r}")


def _apply_flip(c: Vec3, flip: str) -> Vec3:
    """Negate the centre coordinates named in ``flip`` (any of 'x','y','z')."""
    x, y, z = c
    if "x" in flip:
        x = -x
    if "y" in flip:
        y = -y
    if "z" in flip:
        z = -z
    return (x, y, z)


@dataclass(frozen=True)
class Box:
    """Axis-aligned box: centre ``(cx,cy,cz)`` and half-extents ``(hx,hy,hz)``."""

    cx: float
    cy: float
    cz: float
    hx: float
    hy: float
    hz: float

    # ── construction ────────────────────────────────────────────────────────
    @classmethod
    def from_msg(cls, d: dict, axes: str = DEFAULT_AXES, flip: str = DEFAULT_FLIP) -> "Box":
        """Build a box from one ``drone/obs`` entry.

        Accepts the current half-extent shape ``{cx,cy,cz, hw,hd,hh[, yaw]}`` and
        the older full-extent shape ``{c,[w,h,t][, yaw]}`` (halved on parse).
        ``axes`` (chars w/t/d/h) names which extent lands on X, Y, Z; ``flip``
        names centre axes to negate (source→world).
        """
        def num(*keys, default=0.0):
            for k in keys:
                if k in d:
                    return float(d[k])
            return default

        # ── centre: explicit cx/cy/cz, else a `c`/center array ────────────────
        if "cx" in d or "cy" in d or "cz" in d:
            centre = (num("cx"), num("cy"), num("cz"))
        else:
            centre = _as_xyz(d.get("c", d.get("center", d.get("centre", [0, 0, 0]))))
        cx, cy, cz = _apply_flip(centre, flip)

        # ── half-extents: prefer explicit hw/hd/hh, else halve full w/t/h ─────
        def half(half_keys, full_keys):
            for k in half_keys:
                if k in d:
                    return abs(float(d[k]))
            for k in full_keys:
                if k in d:
                    return abs(float(d[k])) / 2.0
            return 0.0

        dims = {
            "w": half(("hw",), ("w", "width")),
            "t": half(("hd",), ("t", "thickness", "d", "depth")),
            "h": half(("hh",), ("h", "height")),
        }
        dims["d"] = dims["t"]  # 'd' (depth) is a synonym for 't' (thickness)
        hx, hy, hz = dims[axes[0]], dims[axes[1]], dims[axes[2]]

        yaw = num("yaw", "rot", "heading")
        if abs(yaw) > 1e-9:
            # Smallest AABB enclosing the horizontal footprint rotated by yaw.
            ca, sa = abs(math.cos(yaw)), abs(math.sin(yaw))
            hx, hy = hx * ca + hy * sa, hx * sa + hy * ca
        return cls(cx, cy, cz, hx, hy, hz)

    # ── queries ──────────────────────────────────────────────────────────────
    @property
    def lo(self) -> Vec3:
        return (self.cx - self.hx, self.cy - self.hy, self.cz - self.hz)

    @property
    def hi(self) -> Vec3:
        return (self.cx + self.hx, self.cy + self.hy, self.cz + self.hz)

    def inflated(self, r: float) -> "Box":
        """Grow the box by ``r`` on every axis (drone-size Minkowski padding)."""
        return Box(self.cx, self.cy, self.cz, self.hx + r, self.hy + r, self.hz + r)

    def contains(self, p: Sequence[float], eps: float = 0.0) -> bool:
        return (abs(p[0] - self.cx) <= self.hx + eps
                and abs(p[1] - self.cy) <= self.hy + eps
                and abs(p[2] - self.cz) <= self.hz + eps)

    def intersects_segment(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """True if the segment a→b touches the box (slab / ray-AABB method)."""
        lo, hi = self.lo, self.hi
        t0, t1 = 0.0, 1.0
        for i in range(3):
            d = b[i] - a[i]
            if abs(d) < 1e-12:
                # Segment parallel to this slab: must already lie within it.
                if a[i] < lo[i] or a[i] > hi[i]:
                    return False
                continue
            inv = 1.0 / d
            tn = (lo[i] - a[i]) * inv
            tf = (hi[i] - a[i]) * inv
            if tn > tf:
                tn, tf = tf, tn
            if tn > t0:
                t0 = tn
            if tf < t1:
                t1 = tf
            if t0 > t1:
                return False
        return True


@dataclass
class ObstacleField:
    """A set of box obstacles plus collision queries used by the planner."""

    boxes: List[Box]

    # ── parsing ────────────────────────────────────────────────────────────────
    @classmethod
    def from_payload(cls, payload, axes: str = DEFAULT_AXES,
                     flip: str = DEFAULT_FLIP) -> "ObstacleField":
        """Parse a ``drone/obs`` JSON payload (a list, or ``{"obstacles": [...]}``)."""
        if isinstance(payload, (bytes, bytearray)):
            payload = payload.decode("utf-8")
        if isinstance(payload, str):
            payload = json.loads(payload)
        return cls.from_list(payload, axes, flip)

    @classmethod
    def from_list(cls, data, axes: str = DEFAULT_AXES,
                  flip: str = DEFAULT_FLIP) -> "ObstacleField":
        if isinstance(data, dict):
            data = data.get("obstacles", data.get("obs", []))
        if not isinstance(data, (list, tuple)):
            raise ValueError("obstacle payload must be a list of boxes")
        boxes = [Box.from_msg(d, axes, flip) for d in data if isinstance(d, dict)]
        return cls(boxes)

    # ── transforms / queries ────────────────────────────────────────────────────
    def inflate(self, r: float) -> "ObstacleField":
        return ObstacleField([b.inflated(r) for b in self.boxes])

    def point_blocked(self, p: Sequence[float]) -> bool:
        return any(b.contains(p) for b in self.boxes)

    def segment_blocked(self, a: Sequence[float], b: Sequence[float]) -> bool:
        # Broad-phase: skip boxes whose AABB can't overlap the segment's AABB.
        amin = (min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2]))
        amax = (max(a[0], b[0]), max(a[1], b[1]), max(a[2], b[2]))
        for box in self.boxes:
            lo, hi = box.lo, box.hi
            if (amax[0] < lo[0] or amin[0] > hi[0]
                    or amax[1] < lo[1] or amin[1] > hi[1]
                    or amax[2] < lo[2] or amin[2] > hi[2]):
                continue
            if box.intersects_segment(a, b):
                return True
        return False

    def bounds(self) -> Optional[Tuple[Vec3, Vec3]]:
        """World AABB enclosing every obstacle (None when empty)."""
        if not self.boxes:
            return None
        los = [b.lo for b in self.boxes]
        his = [b.hi for b in self.boxes]
        lo = (min(l[0] for l in los), min(l[1] for l in los), min(l[2] for l in los))
        hi = (max(h[0] for h in his), max(h[1] for h in his), max(h[2] for h in his))
        return lo, hi

    def __len__(self) -> int:
        return len(self.boxes)
