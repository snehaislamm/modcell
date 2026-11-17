"""Part-inspection camera model (stands in for a Cognex/Keyence smart camera job).

Image: 0.5 mm/px top view of a module bracket with six fastener holes.
Job:   1) locate the plate (threshold + centroid) -> X/Y offset for robot guidance
       2) count holes inside the plate (connected-component blob count)
       3) pass if 6 holes found and the offset is inside the robot's correction window
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MM_PER_PX = 0.5
SIZE = 96
PLATE = (24, 32)                         # plate half-size in px (48 x 64 px = 24 x 32 mm)
HOLES = [(-12, -16), (-12, 0), (-12, 16), (12, -16), (12, 0), (12, 16)]  # px from plate centre
HOLE_R = 3
MAX_OFFSET_MM = 4.0                      # robot can correct up to +/-4 mm


@dataclass
class PartSpec:
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    missing_hole: int | None = None      # index of an undrilled hole (defect)
    robot_nok: bool = False              # fastening will fail (e.g. cross-threaded insert)


@dataclass
class Result:
    passed: bool
    holes: int
    off_x_mm: float
    off_y_mm: float


def render(spec: PartSpec, rng: np.random.Generator) -> np.ndarray:
    img = np.full((SIZE, SIZE), 0.15)
    cy = SIZE / 2 + spec.dy_mm / MM_PER_PX
    cx = SIZE / 2 + spec.dx_mm / MM_PER_PX
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    plate = (np.abs(yy - cy) <= PLATE[0]) & (np.abs(xx - cx) <= PLATE[1])
    img[plate] = 0.85
    for i, (hy, hx) in enumerate(HOLES):
        if i == spec.missing_hole:
            continue
        img[(yy - (cy + hy)) ** 2 + (xx - (cx + hx)) ** 2 <= HOLE_R ** 2] = 0.15
    img += rng.normal(0, 0.04, img.shape)          # sensor noise
    return img


def _count_blobs(mask: np.ndarray) -> int:
    seen = np.zeros_like(mask, dtype=bool)
    count = 0
    for y, x in zip(*np.nonzero(mask)):
        if seen[y, x]:
            continue
        count += 1
        stack = [(y, x)]
        seen[y, x] = True
        while stack:
            cy, cx = stack.pop()
            for ny, nx in ((cy + 1, cx), (cy - 1, cx), (cy, cx + 1), (cy, cx - 1)):
                if 0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1] and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    stack.append((ny, nx))
    return count


def inspect(img: np.ndarray) -> Result:
    bright = img > 0.5
    ys, xs = np.nonzero(bright)
    y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
    off_y = ((y0 + y1) / 2 - SIZE / 2) * MM_PER_PX
    off_x = ((x0 + x1) / 2 - SIZE / 2) * MM_PER_PX
    roi = ~bright[y0 + 2:y1 - 1, x0 + 2:x1 - 1]   # dark pixels inside the plate = holes
    holes = _count_blobs(roi)
    ok = holes == len(HOLES) and abs(off_x) <= MAX_OFFSET_MM and abs(off_y) <= MAX_OFFSET_MM
    return Result(ok, holes, round(off_x, 2), round(off_y, 2))
