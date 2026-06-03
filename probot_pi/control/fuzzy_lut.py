"""Precomputed lookup tables for the fuzzy blocks — make the supervisor real-time.

skfuzzy's ControlSystemSimulation.compute() is ~25 ms on a Pi, so running two
blocks every tick cannot hold 100 Hz (measured ~20 Hz once telemetry varies and
the input cache stops hitting). Here we sample each block's surface ON A GRID
using skfuzzy itself — so the spec-mandated scikit-fuzzy design is preserved
exactly — cache it to .npz, and at runtime do O(1) bilinear interpolation (~µs).

Build once (tens of seconds), reuse forever. The cache key includes the tuning
ranges, so changing params in bsp/params.py auto-rebuilds. To force a rebuild,
delete probot_pi/control/_lut_cache, or pre-build explicitly with:

    python -m probot_pi.control.fuzzy_lut
"""
import hashlib
import os

import numpy as np

from probot_pi.bsp import params as P


class FuzzyLUT:
    """A 2-D grid sampled over [x0,x1]x[y0,y1] with bilinear lookup (clamped)."""

    def __init__(self, table, x0, x1, y0, y1):
        self.table = np.asarray(table, dtype=np.float64)
        self.nx, self.ny = self.table.shape
        self.x0, self.x1 = float(x0), float(x1)
        self.y0, self.y1 = float(y0), float(y1)
        self.dx = (self.x1 - self.x0) / (self.nx - 1)
        self.dy = (self.y1 - self.y0) / (self.ny - 1)

    def at(self, x, y):
        fx = (x - self.x0) / self.dx
        fy = (y - self.y0) / self.dy
        fx = 0.0 if fx < 0.0 else (self.nx - 1 if fx > self.nx - 1 else fx)
        fy = 0.0 if fy < 0.0 else (self.ny - 1 if fy > self.ny - 1 else fy)
        ix, iy = int(fx), int(fy)
        ix1, iy1 = min(ix + 1, self.nx - 1), min(iy + 1, self.ny - 1)
        tx, ty = fx - ix, fy - iy
        t = self.table
        a = t[ix, iy] * (1.0 - tx) + t[ix1, iy] * tx
        b = t[ix, iy1] * (1.0 - tx) + t[ix1, iy1] * tx
        return float(a * (1.0 - ty) + b * ty)

    def save(self, path):
        np.savez(path, table=self.table,
                 bounds=np.array([self.x0, self.x1, self.y0, self.y1]))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        x0, x1, y0, y1 = d["bounds"]
        return cls(d["table"], x0, x1, y0, y1)


# --------------------------------------------------------------------------- #
# Building / caching                                                          #
# --------------------------------------------------------------------------- #
def _sample(compute_fn, x0, x1, y0, y1, nx, ny, progress=None):
    xs = np.linspace(x0, x1, nx)
    ys = np.linspace(y0, y1, ny)
    table = np.empty((nx, ny), dtype=np.float64)
    total, k = nx * ny, 0
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            table[i, j] = compute_fn(float(x), float(y))
            k += 1
        if progress:
            progress(k, total)
    return table


def _key(*vals):
    return hashlib.md5("|".join(str(v) for v in vals).encode()).hexdigest()[:10]


def _cache_path(name, key):
    d = os.path.join(os.path.dirname(__file__), "_lut_cache")
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, f"{name}_{key}.npz")


def _progress(label):
    def report(k, total):
        pct = 100 * k // total
        print(f"  [LUT] {label}: {pct:3d}%", end="\r", flush=True)
        if k == total:
            print()
    return report


def build_yaw_lut(nx=41, ny=41, verbose=True):
    x0, x1 = -P.YAW_EPSI_RANGE, P.YAW_EPSI_RANGE
    y0, y1 = -P.YAW_RERR_RANGE, P.YAW_RERR_RANGE
    path = _cache_path("yaw", _key("yaw", x0, x1, y0, y1, P.YAW_DW_MAX, nx, ny))
    if os.path.exists(path):
        return FuzzyLUT.load(path)
    from probot_pi.control.fuzzy_yaw import YawFuzzy
    if verbose:
        print(f"[LUT] yaw block: sampling {nx}x{ny} grid via skfuzzy (one-time build)...")
    table = _sample(YawFuzzy().compute, x0, x1, y0, y1, nx, ny,
                    _progress("yaw") if verbose else None)
    lut = FuzzyLUT(table, x0, x1, y0, y1)
    lut.save(path)
    return lut


def build_traction_lut(nx=41, ny=41, verbose=True):
    x0, x1 = 0.0, P.TRAC_SIGMA_MAX
    y0, y1 = 0.0, P.TRAC_ABSR_MAX
    path = _cache_path("trac", _key("trac", x1, y1, P.LAMBDA_MIN, P.LAMBDA_MAX, nx, ny))
    if os.path.exists(path):
        return FuzzyLUT.load(path)
    from probot_pi.control.fuzzy_traction import TractionFuzzy
    if verbose:
        print(f"[LUT] traction block: sampling {nx}x{ny} grid via skfuzzy (one-time build)...")
    table = _sample(TractionFuzzy().compute, x0, x1, y0, y1, nx, ny,
                    _progress("trac") if verbose else None)
    # Affine-normalise so the achievable lambda spans exactly [LAMBDA_MIN, MAX].
    # Centroid defuzz cannot reach a universe endpoint, which otherwise leaves
    # lambda stuck ~0.9 at no-slip (a permanent ~10% speed derate). The raw
    # surface is monotone, so stretching [min,max] -> [0.4,1.0] makes no-slip
    # map to 1.0 and worst-slip to 0.4 while preserving the shape.
    tmin, tmax = float(table.min()), float(table.max())
    if tmax - tmin > 1e-9:
        table = P.LAMBDA_MIN + (table - tmin) * (P.LAMBDA_MAX - P.LAMBDA_MIN) / (tmax - tmin)
    lut = FuzzyLUT(table, x0, x1, y0, y1)
    lut.save(path)
    return lut


if __name__ == "__main__":
    import time
    t0 = time.monotonic()
    build_yaw_lut()
    build_traction_lut()
    cache = os.path.join(os.path.dirname(__file__), "_lut_cache")
    print(f"[LUT] done in {time.monotonic() - t0:.1f}s  (cache: {cache})")
