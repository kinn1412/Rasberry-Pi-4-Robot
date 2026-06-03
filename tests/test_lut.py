"""Validate the FuzzyLUT bilinear lookup (numpy only — no skfuzzy needed, fast).
Skips cleanly where numpy is not installed. Run from the project root:

    python tests/test_lut.py

The full skfuzzy build + lambda-endpoint check is exercised separately by
`python -m probot_pi.control.fuzzy_lut` and a --sim run.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import numpy as np
except ImportError:
    print("SKIP: numpy not installed on this host")
    sys.exit(0)

from probot_pi.control.fuzzy_lut import FuzzyLUT


def check(name, cond):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        raise AssertionError(name)


def main():
    print("FuzzyLUT bilinear self-test")
    nx, ny = 41, 41
    x0, x1, y0, y1 = -30.0, 30.0, -120.0, 120.0
    xs = np.linspace(x0, x1, nx)
    ys = np.linspace(y0, y1, ny)
    # a linear surface -> bilinear interpolation must be EXACT
    table = 0.7 + 0.05 * xs[:, None] - 0.011 * ys[None, :]
    lut = FuzzyLUT(table, x0, x1, y0, y1)

    def truth(x, y):
        return 0.7 + 0.05 * x - 0.011 * y

    rng = np.random.default_rng(1)
    err = max(abs(lut.at(float(x), float(y)) - truth(x, y))
              for x, y in rng.uniform([x0, y0], [x1, y1], size=(3000, 2)))
    check(f"linear-surface interp exact (max err {err:.1e})", err < 1e-9)

    gp = max(abs(lut.at(float(xs[i]), float(ys[j])) - table[i, j])
             for i in range(nx) for j in range(ny))
    check(f"grid points exact (max err {gp:.1e})", gp < 1e-12)

    check("clamp below x -> edge", abs(lut.at(-1e6, 0.0) - lut.at(x0, 0.0)) < 1e-12)
    check("clamp above y -> edge", abs(lut.at(0.0, 1e6) - lut.at(0.0, y1)) < 1e-12)

    const = FuzzyLUT(np.full((nx, ny), 0.9), x0, x1, y0, y1)
    check("constant surface stays constant", abs(const.at(3.3, -77.0) - 0.9) < 1e-12)

    # save/load round-trip
    import tempfile
    p = os.path.join(tempfile.gettempdir(), "probot_lut_test.npz")
    lut.save(p)
    rl = FuzzyLUT.load(p)
    check("save/load round-trip", abs(rl.at(7.0, -33.0) - lut.at(7.0, -33.0)) < 1e-12)
    os.remove(p)

    print("\nALL PASS")


if __name__ == "__main__":
    main()
