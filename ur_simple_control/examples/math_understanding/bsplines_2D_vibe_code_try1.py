from smc import load_config
import numpy as np


# ---------------------------------------------------------------------
# B-spline basis functions (cubic)
# ---------------------------------------------------------------------
def bspline_basis(i, k, t, knots):
    """
    Cox–de Boor recursion for B-spline basis.
    i     : basis function index
    k     : degree (3 for cubic)
    t     : evaluation parameter
    knots : knot vector
    """
    if k == 0:
        return 1.0 if knots[i] <= t < knots[i + 1] else 0.0

    denom1 = knots[i + k] - knots[i]
    denom2 = knots[i + k + 1] - knots[i + 1]

    term1 = 0.0
    if denom1 > 1e-12:
        term1 = (t - knots[i]) / denom1 * bspline_basis(i, k - 1, t, knots)

    term2 = 0.0
    if denom2 > 1e-12:
        term2 = (knots[i + k + 1] - t) / denom2 * bspline_basis(i + 1, k - 1, t, knots)

    return term1 + term2


# ---------------------------------------------------------------------
# Derivatives of B-spline basis
# ---------------------------------------------------------------------
def bspline_basis_derivative(i, k, t, knots):
    """
    First derivative of B-spline basis.
    """
    if k == 0:
        return 0.0

    denom1 = knots[i + k] - knots[i]
    denom2 = knots[i + k + 1] - knots[i + 1]

    term1 = 0.0
    if denom1 > 1e-12:
        term1 = k / denom1 * bspline_basis(i, k - 1, t, knots)

    term2 = 0.0
    if denom2 > 1e-12:
        term2 = k / denom2 * bspline_basis(i + 1, k - 1, t, knots)

    return term1 - term2


def bspline_basis_second_derivative(i, k, t, knots):
    """
    Second derivative of B-spline basis (recursively).
    """
    if k <= 1:
        return 0.0

    denom1 = knots[i + k] - knots[i]
    denom2 = knots[i + k + 1] - knots[i + 1]

    term1 = 0.0
    if denom1 > 1e-12:
        term1 = k / denom1 * bspline_basis_derivative(i, k - 1, t, knots)

    term2 = 0.0
    if denom2 > 1e-12:
        term2 = k / denom2 * bspline_basis_derivative(i + 1, k - 1, t, knots)

    return term1 - term2


# ---------------------------------------------------------------------
# Build cubic B-spline
# ---------------------------------------------------------------------
def make_uniform_knot_vector(n_ctrl, degree):
    """Uniform clamped knot vector."""
    n_knots = n_ctrl + degree + 1
    knots = np.zeros(n_knots)
    knots[degree : n_knots - degree] = np.linspace(0, 1, n_knots - 2 * degree)
    knots[n_knots - degree :] = 1.0
    return knots


class BSpline2D:
    def __init__(self, points, degree=3):
        self.points = np.array(points)  # control points Nx2
        self.degree = degree
        self.n_ctrl = len(points)
        self.knots = make_uniform_knot_vector(self.n_ctrl, degree)

    # Evaluate point
    def evaluate(self, s):
        x = 0.0
        y = 0.0
        for i in range(self.n_ctrl):
            B = bspline_basis(i, self.degree, s, self.knots)
            x += B * self.points[i, 0]
            y += B * self.points[i, 1]
        return np.array([x, y])

    # First derivative
    def derivative(self, s):
        dx = 0.0
        dy = 0.0
        for i in range(self.n_ctrl):
            dB = bspline_basis_derivative(i, self.degree, s, self.knots)
            dx += dB * self.points[i, 0]
            dy += dB * self.points[i, 1]
        return np.array([dx, dy])

    # Second derivative
    def second_derivative(self, s):
        ddx = 0.0
        ddy = 0.0
        for i in range(self.n_ctrl):
            d2B = bspline_basis_second_derivative(i, self.degree, s, self.knots)
            ddx += d2B * self.points[i, 0]
            ddy += d2B * self.points[i, 1]
        return np.array([ddx, ddy])

    # Curvature
    def curvature(self, s):
        d = self.derivative(s)
        dd = self.second_derivative(s)
        dx, dy = d
        ddx, ddy = dd
        num = abs(dx * ddy - dy * ddx)
        den = (dx * dx + dy * dy) ** 1.5
        if den < 1e-12:
            return 0.0
        return num / den


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Example control points
    pts = np.array([[0, 0], [1, 2], [3, 1], [4, 3], [6, 0]])

    spline = BSpline2D(pts, degree=3)

    # Sample the curve
    S = np.linspace(0, 1, 200)
    curve = np.array([spline.evaluate(s) for s in S])
    curv = np.array([spline.curvature(s) for s in S])

    # Plot curve
    plt.figure()
    plt.plot(curve[:, 0], curve[:, 1], "b", label="B-spline")
    plt.plot(pts[:, 0], pts[:, 1], "ro--", label="control points")
    plt.legend()
    plt.title("2D cubic B-spline")

    # Plot curvature
    plt.figure()
    plt.plot(S, curv)
    plt.title("Curvature κ(s)")
    plt.xlabel("s")
    plt.ylabel("curvature")

    plt.show()
