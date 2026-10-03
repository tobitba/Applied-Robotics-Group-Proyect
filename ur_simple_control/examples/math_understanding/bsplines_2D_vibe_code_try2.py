from smc import load_config
# import numpy as np
#
#
## -----------------------------------------------------------------------------
## B-spline basis (cubic) and derivatives
## -----------------------------------------------------------------------------
# def bspline_basis(i, k, t, knots):
#    if k == 0:
#        return 1.0 if knots[i] <= t < knots[i + 1] else 0.0
#
#    denom1 = knots[i + k] - knots[i]
#    denom2 = knots[i + k + 1] - knots[i + 1]
#
#    term1 = (
#        ((t - knots[i]) / denom1) * bspline_basis(i, k - 1, t, knots)
#        if denom1 > 1e-12
#        else 0.0
#    )
#    term2 = (
#        ((knots[i + k + 1] - t) / denom2) * bspline_basis(i + 1, k - 1, t, knots)
#        if denom2 > 1e-12
#        else 0.0
#    )
#
#    return term1 + term2
#
#
# def bspline_basis_deriv(i, k, t, knots):
#    if k == 0:
#        return 0.0
#
#    denom1 = knots[i + k] - knots[i]
#    denom2 = knots[i + k + 1] - knots[i + 1]
#
#    term1 = (k / denom1) * bspline_basis(i, k - 1, t, knots) if denom1 > 1e-12 else 0.0
#    term2 = (
#        (k / denom2) * bspline_basis(i + 1, k - 1, t, knots) if denom2 > 1e-12 else 0.0
#    )
#
#    return term1 - term2
#
#
# def bspline_basis_second_deriv(i, k, t, knots):
#    if k <= 1:
#        return 0.0
#
#    denom1 = knots[i + k] - knots[i]
#    denom2 = knots[i + k + 1] - knots[i + 1]
#
#    term1 = (
#        (k / denom1) * bspline_basis_deriv(i, k - 1, t, knots)
#        if denom1 > 1e-12
#        else 0.0
#    )
#    term2 = (
#        (k / denom2) * bspline_basis_deriv(i + 1, k - 1, t, knots)
#        if denom2 > 1e-12
#        else 0.0
#    )
#
#    return term1 - term2
#
#
## -----------------------------------------------------------------------------
## B-spline interpolant class
## -----------------------------------------------------------------------------
# class BSpline2DInterpolant:
#    def __init__(self, points):
#        self.data = np.array(points)
#        self.n = len(points)
#        self.k = 3  # cubic B-spline
#        self.knots = self._build_clamped_uniform_knots()
#        self.params = self._chord_length_params()
#        self.ctrl = self._compute_control_points()
#
#    # uniform clamped knot vector (C^2)
#    def _build_clamped_uniform_knots(self):
#        m = self.n
#        k = self.k
#        knots = np.zeros(m + k + 1)
#        knots[k : m + 1] = np.linspace(0, 1, m - k + 1)
#        knots[m + 1 :] = 1.0
#        return knots
#
#    # chord-length parameterization
#    def _chord_length_params(self):
#        d = np.linalg.norm(np.diff(self.data, axis=0), axis=1)
#        alpha = np.concatenate(([0], np.cumsum(d)))
#        alpha /= alpha[-1]
#        return alpha
#
#    # solve A*C = P for control points C
#    def _compute_control_points(self):
#        A = np.zeros((self.n, self.n))
#
#        for i in range(self.n):
#            for j in range(self.n):
#                A[i, j] = bspline_basis(j, self.k, self.params[i], self.knots)
#
#        # Solve for control points
#        Cx = np.linalg.solve(A, self.data[:, 0])
#        Cy = np.linalg.solve(A, self.data[:, 1])
#
#        return np.vstack((Cx, Cy)).T  # Nx2
#
#    # Evaluate curve point
#    def evaluate(self, s):
#        x = 0.0
#        y = 0.0
#        for j in range(self.n):
#            B = bspline_basis(j, self.k, s, self.knots)
#            x += B * self.ctrl[j, 0]
#            y += B * self.ctrl[j, 1]
#        return np.array([x, y])
#
#    # First derivative
#    def derivative(self, s):
#        dx = 0.0
#        dy = 0.0
#        for j in range(self.n):
#            dB = bspline_basis_deriv(j, self.k, s, self.knots)
#            dx += dB * self.ctrl[j, 0]
#            dy += dB * self.ctrl[j, 1]
#        return np.array([dx, dy])
#
#    # Second derivative
#    def second_derivative(self, s):
#        ddx = 0.0
#        ddy = 0.0
#        for j in range(self.n):
#            d2B = bspline_basis_second_deriv(j, self.k, s, self.knots)
#            ddx += d2B * self.ctrl[j, 0]
#            ddy += d2B * self.ctrl[j, 1]
#        return np.array([ddx, ddy])
#
#    # curvature κ(s)
#    def curvature(self, s):
#        d = self.derivative(s)
#        dd = self.second_derivative(s)
#        dx, dy = d
#        ddx, ddy = dd
#        num = abs(dx * ddy - dy * ddx)
#        den = (dx * dx + dy * dy) ** 1.5
#        return num / den if den > 1e-12 else 0.0

# import numpy as np
#
#
## ============================================================
## B-spline basis (Cox–de Boor)
## ============================================================
# def N(i, p, t, knots):
#    if p == 0:
#        return 1.0 if knots[i] <= t < knots[i + 1] else 0.0
#
#    denom1 = knots[i + p] - knots[i]
#    denom2 = knots[i + p + 1] - knots[i + 1]
#
#    term1 = ((t - knots[i]) / denom1) * N(i, p - 1, t, knots) if denom1 > 1e-12 else 0.0
#    term2 = (
#        ((knots[i + p + 1] - t) / denom2) * N(i + 1, p - 1, t, knots)
#        if denom2 > 1e-12
#        else 0.0
#    )
#
#    return term1 + term2
#
#
# def dN(i, p, t, knots):
#    if p == 0:
#        return 0.0
#
#    denom1 = knots[i + p] - knots[i]
#    denom2 = knots[i + p + 1] - knots[i + 1]
#
#    term1 = (p / denom1) * N(i, p - 1, t, knots) if denom1 > 1e-12 else 0.0
#    term2 = (p / denom2) * N(i + 1, p - 1, t, knots) if denom2 > 1e-12 else 0.0
#
#    return term1 - term2
#
#
# def ddN(i, p, t, knots):
#    if p <= 1:
#        return 0.0
#    denom1 = knots[i + p] - knots[i]
#    denom2 = knots[i + p + 1] - knots[i + 1]
#
#    part1 = (p / denom1) * dN(i, p - 1, t, knots) if denom1 > 1e-12 else 0.0
#    part2 = (p / denom2) * dN(i + 1, p - 1, t, knots) if denom2 > 1e-12 else 0.0
#
#    return part1 - part2
#
#
## ============================================================
## Not-a-knot cubic B-spline interpolant
## ============================================================
# class BSpline2DInterpolant:
#    def __init__(self, points):
#        self.points = np.asarray(points)
#        self.n = len(points)
#        self.p = 3  # cubic
#
#        self.knots = self._not_a_knot_knots()
#        self.params = self._chord_length_parameterization()
#        self.ctrl = self._compute_control_points()
#
#    # --------------------------------------------------------
#    # Not-a-knot knot vector (removes 2 interior knots)
#    # --------------------------------------------------------
#    def _not_a_knot_knots(self):
#        n = self.n
#        p = self.p
#
#        # linear knot distribution
#        knots = np.linspace(0, 1, n + p + 1)
#
#        # enforce not-a-knot conditions:
#        # first two spans equal
#        knots[1] = knots[2]
#        # last two spans equal
#        knots[-2] = knots[-3]
#
#        return knots
#
#    # --------------------------------------------------------
#    def _chord_length_parameterization(self):
#        d = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
#        alpha = np.concatenate(([0], np.cumsum(d)))
#        alpha /= alpha[-1]
#        return alpha
#
#    # --------------------------------------------------------
#    def _compute_control_points(self):
#        n = self.n
#        A = np.zeros((n, n))
#
#        for i, s in enumerate(self.params):
#            for j in range(n):
#                A[i, j] = N(j, self.p, s, self.knots)
#
#        # Solve Ax = P
#        Cx = np.linalg.solve(A, self.points[:, 0])
#        Cy = np.linalg.solve(A, self.points[:, 1])
#
#        ctrl = np.vstack((Cx, Cy)).T  # Nx2
#        return ctrl
#
#    # --------------------------------------------------------
#    def evaluate(self, s):
#        x = y = 0.0
#        for j in range(self.n):
#            B = N(j, self.p, s, self.knots)
#            x += B * self.ctrl[j, 0]
#            y += B * self.ctrl[j, 1]
#        return np.array([x, y])
#
#    def derivative(self, s):
#        dx = dy = 0.0
#        for j in range(self.n):
#            dB = dN(j, self.p, s, self.knots)
#            dx += dB * self.ctrl[j, 0]
#            dy += dB * self.ctrl[j, 1]
#        return np.array([dx, dy])
#
#    def second_derivative(self, s):
#        ddx = ddy = 0.0
#        for j in range(self.n):
#            d2B = ddN(j, self.p, s, self.knots)
#            ddx += d2B * self.ctrl[j, 0]
#            ddy += d2B * self.ctrl[j, 1]
#        return np.array([ddx, ddy])
#
#    def curvature(self, s):
#        d = self.derivative(s)
#        dd = self.second_derivative(s)
#        dx, dy = d
#        ddx, ddy = dd
#        num = abs(dx * ddy - dy * ddx)
#        den = (dx * dx + dy * dy) ** 1.5
#        return num / den if den > 1e-12 else 0.0

import numpy as np


class CubicSpline2D:
    def __init__(self, points):
        self.P = np.array(points)
        self.n = len(points)

        # chord-length parameterization
        self.t = np.zeros(self.n)
        self.t[1:] = np.cumsum(np.linalg.norm(np.diff(self.P, axis=0), axis=1))
        self.t /= self.t[-1]

        # compute 2D spline: do x(t) and y(t) separately
        self.spline_x = self._compute_spline(self.P[:, 0])
        self.spline_y = self._compute_spline(self.P[:, 1])

    # ----------------------------------------------------------
    # Compute classical natural cubic spline
    # ----------------------------------------------------------
    def _compute_spline(self, values):
        n = self.n
        t = self.t
        h = np.diff(t)

        # system matrix is always tridiagonal AND nonsingular
        A = np.zeros((n, n))
        rhs = np.zeros(n)

        A[0, 0] = 1
        A[-1, -1] = 1

        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]

            rhs[i] = 6 * (
                (values[i + 1] - values[i]) / h[i]
                - (values[i] - values[i - 1]) / h[i - 1]
            )

        M = np.linalg.solve(A, rhs)  # M are the second derivatives
        return M

    # ----------------------------------------------------------
    # Evaluate spline at parameter s ∈ [0,1]
    # ----------------------------------------------------------
    def evaluate_axis(self, values, M, s):
        # find segment
        i = np.searchsorted(self.t, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.t[i]
        t1 = self.t[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        term1 = a * values[i] + b * values[i + 1]
        term2 = ((a**3 - a) * M[i] + (b**3 - b) * M[i + 1]) * (h**2) / 6

        return term1 + term2

    def evaluate(self, s):
        x = self.evaluate_axis(self.P[:, 0], self.spline_x, s)
        y = self.evaluate_axis(self.P[:, 1], self.spline_y, s)
        return np.array([x, y])

    # ----------------------------------------------------------
    # First derivative
    # ----------------------------------------------------------
    def derivative_axis(self, values, M, s):
        i = np.searchsorted(self.t, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.t[i]
        t1 = self.t[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        dv = (values[i + 1] - values[i]) / h
        dM = (3 * b * b - 1) * M[i + 1] * h / 6 - (3 * a * a - 1) * M[i] * h / 6
        return dv + dM

    def derivative(self, s):
        dx = self.derivative_axis(self.P[:, 0], self.spline_x, s)
        dy = self.derivative_axis(self.P[:, 1], self.spline_y, s)
        return np.array([dx, dy])

    # ----------------------------------------------------------
    # Second derivative
    # ----------------------------------------------------------
    def second_derivative_axis(self, M, s):
        i = np.searchsorted(self.t, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.t[i]
        t1 = self.t[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        return a * M[i] + b * M[i + 1]

    def second_derivative(self, s):
        ddx = self.second_derivative_axis(self.spline_x, s)
        ddy = self.second_derivative_axis(self.spline_y, s)
        return np.array([ddx, ddy])

    # ----------------------------------------------------------
    # Curvature κ(s)
    # ----------------------------------------------------------
    def curvature(self, s):
        d = self.derivative(s)
        dd = self.second_derivative(s)
        dx, dy = d
        ddx, ddy = dd
        num = abs(dx * ddy - dy * ddx)
        den = (dx * dx + dy * dy) ** 1.5
        return num / den if den > 1e-12 else 0.0


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    points = np.array([[0.2, 2.3], [1, 1.8], [2.5, 1], [4.1, 3.2], [6.3, 1.8]])

    spline = CubicSpline2D(points)

    S = np.linspace(0, 1, 300)
    curve = np.array([spline.evaluate(s) for s in S])
    curv = np.array([spline.curvature(s) for s in S])

    plt.figure()
    plt.plot(points[:, 0], points[:, 1], "ro--", label="data points")
    plt.plot(curve[:, 0], curve[:, 1], "b", label="spline")
    plt.legend()
    plt.title("Interpolating C² cubic B-spline")

    plt.plot(S * points[-1][0], curv)
    plt.title("Curvature κ(s)")
    plt.xlabel("s")
    plt.ylabel("curvature")

    plt.show()
