# TODO: refactor bsplines2d here
# should go something like this.
# you need 1d bsplines anyway.
# the 2d bsplines should be done as bezier curves

import numpy as np
from pinocchio import SE3, rpy
from smc.motion_planning.interpolation.interpolator import (
    AbstractInterpolator,
    AbstractSpline,
)
from smc.bookkeeping.types import (
    Point2D,
    Points2D,
    CartesianReference,
)

# TODO: write a class for just a cubic.
# bezier cubic is different, and a different class,
# even though they are completely equivalent at the end of the day.
# in the end switch everything useful to bezier as it's generalizable
# for any dimension. this 2D cubic spline implementation can stay
# as an educational example now that it's written already.


# NOTE: keep this in this file. use bezier version everywhere else
class Cubic1D(AbstractInterpolator):
    def __init__(
        self, p_0: PointT, v_0: VelocityT, p_1: PointT, v_1: VelocityT, t: float
    ): ...


class NaturalCubicSpline1D(AbstractSpline):
    def __init__(
        self,
        points: np.ndarray,
        init_vel: float,
        final_vel: float,
    ):
        super().__init__(points, init_vel, final_vel)

    # ----------------------------------------------------------
    # Compute classical natural cubic spline
    # ----------------------------------------------------------
    def computeInterpolation(self):
        n = self.n

        # TODO: use init and final vel
        # system matrix is always tridiagonal AND nonsingular
        A = np.zeros((n, n))
        rhs = np.zeros(n)

        A[0, 0] = 1
        A[-1, -1] = 1

        for i in range(1, n - 1):
            A[i, i - 1] = self.hs[i - 1]
            A[i, i] = 2 * (self.hs[i - 1] + self.hs[i])
            A[i, i + 1] = self.hs[i]

            rhs[i] = 6 * (
                (self.P[i + 1] - self.P[i]) / self.hs[i]
                - (self.P[i] - self.P[i - 1]) / self.hs[i - 1]
            )

        self.M = np.linalg.solve(A, rhs)  # M are the second derivatives

    # TODO: obviously refactor so this ins't overriden.
    # i'm keepin the original for easier verification later
    def getSplineIndexAndParameter(self, s: float) -> tuple[int, float]:
        i = np.searchsorted(self.ss, s) - 1
        i = np.clip(i, 0, self.n - 2)
        return i, np.nan

    # TODO: need to validate all indeces match
    # the original horrendous implementation
    def getPoint(self, s: float) -> float:
        # TODO: but for sure i'm supposed to use u instead of a,b bullshit??
        i, u = self.getSplineIndexAndParameter(s)

        a = (self.ss[i + 1] - s) / self.hs[i]
        b = (s - self.ss[i]) / self.hs[i]

        term1 = a * self.P[i] + b * self.P[i + 1]
        term2 = (
            ((a**3 - a) * self.M[i] + (b**3 - b) * self.M[i + 1])
            * (self.hs[i] ** 2)
            / 6
        )
        return term1 + term2

    def getVelocity(self, s: float) -> float:
        i, u = self.getSplineIndexAndParameter(s)

        a = (self.ss[i + 1] - s) / self.hs[i]
        b = (s - self.ss[i]) / self.hs[i]

        dv = (self.P[i + 1] - self.P[i]) / self.hs[i]
        dM = (3 * b * b - 1) * self.M[i + 1] * self.hs[i] / 6 - (
            3 * a * a - 1
        ) * self.M[i] * self.hs[i] / 6
        return dv + dM

    def getAcceleration(self, s: float) -> float:
        # NOTE: shitty hacks, i can't be bothered
        if s <= 0.0:
            s = 1e-5
        # equal on a float lmao
        if s == 1.0:
            s = 1 - 1e-5
        i = np.searchsorted(self.ss, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.ss[i]
        t1 = self.ss[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        return a * self.M[i] + b * self.M[i + 1]


class ApproximatingCubicSpline1D(AbstractSpline):
    def __init__(
        self,
        points: np.ndarray,
        mu: float,
        init_vel: float,
        final_vel: float,
        radii=None,
    ):
        assert mu > 0.0
        self.lambda_straightening: float = (1 - mu) / (6 * mu)
        self.slack_points: np.ndarray
        super().__init__(points, init_vel, final_vel)

    def computeInterpolation(self):
        """
        minimizing straightening is equivalent to minizing the second derivative
        along the path parameter.
        the relaxation is made by introducing slack between the points to pass through
        and where the actual curve will be.
        there is a tradeoff paraterm \mu \in [0,1] which explicitly determines
        how much slack is provided.

        Computes cubic smoothing spline second derivatives.
        values: y-values at parameter points self.s
        """
        n = self.n
        A = np.zeros((n, n))
        C = np.zeros((n, n))

        # NOTE: weights can be set differently, but let's keep it simple for now
        w_flat = np.arange(1, n + 1, dtype=np.float64) / 10
        # w_flat = np.ones(n,dtype=np.float64)
        w_inv_flat = w_flat**-1
        # NOTE: have to have this for exact fit at boundaries
        w_inv_flat[0] = 0.0
        w_inv_flat[-1] = 0.0
        W_inv = np.diag(w_inv_flat)

        A[0, 0] = 2 * self.hs[0]
        A[0, 1] = self.hs[0]
        A[-1, -2] = self.hs[-1]
        A[-1, -1] = 2 * self.hs[-1]
        A[-1, -1] = 1
        C[0, 0] = -6 / self.hs[0]
        C[0, 1] = 6 / self.hs[0]

        for i in range(1, n - 1):
            A[i, i - 1] = self.hs[i - 1]
            A[i, i] = 2 * (self.hs[i - 1] + self.hs[i])
            A[i, i + 1] = self.hs[i]

            if i < n - 1:
                C[i, i - 1] = 6 / self.hs[i - 1]
                C[i, i] = -(6 / self.hs[i - 1] + 6 / self.hs[i])
                C[i, i + 1] = 6 / self.hs[i]
        C[-1][-2] = 6 / self.hs[-1]
        C[-1][-1] = -6 / self.hs[-1]

        # again solving for second derivatives
        # M = np.linalg.solve(A + self.lambda_straightening * C @ W_inv @ C.T, C @ values)
        rhs = C @ self.P
        rhs[0] = rhs[0] - 6 * self.init_vel
        rhs[-1] = 6 * self.final_vel - rhs[-1]
        self.M = np.linalg.solve(A + self.lambda_straightening * C @ W_inv @ C.T, rhs)

        self.slack_points = self.P - self.lambda_straightening * W_inv @ C.T @ self.M

    # TODO: obviously refactor so this ins't overriden.
    # i'm keepin the original for easier verification later
    def getSplineIndexAndParameter(self, s: float) -> tuple[int, float]:
        i = np.searchsorted(self.ss, s) - 1
        i = np.clip(i, 0, self.n - 2)
        return i, np.nan

    def getPoint(self, s: float) -> float:
        i, u = self.getSplineIndexAndParameter(s)
        a = (self.ss[i + 1] - s) / self.hs[i]
        b = (s - self.ss[i]) / self.hs[i]

        term1 = a * self.slack_points[i] + b * self.slack_points[i + 1]
        term2 = (
            ((a**3 - a) * self.M[i] + (b**3 - b) * self.M[i + 1])
            * (self.hs[i] ** 2)
            / 6
        )

        return term1 + term2

    def getVelocity(self, s: float) -> float:
        i, u = self.getSplineIndexAndParameter(s)

        a = (self.ss[i + 1] - s) / self.hs[i]
        b = (s - self.ss[i]) / self.hs[i]

        dv = (self.slack_points[i + 1] - self.slack_points[i]) / self.hs[i]
        dM = (3 * b * b - 1) * self.M[i + 1] * self.hs[i] / 6 - (
            3 * a * a - 1
        ) * self.M[i] * self.hs[i] / 6
        return dv + dM

    def getAcceleration(self, s: float) -> float:
        # NOTE: shitty hacks, i can't be bothered
        if s <= 0.0:
            s = 1e-5
        # equal on a float lmao
        if s == 1.0:
            s = 1 - 1e-5
        i = np.searchsorted(self.ss, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.ss[i]
        t1 = self.ss[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        return a * self.M[i] + b * self.M[i + 1]


# i always want both axis
# i always use all the data, save all of it
# TODO: just get rid of the natural spline, no?
class CubicSpline2D(AbstractSpline[Point2D]):
    def __init__(
        self,
        points: Points2D,
        mu: float,
        init_vel: np.ndarray,
        final_vel: np.ndarray,
        radii=None,
    ):
        self.P = points
        self.n = len(points)
        # chord-length parameterization
        self.t = np.zeros(self.n)
        self.t[1:] = np.cumsum(np.linalg.norm(np.diff(self.P, axis=0), axis=1))
        self.t /= self.t[-1]

        if radii is None:
            self.radii = None
            self.computeInterpolation(mu, init_vel, final_vel)
        else:
            self.radii = radii
            self.fitWithMaximumSlack(init_vel, final_vel, radii)

    def computeInterpolation(self, mu: float, init_vel: Point2D, final_vel: Point2D):
        if mu > 0.0:
            self.lambda_straightening = (1 - mu) / (6 * mu)
        else:
            self.lambda_straightening = 0.0
            self.slack_points_x = None
            self.slack_points_y = None

        # compute 2D spline: do x(t) and y(t) separately
        if self.lambda_straightening > 0.0:
            self.spline_x = ApproximatingCubicSpline1D(
                self.P[:, 0], mu, init_vel[0], final_vel[0]
            )
            self.spline_y = ApproximatingCubicSpline1D(
                self.P[:, 1], mu, init_vel[1], final_vel[1]
            )
        else:
            self.spline_x = NaturalCubicSpline1D(self.P[:, 0], 0.0, 0.0)
            self.spline_y = NaturalCubicSpline1D(self.P[:, 1], 0.0, 0.0)
        # for lifting to SE3

    def fitWithMaximumSlack(
        self, init_vel: Point2D, final_vel: Point2D, radii: np.ndarray
    ):
        assert self.spline_x is ApproximatingCubicSpline1D
        assert self.spline_y is ApproximatingCubicSpline1D
        maximum_slack_found = False
        mu = 0.99999
        delta_mu = 0.00001
        # NOTE: this obviously isn't line search.
        # i ain't got time to learn line search now (despite how simple it is).
        # so we're just gonna do a stupid step
        distances = np.zeros(self.n)
        while not maximum_slack_found:
            self.lambda_straightening = (1 - mu) / (6 * mu)

            self.computeInterpolation(mu, init_vel, final_vel)
            slack_points = np.hstack(
                (
                    self.spline_x.slack_points.reshape((-1, 1)),
                    self.spline_y.slack_points.reshape((-1, 1)),
                )
            )
            distances = np.linalg.norm(slack_points - self.P, axis=1)
            checkd = radii - distances
            check = checkd < 0.0
            if check.any():
                print("most aggressive fit at mu", mu)
                break

            # check slack
            mu -= delta_mu

    def setSE3LiftingParams(self, rotx, height):
        self.rotx = rotx
        self.height = height

    def getPoint(self, s: float) -> Point2D:
        x = self.spline_x.getPoint(s)
        y = self.spline_y.getPoint(s)
        return np.array([x, y])

    # ----------------------------------------------------------
    # First derivative
    # ----------------------------------------------------------

    def getVelocity(self, s: float) -> Point2D:
        dx = self.spline_x.getVelocity(s)
        dy = self.spline_y.getVelocity(s)
        return np.array([dx, dy])

    # ----------------------------------------------------------
    # Second derivative
    # ----------------------------------------------------------

    def getAcceleration(self, s: float) -> Point2D:
        ddx = self.spline_x.getAcceleration(s)
        ddy = self.spline_y.getAcceleration(s)
        return np.array([ddx, ddy])

    # ----------------------------------------------------------
    # Curvature κ(s)
    # ----------------------------------------------------------
    def curvature(self, s: float) -> float:
        # # NOTE: should i need parametrization w/ arclength
        # # to have T_s defined properly?
        # # does it matter if i normalize?
        # # --> only one way to find out
        # v_s = self.derivative(s)
        # T_s = v_s / np.linalg.norm(v_s)
        # # rotate T_s
        # N_s = np.array([-T_s[1], T_s[0]])
        # a_s = self.second_derivative(s)
        # # not correct
        # # --> need to have parametrization of whole spline
        # # via arclength so that i can compute as i should
        # # who says  a_s is unit if v is unit?
        # # that's the whole point with curvature
        # # T_dot_s = a_s / np.linalg.norm(a_s)
        # # shitty hack - > also incorrect
        # v_s_plus = self.derivative(s + 1e-3)
        # T_s_plus = v_s_plus / np.linalg.norm(v_s_plus)
        # T_dot_s = (T_s_plus - T_s) * 1e3
        # # print(T_dot_s)
        # # print(T_s_plus - T_s)
        # # curvature = <T_dot_s, N_s>
        # kappa = T_dot_s.T @ N_s
        # # print("=" * 20)
        # # print("v_s", v_s)
        # # print("T_s:", T_s)
        # # print("a_s", a_s)
        # # print("T_dot_s:",T_dot_s)
        # # print("kappa", kappa)

        # chatgpt again
        dx, dy = self.getVelocity(s)
        ddx, ddy = self.getAcceleration(s)
        denom = (dx * dx + dy * dy) ** 1.5
        if denom < 1e-12:
            kappa = 0.0
        else:
            kappa = (dx * ddy - dy * ddx) / denom

        return kappa

    def getSE2FrenetInduced(self, s):
        T_s = self.getVelocity(s)
        T_sn = T_s / np.linalg.norm(T_s)
        R = np.array([[T_sn[0], -T_sn[1]], [T_sn[1], T_sn[0]]])
        p = self.getPoint(s)
        # T_2 = np.zeros((3,3))
        # T_2[:2,:2] = R
        # T_2[:2,2] = self.evaluate(s)
        # T_2[2,2] = 1.0
        return R, p

    @staticmethod
    def liftSE2ToSE3(R_2, p_2, rotx, height):
        T = SE3.Identity()
        R3 = np.zeros((3, 3))
        R3[:2, :2] = R_2
        R3[2, 2] = 1.0
        T.rotation = R3 @ rpy.rpyToMatrix(rotx, 0.0, 0.0)
        T.translation[:2] = p_2
        T.translation[2] = height
        return T

    # twist in the induced frenet frame
    def getCurrentSE3Ref(self, s) -> tuple[SE3, np.ndarray]:
        R2, p2 = self.getSE2FrenetInduced(s)
        T_s = self.liftSE2ToSE3(R2, p2, self.rotx, self.height)
        v2 = self.derivative(s)
        V_s = np.zeros(6)
        v_norm = np.linalg.norm(v2)
        V_s[0] = v_norm
        # lateral velocity is zero by construction
        V_s[5] = self.curvature(s) * v_norm
        return T_s, V_s

    def getReference(self, s: float) -> CartesianReference:
        T_s, V_s = self.getCurrentSE3Ref(s)
        # TODO: figure out what the acceleration is actually supposed to be
        # a = self.getAcceleration(s)
        A_s = np.zeros(6)
        return T_s, V_s, A_s

    def getClosestCircles(self, s: float):
        assert self.radii is not None
        t = np.searchsorted(self.t, s)
        return [(self.P[t], self.radii[t])]
        # if s > t:
        #    pt1_ind = t
        #    pt2_ind = t + 1
        # if s < t:
        #    pt1_ind = t - 1
        #    pt2_ind = t
        # if t == 0:
        #    pt1_ind = 1
        #    pt2_ind = 2
        # return [
        #    (self.P[pt1_ind], self.radii[pt1_ind]),
        #    (
        #        self.P[pt2_ind],
        #        self.radii[pt2_ind],
        #    ),
        # ]


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # points = np.array([[0.2, 2.3], [1, 1.8], [2.5, 1], [4.1, 3.2], [6.3, 1.8]])
    N = 10
    points = np.zeros((N, 2))
    for i in range(1, N):
        dx = np.random.random() / 5
        dy = (np.random.random() - 0.5) / 5
        points[i, 0] += points[i - 1, 0] + dx
        points[i, 1] += points[i - 1, 1] + dy

    init_vel = np.array([0.0, 0.0])
    final_vel = np.array([0.0, 0.0])
    spline1 = CubicSpline2D(points, 0.0, init_vel, final_vel)
    spline2 = CubicSpline2D(points, 0.9999, init_vel, final_vel)
    # spline3 = CubicSpline2D(points, 1.5)

    S = np.linspace(0, 1, 300)
    curve1 = np.array([spline1.evaluate(s) for s in S])
    curve2 = np.array([spline2.evaluate(s) for s in S])
    # curve3 = np.array([spline3.evaluate(s) for s in S])

    plt.figure()
    plt.plot(points[:, 0], points[:, 1], "ro--", label="data points")
    plt.plot(curve1[:, 0], curve1[:, 1], "b", label="natural spline")
    plt.plot(curve2[:, 0], curve2[:, 1], "g", label="approx spline")
    # plt.plot(curve3[:, 0], curve3[:, 1], "b", label="spline")
    plt.legend()
    plt.title("Smoothening C2 interpolation via cubics")

    # curv = np.array([spline1.curvature(s) for s in S])
    # plt.plot(S * points[-1][0], curv)
    # plt.title("Curvature κ(s)")
    # plt.xlabel("s")
    # plt.ylabel("curvature")

    plt.show()

    # ----------------------------------------------------------
    # Evaluate spline at parameter s ∈ [0,1]
    # ----------------------------------------------------------
    def evaluate_axis(self, values, slack_points, M, s):
        # find segment
        i = np.searchsorted(self.t, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.t[i]
        t1 = self.t[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        if self.lambda_straightening > 0.0:
            term1 = a * slack_points[i] + b * slack_points[i + 1]
        else:
            term1 = a * values[i] + b * values[i + 1]
        term2 = ((a**3 - a) * M[i] + (b**3 - b) * M[i + 1]) * (h**2) / 6

        return term1 + term2
