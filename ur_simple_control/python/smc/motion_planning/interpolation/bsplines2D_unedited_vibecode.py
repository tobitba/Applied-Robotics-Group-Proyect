import numpy as np
from pinocchio import SE3, rpy
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
from smc.bookkeeping.types import Points2D, CartesianReference


# TODO: refactor this
# TODO: refactor this
# TODO: refactor this
# TODO: refactor this
# TODO: refactor this
# i always want both axis
# i always use all the data, save all of it
class CubicSpline2D(AbstractInterpolator):
    def __init__(
        self,
        points: Points2D,
        mu: float,
        init_vel: np.ndarray,
        final_vel: np.ndarray,
        radii=None,
    ):
        if radii is None:
            self.initialize(points, mu, init_vel, final_vel)
            self.radii = None
        else:
            self.fitWithMaximumSlack(points, init_vel, final_vel, radii)
            self.radii = radii

    def computeInterpolation(self):
        pass

    def initialize(self, points: Points2D, mu, init_vel, final_vel):
        self.P = points
        self.n = len(points)
        if mu > 0.0:
            self.lambda_straightening = (1 - mu) / (6 * mu)
        else:
            self.lambda_straightening = 0.0
            self.slack_points_x = None
            self.slack_points_y = None

        # chord-length parameterization
        self.t = np.zeros(self.n)
        self.t[1:] = np.cumsum(np.linalg.norm(np.diff(self.P, axis=0), axis=1))
        self.t /= self.t[-1]

        # compute 2D spline: do x(t) and y(t) separately
        if self.lambda_straightening > 0.0:
            self.spline_x, self.slack_points_x = (
                self._compute_spline_straightening_tradeoff(
                    self.P[:, 0], init_vel[0], final_vel[0]
                )
            )
            self.spline_y, self.slack_points_y = (
                self._compute_spline_straightening_tradeoff(
                    self.P[:, 1], init_vel[1], final_vel[1]
                )
            )
        else:
            self.spline_x = self._compute_natural_spline(self.P[:, 0])
            self.spline_y = self._compute_natural_spline(self.P[:, 1])
        # for lifting to SE3

    def fitWithMaximumSlack(self, points: Points2D, init_vel, final_vel, radii):
        self.P = points
        self.n = len(points)
        # chord-length parameterization
        self.t = np.zeros(self.n)
        self.t[1:] = np.cumsum(np.linalg.norm(np.diff(self.P, axis=0), axis=1))
        self.t /= self.t[-1]
        maximum_slack_found = False
        mu = 0.99999
        delta_mu = 0.00001
        # NOTE: this obviously isn't line search.
        # i ain't got time to learn line search now (despite how simple it is).
        # so we're just gonna do a stupid step
        distances = np.zeros(self.n)
        while not maximum_slack_found:
            self.lambda_straightening = (1 - mu) / (6 * mu)

            self.spline_x, self.slack_points_x = (
                self._compute_spline_straightening_tradeoff(
                    self.P[:, 0], init_vel[0], final_vel[0]
                )
            )
            self.spline_y, self.slack_points_y = (
                self._compute_spline_straightening_tradeoff(
                    self.P[:, 1], init_vel[1], final_vel[1]
                )
            )
            slack_points = np.hstack(
                (
                    self.slack_points_x.reshape((-1, 1)),
                    self.slack_points_y.reshape((-1, 1)),
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

    # ----------------------------------------------------------
    # Compute classical natural cubic spline
    # ----------------------------------------------------------
    def _compute_natural_spline(self, values):
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

        # print("natural")
        # print("=" * 20)
        # print(A.round(3))
        M = np.linalg.solve(A, rhs)  # M are the second derivatives

        self.M = M
        return M

    def _compute_spline_straightening_tradeoff(self, values, init_vel, final_vel):
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
        t = self.t
        h = np.diff(t)
        A = np.zeros((n, n))
        C = np.zeros((n, n))

        # NOTE: weights can be set differently, but let's keep it simple for now
        w_flat = np.arange(1, n + 1, dtype=np.float64) / 10
        # w_flat = np.ones(n,dtype=np.float64)
        w_inv_flat = w_flat**-1
        # NOTE: have to have this for exact fit at boundaries
        w_inv_flat[0] = 0.0
        w_inv_flat[-1] = 0.0
        W = np.diag(w_flat)
        W_inv = np.diag(w_inv_flat)

        A[0, 0] = 2 * h[0]
        A[0, 1] = h[0]
        A[-1, -2] = h[-1]
        A[-1, -1] = 2 * h[-1]
        A[-1, -1] = 1
        C[0, 0] = -6 / h[0]
        C[0, 1] = 6 / h[0]

        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]

            if i < n - 1:
                C[i, i - 1] = 6 / h[i - 1]
                C[i, i] = -(6 / h[i - 1] + 6 / h[i])
                C[i, i + 1] = 6 / h[i]
        C[-1][-2] = 6 / h[-1]
        C[-1][-1] = -6 / h[-1]
        # TODO: remove, don't need this
        self.A = A
        self.C = C

        # again solving for second derivatives
        # M = np.linalg.solve(A + self.lambda_straightening * C @ W_inv @ C.T, C @ values)
        rhs = C @ values
        rhs[0] = rhs[0] - 6 * init_vel
        rhs[-1] = 6 * final_vel - rhs[-1]
        M = np.linalg.solve(A + self.lambda_straightening * C @ W_inv @ C.T, rhs)

        slack_points = values - self.lambda_straightening * W_inv @ C.T @ M
        self.M = M

        # print("=" * 20)
        # print(M)
        # print(A.round(3))
        # print("-" * 20)
        # print(C.round(3))
        #        for i in range(n):
        #            print(values[i], slack_points[i])

        return M, slack_points

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

    def evaluate_axis_y(self, s):
        # find segment
        i = np.searchsorted(self.t, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.t[i]
        t1 = self.t[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        if self.lambda_straightening > 0.0:
            term1 = a * self.slack_points_y[i] + b * self.slack_points_y[i + 1]
        else:
            term1 = a * self.P[i][1] + b * self.P[i + 1][1]
        term2 = ((a**3 - a) * self.M[i] + (b**3 - b) * self.M[i + 1]) * (h**2) / 6

        return term1 + term2

    def evaluate(self, s):
        if self.lambda_straightening > 0.0:
            x = self.evaluate_axis(self.P[:, 0], self.slack_points_x, self.spline_x, s)
            y = self.evaluate_axis(self.P[:, 1], self.slack_points_y, self.spline_y, s)

        else:
            x = self.evaluate_axis(self.P[:, 0], None, self.spline_x, s)
            y = self.evaluate_axis(self.P[:, 1], None, self.spline_y, s)
        return np.array([x, y])

    # ----------------------------------------------------------
    # First derivative
    # ----------------------------------------------------------
    def derivative_axis(self, values, M, slack_points, s):
        i = np.searchsorted(self.t, s) - 1
        i = np.clip(i, 0, self.n - 2)

        t0 = self.t[i]
        t1 = self.t[i + 1]
        h = t1 - t0

        a = (t1 - s) / h
        b = (s - t0) / h

        if self.lambda_straightening > 0.0:
            dv = (slack_points[i + 1] - slack_points[i]) / h
        else:
            dv = (values[i + 1] - values[i]) / h
        dM = (3 * b * b - 1) * M[i + 1] * h / 6 - (3 * a * a - 1) * M[i] * h / 6
        return dv + dM

    def derivative(self, s):
        dx = self.derivative_axis(self.P[:, 0], self.spline_x, self.slack_points_x, s)
        dy = self.derivative_axis(self.P[:, 1], self.spline_y, self.slack_points_y, s)
        return np.array([dx, dy])

    # ----------------------------------------------------------
    # Second derivative
    # ----------------------------------------------------------
    def second_derivative_axis(self, M, s):
        # NOTE: shitty hacks, i can't be bothered
        if s <= 0.0:
            s = 1e-5
        # equal on a float lmao
        if s == 1.0:
            s = 1 - 1e-5
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
        dx, dy = self.derivative(s)
        ddx, ddy = self.second_derivative(s)
        denom = (dx * dx + dy * dy) ** 1.5
        if denom < 1e-12:
            kappa = 0.0
        else:
            kappa = (dx * ddy - dy * ddx) / denom

        return kappa

    def getSE2FrenetInduced(self, s):
        T_s = self.derivative(s)
        T_sn = T_s / np.linalg.norm(T_s)
        R = np.array([[T_sn[0], -T_sn[1]], [T_sn[1], T_sn[0]]])
        p = self.evaluate(s)
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

    def getPathPoint(self, s: float) -> CartesianReference:
        T_s, V_s = self.getCurrentSE3Ref(s)
        # TODO: figure out what the acceleration is actually supposed to be
        a = self.second_derivative(s)
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
