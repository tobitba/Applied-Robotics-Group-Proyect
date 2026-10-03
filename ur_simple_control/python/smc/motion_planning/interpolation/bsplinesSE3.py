import numpy as np
from pinocchio import SE3, log6, exp
from smc.motion_planning.interpolation.interpolator import (
    AbstractInterpolator,
    AbstractSpline,
)
from smc.bookkeeping.types import PointsSE3, CartesianReference, Twist, floatt


class CubicBezierSE3(AbstractInterpolator[SE3]):
    """
    SE3 bezier cubic. works the same way as a vector-space bezier cubic,
    we just have to consistent with frames.
    C_i are the 4 control points of the curve, C_i in SE3.
    the first and the last one are the start and goal.
    the intermediate ones depend on the start and end velocity.
    the d parameter in initialization is distance between the 2 points
    """

    def __init__(self, T_0: SE3, T_1: SE3, V_0: Twist, V_1: Twist, d: floatt):
        self.Cs: tuple[SE3, SE3, SE3, SE3]
        # inverses will be computed for the first time only when we need to access it,
        # i.e. we do lazy evaluation
        # otherwise there will be a large overhead possibly for no reason
        self.x_is: tuple
        self.control_velocities_computed: bool = False
        self.computeInterpolation(T_0, T_1, V_0, V_1, d)

    def computeInterpolation(
        self, T_0: SE3, T_1: SE3, V_0: Twist, V_1: Twist, d: floatt
    ):
        C_1 = T_0 * exp((d / 3.0) * V_0)
        C_2 = T_1 * exp(-(d / 3.0) * V_1)

        self.Cs = (T_0, C_1, C_2, T_1)
        for C in self.Cs:
            if C.translation.any() == np.nan:
                print("failed to fit. some values are Nans. exiting")
                exit()

    def getPoint(self, s: float) -> SE3:
        """
        Evaluate cubic Bezier on SE(3) using geodesic de Casteljau.
        - u in [0,1]
        - in bsplines, this is how to get point from a selected spline
        """
        # Level 1
        L01 = SE3.Interpolate(self.Cs[0], self.Cs[1], s)
        L12 = SE3.Interpolate(self.Cs[1], self.Cs[2], s)
        L23 = SE3.Interpolate(self.Cs[2], self.Cs[3], s)

        # Level 2
        M012 = SE3.Interpolate(L01, L12, s)
        M123 = SE3.Interpolate(L12, L23, s)

        # Level 3
        B = SE3.Interpolate(M012, M123, s)
        return B

    def computex_is(self):
        Cinvs = []
        for C in self.Cs:
            Cinvs.append(C.inverse())
        x_is = []
        # log differences (twists)
        x_is.append(log6(Cinvs[0] * self.Cs[1]))
        x_is.append(log6(Cinvs[1] * self.Cs[2]))
        x_is.append(log6(Cinvs[2] * self.Cs[3]))
        self.x_is = tuple(x_is)

    def getVelocity(self, s: float) -> Twist:
        """
        Computes left-trivialized twist Xi = T^{-1} dT/dt
        for a cubic Bézier on SE(3).
        """
        if not self.control_velocities_computed:
            self.computex_is()
            self.control_velocities_computed = True

        # cubic bezier derivative w.r.t. u (left-trivialized)
        return (
            3.0 * ((1 - s) ** 2) * self.x_is[0]
            + 6.0 * s * (1 - s) * self.x_is[1]
            + 3.0 * (s**2) * self.x_is[2]
        )

    def getAcceleration(self, s: float) -> Twist:
        if not self.control_velocities_computed:
            self.computex_is()
            self.control_velocities_computed = True

        b0p = -2 * (1 - s)
        b1p = 2 - 4 * s
        b2p = 2 * s

        return 3 * (b0p * self.x_is[0] + b1p * self.x_is[1] + b2p * self.x_is[2])

    def getReference(self, s: float) -> CartesianReference:
        return self.getPoint(s), self.getVelocity(s), self.getAcceleration(s)

    def get3DCurvatureVector(self, s: float) -> np.ndarray:  # 3d
        # there are 2 options:
        # 1) just do the linear part for a 3d curve in 3d space
        # 2) do a full 6d curvature on the SE3 manifold

        # option 1
        V = self.getVelocity(s)
        A = self.getAcceleration(s)
        V_n = V / np.linalg.norm(V)
        av = A / np.linalg.norm(V)
        K = av - A * A.dot(av)
        # kappa = np.linalg.norm(K)
        return K

    def getSE3Curvature(self, s: float) -> np.ndarray:  # 6d
        # there are 2 options:
        # 1) just do the linear part for a 3d curve in 3d space
        # 2) do a full 6d curvature on the SE3 manifold

        # option 1
        V = self.getVelocity(s)
        A = self.getAcceleration(s)
        denom = np.linalg.norm(A) ** 3
        if denom < 1e-12:
            kappa = 0.0
        else:
            kappa = ...
        raise NotImplementedError


class CubicBezierSplineSE3(AbstractSpline[SE3]):
    def __init__(self, T_list: PointsSE3, V_start: Twist, V_end: Twist):
        """
        contruct SE(3) bspline interpolation for a path
        defined a list of SE(3) points.
        thus the output is an C^2 parametrized curve (param 0 to 1)
        going through the list of points.

        basic cordlength to determine spacing between poses
        -> does not take orientation error into account
        -> won't work well if there's a huge orientation error in
           between points.

        TODO: use geodesic distance to take orientation differences into account
        """
        self.T_list: PointsSE3 = T_list
        self.N: int = len(T_list) - 1  # number of splines, it's len(T_list) -1
        self.V_start = V_start
        self.V_end = V_end
        self.ss: np.ndarray
        self.hs: (
            np.ndarray
        )  # differences between s's - used to normalize s to given spline
        self.V: np.ndarray
        # TODO: how do i declare the point type here??
        # do i even do it?
        self.splines: list[CubicBezierSE3[SE3]] = []
        # TODO: implement non-zero velocities
        init_vel = np.zeros(6)
        final_vel = np.zeros(6)
        super().__init__(T_list, init_vel, final_vel)

    def computeInterpolation(self):
        """
        computeInterpolation
        --------------------
        computes the matrix of control points C_segments
        """
        # TODO: enable initial and goal velocities,
        # it literally fucking works already
        assert len(self.T_list) >= 2
        if len(self.T_list) == 3:
            self.splines.append(
                CubicBezierSE3(
                    self.T_list[0],
                    self.T_list[1],
                    self.V_start,
                    self.V_end,
                    np.linalg.norm(
                        self.T_list[0].translation - self.T_list[1].translation
                    ),
                )
            )
            return

        # Chord-length parameterization
        s_list = np.zeros(len(self.T_list))
        path_linear_length = 0
        for i in range(len(self.T_list) - 1):
            d = np.linalg.norm(
                self.T_list[i + 1].translation - self.T_list[i].translation
            )
            path_linear_length += d
            s_list[i + 1] = s_list[i] + d
        s_list /= path_linear_length
        self.ss = s_list
        self.N = len(self.T_list) - 1  # number of splines
        self.hs = np.diff(self.ss)

        # do the fit.
        # first determine velocities
        self.computeWaypointVelocities()
        # then determine control points for every spline
        self.computeSplines()

    # TODO:
    # def compute_node_velocities_SE3_with_endpoints(T_list, t_list, V0, VN):
    # impose non-zero beginning and finally velocity -
    # this way you can concatenate a change of path plan online smoothly!
    def computeWaypointVelocities(
        self,
    ):
        """
        Compute left-trivialized node velocities V_i (6-vectors)
        to make a C^2 cubic spline in SE(3).
        Natural boundary conditions V_0 = V_N = 0.
        ts go from 0 to 1
        """

        # Compute secant twists Δ_i
        Delta = []
        for i in range(self.N):
            Xi = log6(self.T_list[i].inverse() * self.T_list[i + 1]) / self.hs[i]
            Delta.append(Xi)
        Delta = np.stack(Delta)  # shape (N, 6)

        # Build tridiagonal system A V = R for V_1..V_{N-1}
        # Natural boundary: V_0 = 0, V_N = 0

        # Matrix A is (N-1)x(N-1)
        A = np.zeros((self.N - 1, self.N - 1))
        R = np.zeros((self.N - 1, 6))

        for i in range(1, self.N):  # interior nodes i=1..N-1
            row = i - 1
            him1 = self.hs[i - 1]
            hi = self.hs[i] if i < self.N else None

            if i - 1 > 0:
                A[row, row - 1] = him1
            A[row, row] = 2 * (him1 + hi)
            if i < self.N - 1:
                A[row, row + 1] = hi

            # RHS for this node:
            RHSi = 3 * (him1 * Delta[i] + hi * Delta[i - 1])
            R[row, :] = RHSi

            # TODO: use non-zero present initial and final velocity:

            ## Adjust RHS for known endpoints V0 and VN
            # if i == 1:
            #    # equation for node 1 includes h_0 * V_0
            #    RHSi = RHSi - him1 * V0
            # if i == N - 1:
            #    # equation for node N-1 includes h_{N-1} * V_N
            #    RHSi = RHSi - hi * VN

        # Solve for V_1..V_{N-1}
        try:
            V_internal = np.linalg.solve(A, R)  # shape (N-1, 6)
        except np.linalg.LinAlgError:
            print(
                "couldn't find the bspline fit. the most likely reason is that your points are too close"
            )

        # Assemble V - start and end node has 0 velocity
        V = np.zeros((self.N + 1, 6))
        V[0, :] = self.V_start
        V[1 : self.N, :] = V_internal
        V[self.N, :] = self.V_end
        # V[1 : self.N] = V_internal
        self.V = V

    def computeSplines(self):
        """
        Given poses T_i and twist velocities V_i, construct cubic Bézier
        control poses C_{i,0..3} for each segment i.
        This defines 1 spline. handled in the CubicBezier class
        for better readability
        """

        for i in range(self.N):
            self.splines.append(
                CubicBezierSE3(
                    self.T_list[i],
                    self.T_list[i + 1],
                    self.V[i],
                    self.V[i + 1],
                    self.hs[i],
                )
            )

    def curvature(self) -> float:
        print("not defined atm. returning zero for compatibility")
        i, u = self.getSplineIndexAndParameter(s)
        # TODO: need to split this into 2 functions, one for
        # 3d curvature and one for full SE3 curvature
        raise NotImplementedError
        return self.splines[i].getCurvature(u)


if __name__ == "__main__":
    import meshcat_shapes
    from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
    import time

    visualizer = MeshcatVisualizer()

    # NOTE: single segment test - works as expected
    # V_0 = np.random.random(6)
    # V_1 = np.random.random(6)
    # T_0 = SE3.Random()
    # T_1 = SE3.Random()
    # meshcat_shapes.frame(visualizer.viewer["T_0"], opacity=1.0)
    # visualizer.viewer["T_0"].set_transform(T_0.homogeneous)
    # meshcat_shapes.frame(visualizer.viewer["T_1"], opacity=1.0)
    # visualizer.viewer["T_1"].set_transform(T_1.homogeneous)

    # segment = CubicBezierSE3(
    #    T_0, T_1, V_0, V_1, np.linalg.norm(T_0.translation - T_1.translation)
    # )
    # segment_pts = []
    # for s in np.linspace(0, 1, 1000):
    #    T = segment.getPoint(s)
    #    segment_pts.append(T)
    # visualizer.addFramePath("", segment_pts, every_nth_to_plot=1)
    # time.sleep(6)

    # Example poses
    T_list = [SE3.Random() for _ in range(2)]
    for i, T in enumerate(T_list):
        meshcat_shapes.frame(visualizer.viewer["T" + str(i)], opacity=1.0)
        visualizer.viewer["T" + str(i)].set_transform(T.homogeneous)
    time.sleep(6)

    # Compute spline
    V_start = np.random.random(6) / 4
    V_end = np.random.random(6) / 4
    start = time.time()
    bsplineSE3 = CubicBezierSplineSE3(T_list, V_start, V_end)
    end = time.time()
    print("bsplines computed in:", end - start, "seconds")

    interpolated = []
    for s in np.linspace(0, 1, 1000):
        T = bsplineSE3.getPoint(s)
        interpolated.append(T)

    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)
    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)

    time.sleep(100)


## One u in [0, 1] for entire geometric path
# def evaluate_SE3_trajectory_global_u(C_segments, t_list, u):
#    # clamp u
#    u = max(0.0, min(1.0, float(u)))
#
#    t0 = t_list[0]
#    tN = t_list[-1]
#    total_T = tN - t0
#    if total_T <= 0:
#        raise ValueError("t_list must be strictly increasing.")
#
#    t = t0 + u * total_T
#
#    if t >= tN:
#        seg_idx = len(t_list) - 2
#        C0, C1, C2, C3 = C_segments[seg_idx]
#        return se3_bezier_casteljau(C0, C1, C2, C3, 1.0)
#
#    N = len(t_list) - 1
#    seg_idx = None
#    for i in range(N):
#        if t_list[i] <= t < t_list[i + 1]:
#            seg_idx = i
#            break
#    if seg_idx is None:
#        seg_idx = N - 1
#
#    ti = t_list[seg_idx]
#    tip1 = t_list[seg_idx + 1]
#    hi = tip1 - ti
#    if hi <= 0:
#        raise ValueError("t_list must be strictly increasing.")
#
#    u_local = (t - ti) / hi
#
#    C0, C1, C2, C3 = C_segments[seg_idx]
#    return se3_bezier_casteljau(C0, C1, C2, C3, u_local)
