from smc.bookkeeping.load_config import GlobalConfig
from smc.motion_planning.interpolation.bsplines2D_unedited_vibecode import CubicSpline2D
from pinocchio import SE3
import pinocchio as pin
import numpy as np

# from smc.robots.interfaces.single_arm_interface import SingleArmInterface


class WorkspaceSE2:
    """
    available 2D workspace expressed as 3D polygon,
    where the 3rd dimension is the heading.
    it's all expressed in the robot's base frame of course.

    most computations first do the planar 2 dimensions x and y,
    and then the heading is treated separately as this makes life
    a bit easier.
    """

    def __init__(
        self,
        cfg: GlobalConfig,
        workspace_pts: dict[str, np.ndarray],
        collect_plot_data=True,
    ):
        # NOTE: for obvious reasons, we want to work with polar coordinates.
        # (we have a circle, so radius is fixed, so we can describe
        # every point just with the angle phi. anything else is just wrong and inumerate)
        # NOTE: everything is to be expressed in mobile base frame.
        # NOTE: slumi magic number, sorry
        self.cfg = cfg
        self.radius = workspace_pts["radius"]
        self.center = workspace_pts["center"]
        xs = workspace_pts["xs"]
        ys = workspace_pts["ys"]
        heading_intervals = workspace_pts["theta_intervals"]
        self.thetas_min = np.zeros(len(xs))
        self.thetas_max = np.zeros(len(xs))
        self.phis = np.zeros(len(xs))
        for i in range(len(xs)):
            self.phis[i] = np.arctan2(ys[i] - self.center[1], xs[i] - self.center[0])
            # +pi is a computation artefact
            if heading_intervals[i] is not None:
                self.thetas_min[i] = heading_intervals[i][0]
                self.thetas_max[i] = heading_intervals[i][1]
            else:
                self.thetas_min[i] = np.nan
                self.thetas_max[i] = np.nan

            # print(f"(x,y) = ({xs[i]}, {ys[i]}")

        # print(self.phis)
        # exit()
        sorted_indeces = np.argsort(self.phis)
        self.phis = self.phis[sorted_indeces]
        print(self.phis)
        # self.phis = self.phis + np.pi
        # this is basically just rotation again.
        # cos for the workspace analysis we rotated the base to work with in the arms' base frame.
        # this was a stupid mistake because it caused so many errors down the line,
        # and it could've been approached better, but it is what it is now.
        temp = self.thetas_min.copy()
        self.thetas_min = -1 * self.thetas_max[sorted_indeces]
        self.thetas_max = -1 * temp[sorted_indeces]
        # print(self.thetas_min)
        mask_sensible = self.thetas_min < 1000
        # print(mask_sensible)
        self.phis = self.phis[mask_sensible]
        self.thetas_max = self.thetas_max[mask_sensible]
        self.thetas_min = self.thetas_min[mask_sensible]
        # self.thetas_min = self.thetas_min[sorted_indeces]
        # self.thetas_max = self.thetas_max[sorted_indeces]

        # TODO: this should be deleted, and it's a hotfix.
        # basically, real yumi has lower joint limits than what it says in the urdf.
        # the real solution would be to recompute the workspace with the correct limits.
        # i ain't got time for that tho.
        # so i'll just chop the available headings
        if cfg is not None:
            if cfg.real:
                retain_perc = 0.90
                cutoff_phi = np.arange(
                    int(len(self.phis) * (1 - retain_perc)),
                    int(len(self.phis) * retain_perc),
                )
                self.phis = self.phis[cutoff_phi]
                self.thetas_max = retain_perc * self.thetas_max
                self.thetas_min = retain_perc * self.thetas_min

        # TODO: jsut a test, remove later
        # maskk = np.arange(len(self.phis))
        # mask = maskk < (len(self.phis) // 4)  # *
        # mm = maskk > (3 * len(self.phis) // 4)
        # mask = np.bitwise_invert(mask) * np.bitwise_invert(mm)
        # print(mask)
        # self.phis = self.phis[mask]
        # self.thetas_min = self.thetas_min[mask]
        # self.thetas_max = self.thetas_max[mask]
        self.fitSmoothThetas()

        center_frame = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
        # i hope this is correct le mao
        transl_center = np.zeros(3)
        # NOTE: magic number is base length and in which frame these
        # circle points are, don't ask me anything please
        transl_center[0] = -0.52 + abs(self.center[0])
        self.T_b_center = SE3(center_frame, transl_center)
        # self.T_b_workspace = SE3(center_frame, np.array([-0.52, 0.0, 0.0]))

        self.collect_plot_data = collect_plot_data
        if self.collect_plot_data:
            self.requested_psi_b_c = []
            self.requested_headings = []
            self.gammas_c = []
            self.headings = []
            self.phi_cs = []
            self.phi_cs_clipped = []

    @staticmethod
    def smoothClip(x, xmin, xmax, k):
        x_range = xmax - xmin
        x_range_plus = xmax + xmin
        return (x_range / 2) * np.tanh(
            (k * (2 * x - x_range_plus)) / x_range
        ) + x_range_plus / 2

    def isIn2DWorkspace(self, point: np.ndarray):
        raise NotImplementedError

    # maybe even SE3 point which we chop here?
    def isInSE2Workspace(self, point: np.ndarray):
        raise NotImplementedError

    def fitSmoothThetas(self):
        N_samples = 3
        points_max = np.zeros((N_samples + 1, 2))
        points_min = np.zeros((N_samples + 1, 2))
        ind_jump = len(self.phis) // N_samples
        safety_distance = 0.1
        for i in range(N_samples + 1):
            index = min(i * ind_jump, len(self.phis) - 1)
            points_max[i][0] = self.phis[index]
            points_max[i][1] = self.thetas_max[index] - safety_distance
            points_min[i][0] = self.phis[index]
            points_min[i][1] = self.thetas_min[index] + safety_distance
        if (ind_jump * N_samples) < len(self.phis):
            points_max[-1][0] = self.phis[-1]
            points_max[-1][1] = self.thetas_max[-1] - safety_distance
            points_min[-1][0] = self.phis[-1]
            points_min[-1][1] = self.thetas_min[-1] + safety_distance

        mu = 0.0
        self.spline_thetas_max = CubicSpline2D(
            points_max,
            mu,
            [0, 0],
            [0, 0],
        )

        # i mean it is just mirrored around (0,0),
        # but this is easier to implement than the symmetry
        self.spline_thetas_min = CubicSpline2D(
            points_min,
            mu,
            [0, 0],
            [0, 0],
        )
        self.phi_range = self.phis[-1] - self.phis[0]

    # TODO: this is potentially quite ass because the first headings
    # are rather close to 0.
    # what you can do instead
    # is go along the circle further until you find a better heading.
    # whatever that means.
    # an even simpler thing to do would be to literally just chop off
    # the computed radii to a smaller range.
    # on top of that, you can also see what the heading requests are,
    # and then manually select a better radius based on that.
    def projectIntoWorkspaceEuclidean(
        self,
        Psi_c: SE3,  # position of cart on the path (if it were there)
        Psi_b: SE3,  # position of base on the path
    ) -> tuple[SE3, np.ndarray]:
        gamma_sb: SE3 = Psi_c.copy()
        # 1. map Psi_c into circle frame
        # Psi_b_c = Psi_b.actInv(Psi_c)
        Psi_center = Psi_b.act(self.T_b_center)
        Psi_center_cart = Psi_b.act(self.T_b_center).actInv(Psi_c)

        # TODO: there needs to be an integrator keeping track of the request phi_c,
        # so that they are capped in [-3,3].
        # the thing is, the cart may be more than half circle away in heading.
        # but it needs to be projected onto the correct side, not teleported to the other one
        # ---> but this is unlikely to happen, so i'll just leave the known error here iksde

        # 2. express it in polar coordinates of our circle
        # NOTE: this one is fine because when base and cart are aligned, it's 0.
        # so we get mapped onto the the workspace correctly
        phi_c = np.arctan2(
            Psi_center_cart.translation[1],
            Psi_center_cart.translation[0],
        )
        # NOTE: arctan2 gives results in [-pi,pi].
        # and we defined phis to be positive and in 2pi,
        # so we need to map the computation accordingly
        # phi_c = phi_c % (2 * np.pi)

        # then find the appropriate workspace limit,
        # AND subsequently the heading limit
        # NOTE: this gives headings in -pi,+pi range,
        # and we have set our theta intervals to comply with this.
        heading = pin.rpy.matrixToRpy(Psi_b.rotation.T @ Psi_c.rotation)[-1]

        translation = np.array(
            [
                self.radius * np.cos(phi_c),
                self.radius * np.sin(phi_c),
                0.0,
            ]
        )

        self.phi_c_req = phi_c
        self.heading_req = heading

        if self.collect_plot_data:
            # self.requested_psi_b_c.append(Psi_b_c.copy())
            self.requested_psi_b_c.append(Psi_center_cart.copy())
            self.requested_headings.append(heading)
            self.phi_cs.append(phi_c)

        # NOTE: first we slam the cart back into the allowed radius
        # if phi_c < self.phi_cs[0]:
        #    heading = self.smoothClip(heading, self.thetas_min[0], self.thetas_max[0])
        # if phi_c > self.phi_cs[-1]:
        #    heading = self.smoothClip(heading, self.thetas_min[-1], self.thetas_max[-1])
        # phi_c = np.clip(self.phis[0], phi_c, self.phis[-1])
        phi_c = self.smoothClip(phi_c, self.phis[0], self.phis[-1], k=1.0)

        # phi_index = np.searchsorted(self.phis, phi_c)
        # phi_index = np.clip(0, phi_index, len(self.phis) - 1)
        # heading = np.clip(
        #    self.thetas_min[phi_index], heading, self.thetas_max[phi_index]
        # )
        theta_min = self.spline_thetas_min.evaluate_axis_y(
            (phi_c - self.phis[0]) / self.phi_range
        )
        theta_max = self.spline_thetas_max.evaluate_axis_y(
            (phi_c - self.phis[0]) / self.phi_range
        )
        heading = self.smoothClip(heading, theta_min, theta_max, k=1.0)
        # heading = np.clip(
        #    theta_min,
        #    heading,
        #    theta_max,
        # )
        # print(f"{theta_min}, {heading}, {theta_max}")

        # NOTE: we use pin.SE3 for both SE2 and SE3 objects as it
        # just makes life easier (has no SE2 class,
        # which is not a problem - this is just a bit of useless compute)
        # NOTE: again, since the heading is in [-pi,pi],
        # this is OK
        rot = pin.rpy.rpyToMatrix(0.0, 0.0, heading)

        translation = np.array(
            [
                # self.radius * np.cos(phi_c) + self.center[0],
                # self.radius * np.sin(phi_c) + self.center[1],
                self.radius * np.cos(phi_c),
                self.radius * np.sin(phi_c),
                0.0,
            ]
        )
        gamma_sb = SE3(rot, translation)

        self.phi_c_clipped = phi_c
        self.heading_clipped = heading
        if self.collect_plot_data:
            self.gammas_c.append(gamma_sb.copy())
            self.phi_cs_clipped.append(phi_c)
            self.headings.append(heading)
        # NOTE: now we map the projected reference back
        # into the world frame.
        # gamma_sb = Psi_b.act(gamma_sb.copy())
        gamma_s = Psi_center.act(gamma_sb.copy())

        return gamma_s, np.zeros(6)

    # NOTE: i ain't finna do this one
    def projectIntoWorkspaceFixedArclength(
        self,
        d_b_c: float,  # arclength from base
        Psi_c: SE3,  # position of cart on the path (if it were there)
        Psi_b: SE3,  # position of base on the path
        V_s: np.ndarray,
    ) -> tuple[SE3, np.ndarray]:
        """
        for fixed distance d_b_c, where d_b_c implies a non-elastic string with fixed length d_b_c.
        string is to act as an involute of the path, and moving closer to T_b_neutral.
        essentially make Psi_c follow involute. as soon as it ends up is in arm workspace,
        you are done. this is the "projection via involute"
        """
        raise NotImplementedError


if __name__ == "__main__":
    import pickle
    import matplotlib.pyplot as plt

    # file = open("./parameters/workspace.pickle", "rb")
    # file = open("./parameters/workspace_center_at_base.pickle", "rb")
    # file = open("./parameters/workspace_best.pickle", "rb")
    file = open("./parameters/workspace_aggressive.pickle", "rb")
    workspacel = pickle.load(file)
    file.close()
    workspace = WorkspaceSE2(None, workspacel)
    N = 400
    phis = np.linspace(workspace.phis[0], workspace.phis[-1], N)
    ys_max = np.zeros(N)
    ys_min = np.zeros(N)
    # good god the spline really needs to be rewritten
    for i, z in enumerate(np.linspace(0, 1.0, N)):
        ys_max[i] = workspace.spline_thetas_max.evaluate_axis_y(
            z,
        )
        ys_min[i] = workspace.spline_thetas_min.evaluate_axis_y(
            z,
        )

    plt.plot(workspace.phis, workspace.thetas_max, color="b", label="raw_thetas_max")
    plt.plot(phis, ys_max, label="thetamaxfit")
    plt.plot(workspace.phis, workspace.thetas_min, color="r", label="raw_thetas_min")
    plt.plot(phis, ys_min, label="thetaminfit")
    plt.legend()
    plt.xlabel("phi/rad")
    plt.ylabel("theta/rad")

    plt.show()

    plt.plot(workspacel["xs"], workspacel["ys"], color="r", label="circle_orig")
    xs_new = np.zeros(len(workspacel["xs"]))
    ys_new = np.zeros(len(workspacel["xs"]))
    for i in range(len(workspace.phis)):
        xs_new[i] = workspace.radius * np.cos(workspace.phis[i]) + workspace.center[0]
        ys_new[i] = workspace.radius * np.sin(workspace.phis[i]) + workspace.center[1]
    plt.plot(xs_new, ys_new, color="b", label="circle_new")
    plt.legend()
    plt.show()

    #    N = 400
    #    phis = np.linspace(-3.14, 3.14, N)
    #    xs = np.zeros(N)
    #    ys = np.zeros(N)
    #    for i in range(N):
    #        xs[i] = workspace.radius * np.cos(phis[i]) + workspace.center[0]
    #        ys[i] = workspace.radius * np.sin(phis[i]) + workspace.center[1]
    #    plt.plot(xs, ys, color="m", label="full circle")
    # xs_up = np.zeros(N // 2)
    # ys_up = np.zeros(N // 2)
    # xs_down = np.zeros(N // 2)
    # ys_down = np.zeros(N // 2)
    # for i in range(N // 2):
    #    xs_up[i] = workspace.radius * np.cos(phis[i]) + workspace.center[0]
    #    ys_up[i] = workspace.radius * np.sin(phis[i]) + workspace.center[1]
    # for i in range(N // 2):
    #    xs_down[i + N // 2] = workspace.radius * np.cos(phis[i]) + workspace.center[0]
    #    ys_down[i + N // 2] = workspace.radius * np.sin(phis[i]) + workspace.center[1]
    # plt.plot(xs_new, ys_new, color="m", label="full circle")
    # plt.plot(xs_new, ys_new, color="m", label="full circle")
