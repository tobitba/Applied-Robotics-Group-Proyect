from smc import load_config
import numpy as np
from smc.motion_planning.interpolation.bsplines2D_unedited_vibecode import CubicSpline2D
from utils import getPathPoints
from workspace2d import WorkspaceSE2
from cart_path_feasibility_checker import findCartFromBase
from pinocchio import SE3, rpy


import matplotlib.pyplot as plt
import matplotlib
import pickle
from matplotlib.patches import Rectangle


def plotPathPoints(points, ax, radii, fill_alpha):
    for i, point in enumerate(points):
        circle = matplotlib.patches.Circle(
            (point[0], point[1]),
            radii[i],
            facecolor=(1, 0, 0, fill_alpha),
            edgecolor=(1, 0, 0, 0.5),
            linewidth=1.2,
            linestyle="--",
            #            antialiased=True,
        )
        ax.add_patch(circle)
    ax.scatter(points[:, 0], points[:, 1], color="r", label="data points")


def plotCartAndBaseRectangles(ax, spline3, workspace, L_b_c_desired, s_b: float):
    slumi_l = 0.52 * 2
    slumi_w = 0.58
    cart_l = 0.86
    cart_w = 0.35
    s_c = 0.0

    L_b_c = 0
    N_pts = 500
    begin = 0.0
    end = 1.0
    S = np.linspace(begin, end, N_pts)
    d_s_b = (end - begin) / N_pts
    L_FIRST = abs(-0.52 + abs(workspace.center[0])) + workspace.radius
    for s in S:
        derivative = spline3.derivative(s)
        L_b_c += np.linalg.norm(derivative) * d_s_b
        if L_b_c > L_FIRST:
            smallest_s_b = s
            print("smallest_s_b", smallest_s_b)
            break
    for s in S:
        if s < smallest_s_b:
            continue
        derivative = spline3.derivative(s)
        L_b_c += np.linalg.norm(derivative) * d_s_b
        if L_b_c < L_b_c_desired:
            continue
        Psi_b = spline3.getPathPoint(s)[0]
        s_c, L_b_c = findCartFromBase(workspace, spline3, s, s_c, L_b_c, L_b_c_desired)
        Psi_c = spline3.getPathPoint(s_c)[0]
        gamma_sb, _ = workspace.projectIntoWorkspaceEuclidean(Psi_c, Psi_b)
        if s >= s_b:
            break

    # gamma_sb.rotation = gamma_sb.rotation @ rpy.rpyToMatrix(0.0, 0.0, np.pi)
    mobile_base = Rectangle(
        (-slumi_l / 2, -slumi_w / 2),
        slumi_l,
        slumi_w,
        edgecolor="royalblue",
        facecolor=(0, 0, 1, 0.1),
    )
    cart = Rectangle(
        (-cart_l / 2, -cart_w / 2),
        cart_l,
        cart_w,
        edgecolor="turquoise",
        facecolor=(0, 1, 1, 0.1),
    )
    tb = (
        matplotlib.transforms.Affine2D()
        .rotate_deg((180 / np.pi) * rpy.matrixToRpy(Psi_b.rotation)[-1])
        .translate(Psi_b.translation[0], Psi_b.translation[1])
        + ax.transData
    )
    mobile_base.set_transform(tb)
    # gamma s_b is the location for the arms, not the cart
    # and the rectangle plot requires the rectangle center
    cart_center_offset = SE3(np.eye(3), np.array([cart_l / 2, 0.0, 0.0]))
    gamma_sb_plot = gamma_sb.act(cart_center_offset)
    tc = (
        matplotlib.transforms.Affine2D()
        .rotate_deg((180 / np.pi) * rpy.matrixToRpy(gamma_sb_plot.rotation)[-1])
        .translate(gamma_sb_plot.translation[0], gamma_sb_plot.translation[1])
        + ax.transData
    )
    cart.set_transform(tc)

    ax.add_patch(mobile_base)
    ax.add_patch(cart)
    return gamma_sb


if __name__ == "__main__":
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "serif",
            "font.serif": ["Computer Modern Roman"],
            "font.size": 18,
        }
    )
    cfg = load_config()

    #######################################################
    ############## PAPER FIGURES ##########################
    #######################################################

    # N = 7
    ## t = np.linspace(0.0, 3.14, N) / 2
    ## sint = np.sin(t * 2) * 1
    ## points = np.hstack((t.reshape((-1, 1)), sint.reshape((-1, 1))))
    # points = np.array(
    #    [
    #        [-1.0, -1.0],
    #        [0.0, 0.0],
    #        [1.0, 1.0],
    #        [2.0, 2.0],
    #        [3.0, 1.0],
    #        [4.0, 0.0],
    #        [5.0, -1.0],
    #        #        [3.0, 2.0],
    #    ]
    # )
    # points /= 3
    ## points = np.zeros((N, 2))
    ## for i in range(1, N):
    ##    dx = np.random.random() * 2
    ##    dy = (np.random.random() - 0.5) * 4
    ##    points[i, 0] += points[i - 1, 0] + dx
    ##    points[i, 1] += points[i - 1, 1] + dy

    base_width = 0.58 / 2
    points, radii = getPathPoints()
    N = len(points)
    if cfg.real:
        points = points[:-1]
        radii = radii[:-1]

    # init_vel = np.array([-10.0, -7.0])
    init_vel = np.array([0.0, 0.0])
    final_vel = np.array([0.0, 0.0])
    spline3 = CubicSpline2D(
        points, 0.9999, init_vel, final_vel, radii=radii - base_width
    )

    S = np.linspace(0, 1, 400)
    curve3 = np.array([spline3.evaluate(s) for s in S])
    # curve3 = np.array([spline3.evaluate(s) for s in S])

    ######################################
    ############# FIGURE 1 ###############
    ######################################

    fig, ax = plt.subplots(edgecolor="black", linewidth=5)
    ax.set_aspect("equal")
    ax.axis("off")

    plotPathPoints(points, ax, radii, 0.05)
    ax.set_title("Path planning")
    # plt.text(0.0, 2.0, r"$\mathbold{p} = \left\{ (x_0,y_0), \dots \right\}$")
    plt.text(
        -0.2,
        -1.0,
        r"$\mathbf{p} = \left\{ (x_0,y_0), \dots (x_N,y_N) \right\}$",
        fontsize=18,
    )
    plt.savefig("./plots/PLOT_1.pdf", dpi=600)
    plt.savefig("./plots/PLOT_1.svg", dpi=600)
    plt.savefig("./plots/PLOT_1.png", dpi=600)
    plt.show()

    ######################################
    ############# FIGURE 2 ###############
    ######################################

    # ax.plot(curve1[:, 0], curve1[:, 1], "b", label="natural spline")
    # ax.plot(curve2[:, 0], curve2[:, 1], "g", label="approx spline")
    fig, ax = plt.subplots(edgecolor="black", linewidth=5)
    ax.set_aspect("equal")
    ax.axis("off")
    plotPathPoints(points, ax, radii, 0.05)
    # ax.scatter(points[:, 0], points[:, 1], color="r", label="data points")
    ax.set_title("Smoothing interpolation")

    plt.text(
        0.3,
        -1.0,
        r"$\Psi (s) \in C^2$",
        fontsize=18,
    )

    ax.plot(curve3[:, 0], curve3[:, 1], color="royalblue", label="most approx spline")
    slack_points = np.hstack(
        (
            spline3.slack_points_x.reshape((-1, 1)),
            spline3.slack_points_y.reshape((-1, 1)),
        )
    )
    plt.savefig("./plots/PLOT_2.pdf", dpi=600)
    plt.savefig("./plots/PLOT_2.svg", dpi=600)
    plt.savefig("./plots/PLOT_2.png", dpi=600)

    # litrally just to flush
    plt.show()

    ######################################
    ############# FIGURE 3 ###############
    ######################################

    fig, ax = plt.subplots(edgecolor="black", linewidth=5)
    ax.set_aspect("equal")
    ax.axis("off")
    plotPathPoints(points, ax, radii, 0.00)
    ax.scatter(points[:, 0], points[:, 1], color="r", label="data points")
    ax.plot(curve3[:, 0], curve3[:, 1], color="royalblue", label="most approx spline")

    file = open("./parameters/workspace_best.pickle", "rb")
    # file = open("./workspace_aggressive.pickle", "rb")
    # file = open("./workspace_center_at_base.pickle", "rb")
    workspace_raw = pickle.load(file)
    file.close()
    workspace = WorkspaceSE2(cfg, workspace_raw)
    print(
        f"opened workspace with center {workspace.center} and radius {workspace.radius}"
    )

    spline3.setSE3LiftingParams(np.pi, 0.96)

    gamma_s = []
    s_c = 0.0
    gamma_s_tangents = []
    Psi_c_tangents = []
    Psi_cs = []
    s_cs = []
    begin = 0.0
    end = 1.0
    N_pltpls = 400
    S = np.linspace(begin, end, N_pltpls)
    d_s_b = (end - begin) / N_pltpls

    print(
        "theoretical best base-cart arclength is",
        abs(-0.52 + abs(workspace.center[0])) + workspace.radius,
    )
    L_b_c_desired = cfg.cart_pulling.base_to_cart_arclength
    # L_b_c_desired = 0.8
    print("we want arclength distance to be", L_b_c_desired)
    L = 0.0
    for s in S:
        derivative = spline3.derivative(s)
        L += np.linalg.norm(derivative) * d_s_b
    print("total length is", L)

    L_b_c = 0.0
    ss = []
    L_FIRST = abs(-0.52 + abs(workspace.center[0])) + workspace.radius
    for s in S:
        derivative = spline3.derivative(s)
        L_b_c += np.linalg.norm(derivative) * d_s_b
        if L_b_c > L_FIRST:
            smallest_s_b = s
            print("smallest_s_b", smallest_s_b)
            break
    for s in S:
        if s < smallest_s_b:
            continue
        derivative = spline3.derivative(s)
        L_b_c += np.linalg.norm(derivative) * d_s_b
        if L_b_c < L_b_c_desired:
            continue
        ss.append(s)
        # NOTE: idk if doing it this way, or even the derivative itself is correct
        Psi_b = spline3.getPathPoint(s)[0]
        #    tb = spline3.derivative(s)
        #    tb = tb / np.linalg.norm(tb)
        #    R_b = np.array([[tb[0], -tb[1], 0.0], [tb[1], tb[0], 0.0], [0.0, 0.0, 1.0]])
        #    T_w_s_base = SE3(R_b, np.array([
        s_c_new, L_b_c = findCartFromBase(
            workspace, spline3, s, s_c, L_b_c, L_b_c_desired
        )
        Psi_c = spline3.getPathPoint(s_c)[0]
        #    if s_c_new - s_c < 1e-8:
        #        print("we have a problem")
        s_c = s_c_new
        s_cs.append(s_c)
        gamma_sb, _ = workspace.projectIntoWorkspaceEuclidean(Psi_c, Psi_b)
        # TODO: this check does nothing good.
        # because the base distance from the first point needs to be insured,
        # not this. with this they can both be at the first point.
        # if (gamma_sb.translation[0] < points[0][0]) or (
        #    gamma_sb.translation[1] < points[0][1]
        # ):
        #    continue
        gamma_s.append(gamma_sb.translation[:2])
        gamma_s_tangents.append([gamma_sb.rotation[0][0], gamma_sb.rotation[1][0]])
        Psi_c_tangents.append([Psi_c.rotation[0][0], Psi_c.rotation[1][0]])
        Psi_cs.append([Psi_c.translation[0], Psi_c.translation[1]])

    gamma_s_tangents = np.array(gamma_s_tangents)
    mask_gamma_s_tangents = np.arange(len(gamma_s_tangents)) % 10 == 0

    Psi_cs = np.array(Psi_cs)
    Psi_c_tangents = np.array(Psi_c_tangents)
    mask_Psi_c_tangents = np.arange(len(Psi_c_tangents)) % 10 == 0

    gamma_s = np.array(gamma_s)
    ax.plot(gamma_s[:, 0], gamma_s[:, 1], color="turquoise", label="gamma_s")
    ax.plot(
        curve3[:, 0], curve3[:, 1], color="royalblue"
    )  # , label="most approx spline")
    # ax.plot(Psi_cs[:, 0], Psi_cs[:, 1], color="turquoise", label="Psi_cs")

    # tu treba stavit slicicu robota i kolica
    plotCartAndBaseRectangles(ax, spline3, workspace, L_b_c_desired, 0.6)

    plt.text(
        0.25,
        -1.0,
        r"$\Gamma (s) \in C^2, \mathcal{W}$",
        fontsize=18,
    )
    ax.set_title(r"Feasibility evaluation and cart reference construction")
    plt.savefig("./plots/PLOT_3.pdf", dpi=600)
    plt.savefig("./plots/PLOT_3.svg", dpi=600)
    plt.savefig("./plots/PLOT_3.png", dpi=600)

    plt.show()

    #########################################
    ####### PLOTS FOR UNDERSTANDING #########
    #########################################

    # ax.quiver(
    #    gamma_s[mask_gamma_s_tangents][:, 0],
    #    gamma_s[mask_gamma_s_tangents][:, 1],
    #    gamma_s_tangents[mask_gamma_s_tangents][:, 0],
    #    gamma_s_tangents[mask_gamma_s_tangents][:, 1],
    #    width=0.002,
    #    # color="turquoise",
    #    color="royalblue",
    # )

    # NOTE: this is proof quivers are correct
    # ax.quiver(
    #    Psi_cs[mask_Psi_c_tangents][:, 0],
    #    Psi_cs[mask_Psi_c_tangents][:, 1],
    #    Psi_c_tangents[mask_Psi_c_tangents][:, 0],
    #    Psi_c_tangents[mask_Psi_c_tangents][:, 1],
    #    width=0.003,
    #    color="royalblue",
    # )

    # ss = np.array(ss)
    # ttt = np.arange(len(ss))
    # plt.plot(ttt, ss, label="s_b")
    # s_cs = np.array(s_cs)
    # plt.plot(ttt, s_cs, label="s_c")
    # plt.legend()
    # plt.show()

    #################################################################
    ############## PROJECTION VERIFICATION ##########################
    #################################################################

    fig, (ax1, ax2) = plt.subplots(1, 2)

    ax1.set_title(r"$\Psi_b^c(s_c)$")
    ax1.set_aspect("equal")
    ax1.set_xlabel("x/m")
    ax1.set_ylabel("y/m")
    ax2.set_title(r"$\Gamma_b^c(s_b)$")
    ax2.set_aspect("equal")
    ax2.set_xlabel("x/m")
    ax2.set_ylabel("y/m")

    # ############### requested ######################
    xss = np.array(
        [psi_c_pp.translation[0] for psi_c_pp in workspace.requested_psi_b_c]
    )
    yss = np.array(
        [psi_c_pp.translation[1] for psi_c_pp in workspace.requested_psi_b_c]
    )
    requested_headings = np.zeros((len(xss), 2))
    for i in range(len(xss)):
        requested_headings[i][0] = np.cos(workspace.requested_headings[i])
        requested_headings[i][1] = np.sin(workspace.requested_headings[i])

    # mask = np.arange(0, len(xss) - 1, 10)
    mask = np.arange(len(xss)) % 10 == 0
    scale_arrows = 1.0
    ax1.quiver(
        xss[mask],
        yss[mask],
        requested_headings[mask][:, 0] * scale_arrows,
        requested_headings[mask][:, 1] * scale_arrows,
        scale=10.0,
        width=0.006,
        color="turquoise",
    )

    # plt.plot(xss, yss, color="green")
    # ax1.scatter(
    #    np.array([workspace.center[0]]),
    #    np.array([workspace.center[1]]),
    #    marker="*",
    #    color="gold",
    # )
    xs = np.zeros(len(workspace.phis))
    ys = np.zeros(len(workspace.phis))
    headings_min = np.zeros((len(workspace.phis), 2))
    headings_max = np.zeros((len(workspace.phis), 2))
    for i in range(len(workspace.phis)):
        xs[i] = workspace.radius * np.cos(workspace.phis[i])  # + workspace.center[0]
        ys[i] = workspace.radius * np.sin(workspace.phis[i])  # + workspace.center[1]
        headings_min[i][0] = np.cos(workspace.thetas_min[i])
        headings_min[i][1] = np.sin(workspace.thetas_min[i])
        headings_max[i][0] = np.cos(workspace.thetas_max[i])
        headings_max[i][1] = np.sin(workspace.thetas_max[i])

        # print(
        #    f"for phi: {workspace.phis[i]}, heading range is [{workspace.thetas_min[i]},{workspace.thetas_max[i]}]"
        # )
    phis_bigger = np.linspace(-1.4, 1.4, 300)
    xs_bigger = workspace.radius * np.cos(phis_bigger)  # + workspace.center[0]
    ys_bigger = workspace.radius * np.sin(phis_bigger)  # + workspace.center[1]
    ax1.plot(xs_bigger, ys_bigger, color="forestgreen")
    # plt.plot(xs, ys, color="red")
    mask = np.arange(len(xs)) % 2 == 0
    # too busy rn, was a useful check
    ax1.quiver(
        xs[mask],
        ys[mask],
        headings_min[mask][:, 0] * scale_arrows,
        headings_min[mask][:, 1] * scale_arrows,
        scale=10.0,
        width=0.004,
        color="lightcoral",
    )
    ax1.quiver(
        xs[mask],
        ys[mask],
        headings_max[mask][:, 0] * scale_arrows,
        headings_max[mask][:, 1] * scale_arrows,
        scale=10.0,
        width=0.004,
        color="maroon",
    )

    # ############### projected ######################

    xss = np.array([psi_c_pp.translation[0] for psi_c_pp in workspace.gammas_c])
    yss = np.array([psi_c_pp.translation[1] for psi_c_pp in workspace.gammas_c])
    headings = np.zeros((len(xss), 2))
    for i in range(len(xss)):
        headings[i][0] = np.cos(workspace.headings[i])
        headings[i][1] = np.sin(workspace.headings[i])

    # mask = np.arange(0, len(xss) - 1, 10)
    mask = np.arange(len(xss)) % 2 == 0

    ax2.quiver(
        xss[mask],
        yss[mask],
        headings[mask][:, 0] * scale_arrows,
        headings[mask][:, 1] * scale_arrows,
        scale=10.0,
        width=0.006,
        color="royalblue",
    )

    # plt.plot(xss, yss, color="green")
    # ax2.scatter(
    #    np.array([workspace.center[0]]),
    #    np.array([workspace.center[1]]),
    #    marker="*",
    #    color="gold",
    # )
    xs = np.zeros(len(workspace.phis))
    ys = np.zeros(len(workspace.phis))
    headings_min = np.zeros((len(workspace.phis), 2))
    headings_max = np.zeros((len(workspace.phis), 2))
    for i in range(len(workspace.phis)):
        xs[i] = workspace.radius * np.cos(workspace.phis[i])  # + workspace.center[0]
        ys[i] = workspace.radius * np.sin(workspace.phis[i])  # + workspace.center[1]
        headings_min[i][0] = np.cos(workspace.thetas_min[i])
        headings_min[i][1] = np.sin(workspace.thetas_min[i])
        headings_max[i][0] = np.cos(workspace.thetas_max[i])
        headings_max[i][1] = np.sin(workspace.thetas_max[i])

        # print(
        #    f"for phi: {workspace.phis[i]}, heading range is [{workspace.thetas_min[i]},{workspace.thetas_max[i]}]"
        # )
    # ax2.plot(xs, ys, color="coral")
    ax2.plot(xs_bigger, ys_bigger, color="forestgreen")

    plt.savefig("./plots/PLOT_4.pdf", dpi=600)
    plt.savefig("./plots/PLOT_4.svg", dpi=600)
    plt.savefig("./plots/PLOT_4.png", dpi=600)
    # plt.plot(xs, ys, color="red")
    mask = np.arange(len(xs)) % 2 == 0
    # too busy rn, was a useful check
    # ax2.quiver(
    #    xs[mask],
    #    ys[mask],
    #    headings_min[mask][:, 0],
    #    headings_min[mask][:, 1],
    #    width=0.002,
    #    color="turquoise",
    # )
    # ax2.quiver(
    #    xs[mask],
    #    ys[mask],
    #    headings_max[mask][:, 0],
    #    headings_max[mask][:, 1],
    #    width=0.002,
    #    color="blue",
    # )
    # base frame is at (0,0)
    # the circle radius is then at workspace.center
    # base frame
    plt.show()

    NN = len(workspace.requested_headings)
    NN_arange = np.arange(NN)
    plt.plot(NN_arange, workspace.phi_cs, color="red", label="phi_c")
    plt.plot(NN_arange, workspace.phi_cs_clipped, color="orange", label="phis_clipped")
    plt.plot(
        np.arange(NN),
        workspace.requested_headings,
        color="royalblue",
        label="theta_requested",
    )
    plt.plot(NN_arange, workspace.headings, color="turquoise", label="theta_projected")
    plt.plot(
        NN_arange,
        np.ones(NN) * workspace.phis[0],
        color="lightblue",
        linestyle="dashed",
    )
    plt.plot(
        NN_arange,
        np.ones(NN) * workspace.phis[-1],
        color="lightblue",
        linestyle="dashed",
    )

    thetas_min = np.array(
        [
            workspace.spline_thetas_min.evaluate_axis_y(
                (phi - workspace.phis[0]) / workspace.phi_range
            )
            for phi in workspace.phi_cs_clipped
        ]
    )
    plt.plot(NN_arange, thetas_min, color="maroon", linestyle="dashed")
    thetas_max = np.array(
        [
            workspace.spline_thetas_max.evaluate_axis_y(
                (phi - workspace.phis[0]) / workspace.phi_range
            )
            for phi in workspace.phi_cs_clipped
        ]
    )
    plt.plot(NN_arange, thetas_max, color="maroon", linestyle="dashed")
    plt.legend()
    plt.show()

    plt.plot(
        workspace.phi_cs,
        workspace.requested_headings,
        color="royalblue",
        label="theta_requested",
    )
    plt.plot(
        workspace.phi_cs_clipped,
        workspace.headings,
        color="turquoise",
        linestyle="dashed",
        label="theta_projected",
    )
    thetas_min = np.array(
        [
            workspace.spline_thetas_min.evaluate_axis_y(s)
            for s in np.linspace(0, 1.0, len(workspace.phis))
        ]
    )
    thetas_max = np.array(
        [
            workspace.spline_thetas_max.evaluate_axis_y(s)
            for s in np.linspace(0, 1.0, len(workspace.phis))
        ]
    )
    plt.plot(workspace.phis, thetas_min, color="red", label="min")
    plt.plot(workspace.phis, thetas_max, color="red", label="max")
    # plt.plot(workspace.phis, workspace.thetas_min, color="red", label="min")
    # plt.plot(workspace.phis, workspace.thetas_max, color="red", label="max")
    plt.legend()
    plt.show()

    # t = np.arange(len(workspace.phi_cs))
    # plt.plot(t, workspace.phi_cs, color="royalblue", label="phis_requested")
    # plt.plot(t, workspace.phi_cs_clipped, color="turquoise", label="phis_clipped")
    # plt.legend()
    # plt.show()

    # psis_c = np.array([a.translation[:2] for a in workspace.requested_psi_b_c])
    # gammas_c = np.array([a.translation[:2] for a in workspace.gammas_c])
    # plt.plot(psis_c[:, 0], psis_c[:, 1], color="royalblue", label="requested")
    # plt.plot(gammas_c[:, 0], gammas_c[:, 1], color="turquoise", label="projected")
    # plt.legend()
    # plt.show()

    # ax.scatter(slack_points[:, 0], slack_points[:, 1], marker="*", color="y")
    # plt.plot(curve3[:, 0], curve3[:, 1], "b", label="spline")
    # ax.legend()

    # curv = np.array([spline1.curvature(s) for s in S])
    # plt.plot(S * points[-1][0], curv)
    # plt.title("Curvature κ(s)")
    # plt.xlabel("s")
    # plt.ylabel("curvature")
