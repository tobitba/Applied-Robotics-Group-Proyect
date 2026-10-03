from smc import load_config
import numpy as np
import pinocchio as pin
from workspace2d import WorkspaceSE2
from utils import ConfigCartPulling, getCartPoints, getPathPoints


# returns s_c maintaining distance
# NOTE: THE DISTANCE HAS TO BE IN PATH ARCLENGTH
def findCartFromBase(
    workspace: WorkspaceSE2, smooth_path, s_b, s_c_old, L_b_c, L_b_c_desired
) -> tuple[float, float]:
    ds_c = 1e-4
    s_c = s_c_old
    while L_b_c > L_b_c_desired:
        s_c += ds_c
        L_b_c -= np.linalg.norm(smooth_path.derivative(s_c)) * ds_c
    return s_c, L_b_c


def actualClosestPoint(
    point: np.ndarray, target_points: list[tuple[int, np.ndarray]]
) -> tuple[int, np.ndarray]:
    smallest_distance = 100000.0
    actual_closest_point = None
    for tp in target_points:
        distance = np.linalg.norm(point - tp[1])
        if distance < smallest_distance:
            smallest_distance = distance
            actual_closest_point = tp
    return tp


def pathFeasibilityCheck(
    cfg: ConfigCartPulling,
    smooth_path,
    workspace,
    cart_shape,
    collect_data=True,
):
    path_feasible = True
    s_c = 0.0
    begin = 0.0
    end = 1.0
    N_points = 500
    S = np.linspace(begin, end, N_points)
    d_s_b = (end - begin) / N_points
    L_b_c_desired = cfg.base_to_cart_arclength
    L_b_c = 0.0

    s_cs = []
    Gammas = []
    cart_diagonal = np.sqrt(cart_shape[0] ** 2 + cart_shape[1] ** 2)

    offending_s = 1000
    offending_point = np.array([0.0, 0.0])

    L_FIRST = abs(-0.52 + abs(workspace.center[0])) + workspace.radius
    for s in S:
        derivative = smooth_path.derivative(s)
        L_b_c += np.linalg.norm(derivative) * d_s_b
        if L_b_c > L_FIRST:
            smallest_s_b = s
            print("smallest_s_b", smallest_s_b)
            break

    #    L_b_c = 0.0
    for s in S:
        if s < smallest_s_b:
            continue
        derivative = smooth_path.derivative(s)
        L_b_c += np.linalg.norm(derivative) * d_s_b
        if L_b_c < L_b_c_desired:
            continue
        Psi_b = smooth_path.getPathPoint(s)[0]
        s_c, L_b_c = findCartFromBase(
            workspace, smooth_path, s, s_c, L_b_c, L_b_c_desired
        )
        Psi_c = smooth_path.getPathPoint(s_c)[0]
        Gamma_s, _ = workspace.projectIntoWorkspaceEuclidean(Psi_c, Psi_b)
        # print(np.linalg.norm(Psi_b.translation[:2] - Gamma_s.translation[:2]))

        offset = abs(-0.52 + abs(workspace.center[0]))
        oft = pin.SE3.Identity()
        oft.translation[0] = -offset
        gm_trans = Gamma_s.act(oft)
        distance = np.linalg.norm(gm_trans.translation[:2] - Psi_c.translation[:2])
        print(f"distance {distance} / {spline3.radii[0]}")
        if distance > spline3.radii[0]:
            print("collision")

        # NOTE: it doesn't work.
        # i dont' know why.
        # i'm doing a stupid hack instead
        # cart_points = getCartPoints(Gamma_s, cart_shape)
        # closest_pt_to_cart_index = np.searchsorted(smooth_path.t, s)
        # closest_pt = smooth_path.P[closest_pt_to_cart_index]
        ## NOTE: the other closer points can only be behind the Psi_c on the path
        ## and they are not further away than half cart diagonal because that's
        ## how far the cart edges are from Psi_c
        # possible_points = [(closest_pt_to_cart_index, closest_pt)]
        # distance = np.linalg.norm(Psi_c.translation[:2] - closest_pt)

        # while (distance < cart_diagonal) and (closest_pt_to_cart_index > 0):
        #    closest_pt_to_cart_index -= 1
        #    possible_points.append(
        #        (closest_pt_to_cart_index, smooth_path.P[closest_pt_to_cart_index])
        #    )
        #    distance = np.linalg.norm(
        #        Psi_c.translation[:2] - smooth_path.P[closest_pt_to_cart_index]
        #    )

        ## now, for each cart point, find the actual closest path point
        ## print(possible_points)
        # for cart_point in cart_points:
        #    closest_path_point = actualClosestPoint(
        #        cart_point.translation[:2], possible_points
        #    )
        #    # print(
        #    #    f"cart_point: {cart_point.translation[:2]}, closest_path_point: {closest_path_point}"
        #    # )
        #    distance = np.linalg.norm(
        #        closest_path_point[1] - cart_point.translation[:2]
        #    )
        #    if distance > smooth_path.radii[closest_path_point[0]]:
        #        path_feasible = False
        #        print(
        #            f"we have a collision at path parameter s {s} with path point {closest_path_point[0]} which is at {closest_path_point[1]}"
        #        )
        #        offending_s = s
        #        offending_point = closest_path_point[1]

        if collect_data:
            s_cs.append(s_c)
            Gammas.append(Gamma_s)
    return path_feasible, s_cs, Gammas, offending_s, offending_point


if __name__ == "__main__":
    from plot_city import plotPathPoints
    from smc.motion_planning.interpolation.bsplines2D_unedited_vibecode import (
        CubicSpline2D,
    )
    import matplotlib.pyplot as plt
    import pickle
    from matplotlib.patches import Rectangle
    from plot_city import plotCartAndBaseRectangles

    # get basic data in
    cfg = load_config()
    file = open("./parameters/workspace_best.pickle", "rb")
    workspace_raw = pickle.load(file)
    file.close()
    workspace = WorkspaceSE2(None, workspace_raw)
    cart_shape = (0.86, 0.35, 0.96)

    # create the same path again
    base_width = 0.58 / 2
    points, radii = getPathPoints()
    N = len(points)

    # init_vel = np.array([-10.0, -7.0])
    init_vel = np.array([0.0, 0.0])
    final_vel = np.array([0.0, 0.0])
    spline3 = CubicSpline2D(
        points, 0.9999, init_vel, final_vel, radii=radii - base_width
    )
    spline3.setSE3LiftingParams(np.pi, 0.96)

    S = np.linspace(0, 1, 400)
    curve3 = np.array([spline3.evaluate(s) for s in S])

    fig, ax = plt.subplots(edgecolor="black", linewidth=5)
    plotPathPoints(points, ax, radii, 0.05)
    ax.plot(curve3[:, 0], curve3[:, 1], color="royalblue", label="Psi")
    path_feasible, s_cs, Gammas, offending_s, offending_point = pathFeasibilityCheck(
        cfg.cart_pulling,
        spline3,
        workspace,
        cart_shape,
        collect_data=True,
    )
    gammas = np.array([G.translation[:2] for G in Gammas])
    ax.plot(gammas[:, 0], gammas[:, 1], color="turquoise", label="Gamma")
    if not path_feasible:
        Gamma = plotCartAndBaseRectangles(
            ax, spline3, workspace, cfg.base_to_cart_arclength, offending_s
        )
        cart_points = getCartPoints(Gamma, cart_shape)
        cpts = np.array([cp.translation[:2] for cp in cart_points])
        plt.scatter(cpts[:, 0], cpts[:, 1], marker="*", color="y")
        plt.scatter(offending_point[0], offending_point[1], marker="*", color="g")
        # chck = []
        # for sc in s_cs:
        #    psi_c = spline3.getPathPoint(sc)[0]
        #    chck.append(psi_c)
        # psics = np.array([cp.translation[:2] for cp in chck])
        # plt.plot(psics[:, 0], psics[:, 1], color="r")

    plt.show()


"""

    # NOTE: old. but has valuable note
    # while sb < 1.0:
    #    sc += ds
    #    d_cb_current = 0.0  # whatever
    #    # NOTE:
    #    # the evaluations take almost all of the time.
    #    # so it needs to be optimized.
    #    # first of all.
    #    # avoid all copy pasting. like i copy all of the points 2 times for every eval, cmon man.
    #    # second of all, store coeffs for every spline.
    #    # there is no need to compute all the fuckin' polynomials all the time.
    #    # third, there is no need to do the sort for index every time.
    #    # you are going forward. so you only need to check if you're
    #    # on the next spline or not.
    #    # make _evaluate_fast function which takes in last i.
    #    # bezier fit so you don't compute everything 2 fucking times
    #    # instead of once.
    #    # and with all that it should be fine.
    #    # you can also claim it can be done in 1/10 the time via parallelization.
    #    # end of the story.
    #    psi_sc, _ = smooth_path.getPathPoint(sc)
    #    psi_sb, _ = smooth_path.getPathPoint(sb)
    #    while np.linalg.norm(psi_sc - psi_sb) < d_cb:
    #        sb += db
    #        psi_sb, _ = smooth_path.getPathPoint(sb)

    #    # TODO:
    #    # 1. project psi_sc into workspace to get gamma_sc
    #    # ---> this is a different function you just call
    #    # 2. get cart corners from gamma_sc
    #    # 3. verify they are in the safety radii.

    #    iter_n += 1
"""
