from smc.bookkeeping.load_config import GlobalConfig
from utils import (
    transfEERefToBRef,
)
from cart_path_feasibility_checker import findCartFromBase
from workspace2d import WorkspaceSE2
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.cartesian_space.ik_solvers import (
    getIKSolver,
)
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.motion_planning.trajectory_generation.trajectory_generator import (
    AbstractTimeLaw,
)


import numpy as np
from functools import partial
from collections import deque
import pinocchio as pin
from typing import Callable


# TODO: this has to be updated to only have one s,
# and that s is for the base.
# s_base_offset is still fine, although it will be
# arbitrary
def CartPullingDisjointEEAndBaseCartesianCtrlControlLoop(
    base_s_offset: float,
    workspace: WorkspaceSE2,
    T_absgoal_l: pin.SE3,
    T_absgoal_r: pin.SE3,
    path,
    time_law: AbstractTimeLaw,
    ik_solver_arms,
    ik_solver_base,
    cfg: GlobalConfig,
    robot: DualArmWholeBodyInterface,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    an attempt
    """
    t = i * robot.dt
    # NOTE: uniform time law so same s_dot for all t.
    # TODO: of course this stupid evil s_dot makes no sense,
    # literaly neither the cart nor the base is at this s,
    # and hence this t.
    # but i'm skipping some steps anyway so we'll skip this one as well.
    s, s_dot, s_ddot = time_law.getCurrentPathParameter(t)
    s_base = s + base_s_offset
    Psi_s_b, V_s_b, _ = path.getPathPoint(s_base)

    # TODO: right here somewhere is where you would map
    # from abs frame to some other cart point

    # NOTE: this is stupidly implemented
    # NOTE: idk if V_s_b needs to be multiplied with s_base_dot
    L_b_c = past_data["L_b_c"][-1]
    L_b_c += np.linalg.norm(V_s_b) * robot.dt
    s_c, L_b_c = findCartFromBase(
        workspace,
        path,
        s_base,
        s - 0.01,
        L_b_c,
        cfg.cart_pulling.base_to_cart_arclength,
    )
    Psi_s_c, V_s, _ = path.getPathPoint(s_c)
    V_s = V_s * s_dot

    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"T_w_l": Psi_s_c})
    # NOTE: since we're lagging a bit as we rely on P control mainly (because i didn't debug
    # the velocities (oops)), we should send the robot's actual position for
    # the projection point.
    # but really this is what's actuall correct as the projection
    # is defined in the robot's base frame, not in aspirational robot's base frame.
    # gamma_sb, _ = workspace.projectIntoWorkspaceEuclidean(Psi_s_c, Psi_s_b)

    T_w_b_rotated = robot.T_w_b
    T_w_b_rotated.rotation = T_w_b_rotated.rotation @ pin.rpy.rpyToMatrix(
        cfg.test_path.path_rotx, 0.0, 0.0
    )
    T_w_b_rotated.translation[2] = cfg.test_path.path_height
    gamma_sb, _ = workspace.projectIntoWorkspaceEuclidean(Psi_s_c, T_w_b_rotated)
    # print(np.linalg.norm(gamma_sb.translation[:2] - Psi_s_b.translation[:2]))
    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"T_w_r": gamma_sb})
    gamma_sb.rotation = gamma_sb.rotation @ pin.rpy.rpyToMatrix(0.0, 0.0, np.pi)
    T_w_armsref = gamma_sb

    # check cart distance
    cart_half_diagonal = np.sqrt(
        (robot.cart_shape[0] / 2) ** 2 + (robot.cart_shape[1] / 2) ** 2
    )
    # TODO: this needs to be verified somehow
    # NOTE: bit of evil using bsplines instead of something general.
    # i have a deadline in a few hours man.
    closest_pt_to_cart_index = np.searchsorted(path.t, s)
    closest_pt = path.P[closest_pt_to_cart_index]
    # NOTE: cart points seem to be correct
    # cart_points = getCartPoints(gamma_sb, robot.cart_shape)
    #    print("-" * 40)
    #    for cart_point in cart_points:
    #        print(closest_pt)
    #        print(
    #            "cart_point distance to point:",
    #            np.linalg.norm(cart_point.translation[:2] - closest_pt),
    #        )
    # if (
    #    np.linalg.norm(
    #        Psi_s_c.translation - gamma_sb.translation
    #    )  # + cart_half_diagonal
    #    > path.radii[0]
    # ):
    #    print("collision!")

    # print("-" * 40)
    # print(pin.rpy.matrixToRpy(T_w_armsref.rotation), T_w_armsref.translation)
    # print(pin.rpy.matrixToRpy(gamma_sb.rotation), gamma_sb.translation)
    # print("T_w_armsref:", T_w_armsref)
    # print("gamma_sb:", gamma_sb)
    # the higher ebd_vs_ee_path_tradeoff the closer to the path
    # and it's in 0-1 range so ok
    # K_ff_arms = cfg.ebd_vs_ee_path_tradeoff * cfg.K_ff
    # K_ff_arms = cfg.K_ff
    K_ff_arms = 0.0

    # the base just follows the path, end of story
    # NOTE: completely useless recalculation, but i'll just leave it here to
    # make my life easier
    T_w_s_base, V_s_base = path.getCurrentSE3Ref(s_base)
    V_s_base = V_s_base * s_dot
    T_w_s_base = transfEERefToBRef(cfg.test_path, T_w_s_base)

    # NOTE:
    # V_s is the same for both arms because the orientation
    # of the absolute frame is the same as the desired orientation
    # of both end effectors in this case
    T_w_s_l = T_w_armsref.act(T_absgoal_l)
    V_fb_cmd_left = pin.log6(robot.T_w_l.actInv(T_w_s_l)).vector
    T_w_s_r = T_w_armsref.act(T_absgoal_r)
    V_fb_cmd_right = pin.log6(robot.T_w_r.actInv(T_w_s_r)).vector
    V_fb_cmd_arms = np.concatenate((V_fb_cmd_left, V_fb_cmd_right))
    V_ff_arms = np.concatenate((V_s, V_s))
    V_cmd_arms = cfg.cs.K_fb * V_fb_cmd_arms + K_ff_arms * V_ff_arms

    V_cmd_base = (
        cfg.cs.K_fb * pin.log6(robot.T_w_b.actInv(T_w_s_base)).vector + V_s_base
    )

    # base
    robot.mode = robot.control_mode.base_only
    v_cmd_base = ik_solver_base(cfg.cs, robot, V_cmd_base)

    # arms
    # NOTE: this could be needed if the dynamic are ruining kinematic assumption.
    # but otherwise no (i've tried and it's obv wrong)
    # V_cmd_arms -= V_base_offset
    robot.mode = robot.control_mode.upper_body
    v_cmd_arms = ik_solver_arms(cfg.cs, robot, V_cmd_arms)

    # combining
    robot.mode = robot.control_mode.whole_body
    v_cmd = np.zeros(robot.nv)
    v_cmd[:3] = v_cmd_base
    v_cmd[3:] = v_cmd_arms

    robot.sendVelocityCommand(v_cmd)

    log_item = {}
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    #    log_item["vs_cmd"] = v_cmd
    log_item["s"] = np.array([s]).reshape((1,))
    # base obviously unlimited, keeping it in to avoid shitty indexing
    limit_low = np.min(np.abs(robot.model.lowerPositionLimit - robot.q))
    limit_up = np.min(np.abs(robot.model.upperPositionLimit - robot.q))
    log_item["phis"] = np.array([workspace.phi_c_clipped, workspace.phi_c_req])
    log_item["headings"] = np.array([workspace.heading_clipped, workspace.heading_req])
    # log_item["phi_required"] = workspace.phi_c_req
    # log_item["heading_required"] = workspace.heading_req
    log_item["cart_path_distance"] = np.linalg.norm(
        gamma_sb.translation[:2] - Psi_s_c.translation[:2]
    ).reshape((1,))

    log_item["closes_joint_limit"] = np.min(np.array([limit_low, limit_up])).reshape(
        (1,)
    )
    # log_item["ee_pose_error"] = V_fb_cmd_arms
    log_item["base_position_error"] = np.linalg.norm(
        T_w_s_base.translation[:2] - robot.T_w_b.translation[:2]
    ).reshape((1,))
    #    log_item["ee_velocity_error"] = V_ff_arms - robot.getJacobian() @ robot.v
    T_b_e = robot.T_w_b.actInv(robot.T_w_e)
    ebd = np.zeros(2)
    ebd[0] = np.linalg.norm(robot.T_w_b.translation[:2] - robot.T_w_e.translation[:2])
    # TODO: avoid rpy pls, we know better
    ebd[1] = pin.rpy.matrixToRpy(T_b_e.rotation)[2]
    # log_item["ebd_dist"] = ebd
    log_item["path_curvature"] = np.array([path.curvature(s_base)]).reshape((1,))
    # log_item["gamma_s"] = gamma_sb.translation[:2]
    save_past_item = {"L_b_c": L_b_c}

    breakFlag = s_base >= 1.0
    return breakFlag, save_past_item, log_item


def CartPullingDisjointEEAndBaseCartesianCtrl(
    s_base_offset: float,
    workspace: WorkspaceSE2,
    T_absgoal_l: pin.SE3,
    T_absgoal_r: pin.SE3,
    path,
    time_law: Callable[[float], tuple[float, float]],
    cfg: GlobalConfig,
    robot: DualArmWholeBodyInterface,
    run=True,
) -> None | ControlLoopManager:
    """
    attempt
    -----
    """

    ik_solver_selected = cfg.cs.ik_solver
    ik_solver_arms = getIKSolver(cfg.cs, robot)
    cfg.cs.ik_solver = "QPVanilla"
    ik_solver_base = getIKSolver(cfg.cs, robot)
    cfg.cs.ik_solver = ik_solver_selected

    controlLoop = partial(
        CartPullingDisjointEEAndBaseCartesianCtrlControlLoop,
        s_base_offset,
        workspace,
        T_absgoal_l,
        T_absgoal_r,
        path,
        time_law,
        ik_solver_arms,
        ik_solver_base,
        cfg,
        robot,
    )

    log_item = {}
    log_item["qs"] = np.zeros(robot.nq)
    log_item["vs"] = np.zeros(robot.nv)
    #    log_item["vs_cmd"] = np.zeros(robot.nv)
    log_item["s"] = np.zeros(
        1,
    )
    log_item["phis"] = np.zeros(
        2,
    )

    log_item["headings"] = np.zeros(
        2,
    )
    # log_item["phi_required"] = np.zeros(
    #    1,
    # )

    # log_item["heading_required"] = np.zeros(
    #    1,
    # )
    log_item["cart_path_distance"] = np.zeros(1)

    log_item["closes_joint_limit"] = np.zeros(
        1,
    )
    # log_item["ee_pose_error"] = np.zeros(2 * 6)
    #    log_item["ee_velocity_error"] = np.zeros(2 * 6)
    # log_item["ebd_dist"] = np.zeros(3)
    # log_item["ebd_dist"] = np.zeros(2)
    log_item["base_position_error"] = np.zeros(
        1,
    )
    log_item["path_curvature"] = np.zeros(
        1,
    )
    # log_item["gamma_s"] = np.zeros(2)
    save_past_data = {"L_b_c": cfg.cart_pulling.base_to_cart_arclength}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_data, log_item)

    if run:
        loop_manager.run()
    else:
        return loop_manager
