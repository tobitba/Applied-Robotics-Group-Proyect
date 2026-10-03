from smc.bookkeeping.types import CartesianReference, PastData
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.control.optimal_control.oc_config import ConfigOptimalControl
from smc.control.control_loop_manager import ControlLoopManager
from smc.multiprocessing import CollaborativeProcess
from smc.control.optimal_control.croco_path_following.ocp.base_and_dual_arm_reference_ocp import (
    BaseAndDualArmEEPathFollowingOCP,
)
from smc.motion_planning.motion_planner import AbstractMotionPlanner
from smc.control.controller_templates.path_following_template import (
    PathFollowingFromPlannerCtrllLoopTemplate,
)

import numpy as np
from functools import partial
import types
from pinocchio import SE3, log6


def BaseAndDualEEPathFollowingMPCControlLoop(
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    ocp: BaseAndDualArmEEPathFollowingOCP,
    path_base: AbstractMotionPlanner[CartesianReference],
    path_abs: AbstractMotionPlanner[CartesianReference],
    cfg: ConfigOptimalControl,
    robot: DualArmWholeBodyInterface,
    t: int,
    past_data: PastData,
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    BaseAndDualEEPathFollowingMPCControlLoop
    -----------------------------
    cart pulling dual arm control loop
    """
    p = robot.q[:2]

    raise NotImplementedError
    # TODO: first of all the mpc should optimize over the path parameter
    # and not points.
    # but if you do want to do it with points,
    # they should come from interpolation.
    # and this needs to be a function in motion planner.
    # since i don't need this now, i'll just leave it broken
    # until i get around to fixing optimal control overall
    # NOTE: this one is obtained as the future path from path planner
    max_base_v = np.linalg.norm(robot._max_v[:2])

    trajectorySE3_T_w_abs = pathSE3_to_trajectorySE3(cfg, path_T_w_abs, max_base_v)
    trajectorySE3_l = []
    trajectorySE3_r = []
    for traj_pose in trajectorySE3_T_w_abs:
        trajectorySE3_l.append(T_absgoal_l.act(traj_pose))
        trajectorySE3_r.append(T_absgoal_r.act(traj_pose))

    if cfg.visualizer:
        if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
            robot.visualizer_manager.sendCommand({"path": trajectory_base})
            robot.visualizer_manager.sendCommand({"frame_path": trajectorySE3_T_w_abs})

    x0 = np.concatenate([robot.q, robot.v])
    ocp.warmstartAndReSolve(
        x0, data=(trajectory_base, trajectorySE3_l, trajectorySE3_r)
    )
    xs = np.array(ocp.solver.xs)
    v_cmd = xs[1, robot.model.nq :]

    err_vector_ee_l = log6(robot.T_w_l.actInv(trajectorySE3_l[0]))
    err_norm_ee_l = np.linalg.norm(err_vector_ee_l)
    err_vector_ee_r = log6(robot.T_w_r.actInv(trajectorySE3_r[0]))
    err_norm_ee_r = np.linalg.norm(err_vector_ee_r)
    err_vector_base = np.linalg.norm(p - trajectory_base[0][:2])  # z axis is irrelevant
    log_item = {}
    log_item["err_vec_ee_l"] = err_vector_ee_l
    log_item["err_norm_ee_l"] = err_norm_ee_l.reshape((1,))
    log_item["err_vec_ee_r"] = err_vector_ee_r
    log_item["err_norm_ee_r"] = err_norm_ee_r.reshape((1,))
    log_item["err_norm_base"] = np.linalg.norm(err_vector_base).reshape((1,))
    save_past_item = {"path2D": p}
    return v_cmd, save_past_item, log_item


def BaseAndDualEEPathFollowingMPC(
    cfg: ConfigOptimalControl,
    robot: DualArmWholeBodyInterface,
    path_planner: CollaborativeProcess | types.FunctionType,
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    run=True,
) -> None | ControlLoopManager:
    """
    BaseAndDualEEPathFollowingMPC
    -------------------------------
    path following with 3 refereces: base, left arm, right arm.
    the path planner has to provide base path and T_w_abs path,
    and T_absgoal_l and T_absgoal_r from which the left and right
    references are constructed
    """
    robot._mode = DualArmWholeBodyInterface.control_mode.whole_body
    x0 = np.concatenate([robot.q, robot.v])
    ocp = BaseAndDualArmEEPathFollowingOCP(cfg, robot, x0)
    ocp.solveInitialOCP(x0)

    get_position = lambda robot: robot.q[:2]
    BaseAndDualEEPathFollowingMPCControlLoop_with_l_r = partial(
        BaseAndDualEEPathFollowingMPCControlLoop, T_absgoal_l, T_absgoal_r
    )
    controlLoop = partial(
        PathFollowingFromPlannerCtrllLoopTemplate,
        path_planner,
        get_position,
        ocp,
        BaseAndDualEEPathFollowingMPCControlLoop_with_l_r,
        cfg,
        robot,
    )

    log_item = {
        "qs": np.zeros(robot.model.nq),
        "vs": np.zeros(robot.model.nv),
        "err_vec_ee_l": np.zeros((6,)),
        "err_norm_ee_l": np.zeros((1,)),
        "err_vec_ee_r": np.zeros((6,)),
        "err_norm_ee_r": np.zeros((1,)),
        "err_norm_base": np.zeros((1,)),
    }
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, {}, log_item)

    if run:
        loop_manager.run()
    else:
        return loop_manager
