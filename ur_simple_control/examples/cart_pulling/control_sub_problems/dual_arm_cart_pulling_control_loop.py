from smc import load_config
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.control.control_loop_manager import ControlLoopManager
from smc.multiprocessing.process_manager import ProcessManager
from smc.control.optimal_control.abstract_croco_ocp import CrocoOCP
from smc.control.optimal_control.croco_path_following.ocp.base_and_dual_arm_reference_ocp import (
    BaseAndDualArmEEPathFollowingOCP,
)
from smc.path_generation.path_math.cart_pulling_path_math import (
    construct_EE_path,
)
from smc.path_generation.path_math.path_to_trajectory import (
    path2D_to_trajectory2D,
)
from smc.control.controller_templates.path_following_template import (
    PathFollowingFromPlannerCtrllLoopTemplate,
)
from smc.path_generation.path_math.path2d_to_6d import (
    path2D_to_SE3,
)
from smc.control.cartesian_space.ik_solvers import getIKSolver, dampedPseudoinverse

import numpy as np
from functools import partial
import types
from argparse import Namespace
from pinocchio import SE3, log6
from collections import deque
from utils import initializePastData
import pinocchio as pin


def DualArmCartPullingControlLoop(
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    ik_solver,
    path2D_base: np.ndarray,
    cfg: MasterConfig,
    robot: DualArmWholeBodyInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    BaseAndDualEEPathFollowingMPCControlLoop
    -----------------------------
    cart pulling dual arm control loop
    """
    v_cmd = np.zeros(robot.model.nv)
    p = robot.T_w_b.translation[:2]

    max_base_v = np.linalg.norm(robot._max_v[:2])
    # NOTE: can't initialize a priori now because we're not running immediately,
    # hence nothing is initialized
    if t == 0:
        path2D_handlebar = initializePastData(
            cfg, robot.T_w_abs, robot.T_w_b.translation[:2], float(max_base_v)
        )
        past_data["path2D"].clear()
        past_data["path2D"].extend(
            path2D_handlebar[i] for i in range(cfg.past_window_size)
        )
    trajectory_base = path2D_to_trajectory2D(cfg, path2D_base, max_base_v)
    trajectory_base = np.hstack((trajectory_base, np.zeros((len(trajectory_base), 1))))

    trajectorySE3_handlebar = construct_EE_path(cfg, p, past_data["path2D"])
    trajectorySE3_l = []
    trajectorySE3_r = []
    for traj_pose in trajectorySE3_handlebar:
        # trajectorySE3_l.append(T_absgoal_l.act(traj_pose))
        # trajectorySE3_r.append(T_absgoal_r.act(traj_pose))
        trajectorySE3_l.append(traj_pose.act(T_absgoal_l))
        trajectorySE3_r.append(traj_pose.act(T_absgoal_r))

    if cfg.visualizer:
        if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
            robot.visualizer_manager.sendCommand({"path": trajectory_base})
            robot.visualizer_manager.sendCommand(
                {"frame_path": trajectorySE3_handlebar}
            )

    # CONTROL BASE
    robot.mode = robot.control_mode.base_only
    trajectory_base_SE3 = path2D_to_SE3(trajectory_base[:, :2], 0.0, 0.0)
    rotm_z = pin.rpy.rpyToMatrix(0.0, 0.0, np.pi)
    for z in range(len(trajectory_base_SE3)):
        trajectory_base_SE3[z].rotation = rotm_z @ trajectory_base_SE3[z].rotation

    SEerror = robot.T_w_e.actInv(trajectory_base_SE3[-1])
    err_vector = pin.log6(SEerror).vector
    if np.linalg.norm(err_vector) < 0.2:
        V_path = (
            5 * pin.log6(trajectory_base_SE3[1].actInv(trajectory_base_SE3[2])).vector
        )
        err_vector += V_path
    err_vector[3:] = err_vector[3:] * 2
    J = robot.getJacobian()
    v_cmd_base = ik_solver(cfg, robot, err_vector)
    if v_cmd_base is None:
        v_cmd_base = dampedPseudoinverse(cfg, robot, err_vector)
    v_cmd[:3] = v_cmd_base

    # CONTROL ARMS
    robot._mode = robot.control_mode.upper_body
    SEerror_left = robot.T_w_l.actInv(trajectorySE3_l[0])
    SEerror_right = robot.T_w_r.actInv(trajectorySE3_r[0])
    err_vector_left = pin.log6(SEerror_left).vector
    err_vector_right = pin.log6(SEerror_right).vector

    err_vector = np.concatenate((err_vector_left, err_vector_right))
    J = robot.getJacobian()

    v_cmd_arms = ik_solver(cfg, robot, err_vector)
    if v_cmd_arms is None:
        v_cmd_arms = dampedPseudoinverse(cfg, robot, err_vector)
    v_cmd[3:] = v_cmd_arms

    robot._mode = robot.control_mode.whole_body
    err_vector_ee_l = log6(robot.T_w_l.actInv(trajectorySE3_l[0]))
    err_norm_ee_l = np.linalg.norm(err_vector_ee_l)
    err_vector_ee_r = log6(robot.T_w_r.actInv(trajectorySE3_r[0]))
    err_norm_ee_r = np.linalg.norm(err_vector_ee_r)
    err_vector_base = np.linalg.norm(p - trajectory_base[0][:2])  # z axis is irrelevant
    log_item = {}
    log_item["err_vec_ee_l"] = err_vector_ee_l.vector
    log_item["err_norm_ee_l"] = err_norm_ee_l.reshape((1,))
    log_item["err_vec_ee_r"] = err_vector_ee_r.vector
    log_item["err_norm_ee_r"] = err_norm_ee_r.reshape((1,))
    log_item["err_norm_base"] = np.linalg.norm(err_vector_base).reshape((1,))
    save_past_item = {"path2D": p}
    return v_cmd, save_past_item, log_item


def DualArmCartPulling(
    cfg: MasterConfig,
    robot: DualArmWholeBodyInterface,
    path_planner: ProcessManager | types.FunctionType,
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    run=True,
) -> None | ControlLoopManager:
    """
    CrocoEndEffectorPathFollowingMPC
    -----
    run mpc for a point-to-point inverse kinematics.
    note that the actual problem is solved on
    a dynamics level, and velocities we command
    are actually extracted from the state x(q,dq).
    """
    robot._mode = robot.control_mode.whole_body
    # NOTE: i'm shoving these into the class here - bad style,
    # but i don't
    T_w_abs = robot.T_w_abs
    ik_solver = getIKSolver(cfg, robot)

    max_base_v = np.linalg.norm(robot._max_v[:2])

    path2D_handlebar = initializePastData(
        cfg, T_w_abs, robot.T_w_b.translation[:2], float(max_base_v)
    )

    get_position = lambda robot: robot.q[:2]
    DualArmCartPullingControlLoop_with_l_r = partial(
        DualArmCartPullingControlLoop, T_absgoal_l, T_absgoal_r
    )
    controlLoop = partial(
        PathFollowingFromPlannerCtrllLoopTemplate,
        path_planner,
        get_position,
        ik_solver,
        DualArmCartPullingControlLoop_with_l_r,
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
    save_past_dict = {"path2D": T_w_abs.translation[:2]}
    loop_manager = ControlLoopManager(
        robot, controlLoop, cfg, save_past_dict, log_item
    )
    # actually put past data into the past window
    loop_manager.past_data["path2D"].clear()
    loop_manager.past_data["path2D"].extend(
        path2D_handlebar[i] for i in range(cfg.past_window_size)
    )

    if run:
        loop_manager.run()
    else:
        return loop_manager


def DualArmCartPullingMPCControlLoop(
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    ocp: CrocoOCP,
    path2D_base: np.ndarray,
    cfg: MasterConfig,
    robot: DualArmWholeBodyInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    BaseAndDualEEPathFollowingMPCControlLoop
    -----------------------------
    cart pulling dual arm control loop
    """
    p = robot.T_w_b.translation[:2]

    # NOTE: this one is obtained as the future path from path planner
    max_base_v = np.linalg.norm(robot._max_v[:2])
    trajectory_base = path2D_to_trajectory2D(cfg, path2D_base, max_base_v)
    trajectory_base = np.hstack((trajectory_base, np.zeros((len(trajectory_base), 1))))

    trajectorySE3_handlebar = construct_EE_path(cfg, p, past_data["path2D"])
    trajectorySE3_l = []
    trajectorySE3_r = []
    for traj_pose in trajectorySE3_handlebar:
        trajectorySE3_l.append(T_absgoal_l.act(traj_pose))
        trajectorySE3_r.append(T_absgoal_r.act(traj_pose))

    if cfg.visualizer:
        if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
            robot.visualizer_manager.sendCommand({"path": trajectory_base})
            robot.visualizer_manager.sendCommand(
                {"frame_path": trajectorySE3_handlebar}
            )

    x0 = np.concatenate([robot.q, robot.v])
    ocp.warmstartAndReSolve(
        x0, data=(trajectory_base, trajectorySE3_l, trajectorySE3_r)
    )
    xs = np.array(ocp.solver.xs)
    v_cmd = xs[1, robot.model.nq :]

    if np.linalg.norm(v_cmd[:3]) < 0.05:
        print(t, "RESOLVING FOR ONLY FINAL PATH POINT")
        last_point_only = np.ones((len(trajectory_base), 2))
        last_point_only = np.hstack(
            (last_point_only, np.zeros((len(trajectory_base), 1)))
        )
        last_point_only = last_point_only * trajectory_base[-1]
        ocp.warmstartAndReSolve(
            x0, data=(last_point_only, trajectorySE3_l, trajectorySE3_r)
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


def DualArmCartPullingMPC(
    cfg: MasterConfig,
    robot: DualArmWholeBodyInterface,
    path_planner: ProcessManager | types.FunctionType,
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    run=True,
) -> None | ControlLoopManager:
    """
    CrocoEndEffectorPathFollowingMPC
    -----
    run mpc for a point-to-point inverse kinematics.
    note that the actual problem is solved on
    a dynamics level, and velocities we command
    are actually extracted from the state x(q,dq).
    """
    robot._mode = robot.control_mode.whole_body
    # NOTE: i'm shoving these into the class here - bad style,
    # but i don't
    T_w_abs = robot.T_w_abs
    x0 = np.concatenate([robot.q, robot.v])
    ocp = BaseAndDualArmEEPathFollowingOCP(cfg, robot, x0)
    ocp.solveInitialOCP(x0)

    max_base_v = np.linalg.norm(robot._max_v[:2])

    path2D_handlebar = initializePastData(cfg, T_w_abs, robot.q[:2], float(max_base_v))

    get_position = lambda robot: robot.q[:2]
    DualArmCartPullingControlLoop_with_l_r = partial(
        DualArmCartPullingMPCControlLoop, T_absgoal_l, T_absgoal_r
    )
    controlLoop = partial(
        PathFollowingFromPlannerCtrllLoopTemplate,
        path_planner,
        get_position,
        ocp,
        DualArmCartPullingControlLoop_with_l_r,
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
    save_past_dict = {"path2D": T_w_abs.translation[:2]}
    loop_manager = ControlLoopManager(
        robot, controlLoop, cfg, save_past_dict, log_item
    )
    # actually put past data into the past window
    loop_manager.past_data["path2D"].clear()
    loop_manager.past_data["path2D"].extend(
        path2D_handlebar[i] for i in range(cfg.past_window_size)
    )

    if run:
        loop_manager.run()
    else:
        return loop_manager
