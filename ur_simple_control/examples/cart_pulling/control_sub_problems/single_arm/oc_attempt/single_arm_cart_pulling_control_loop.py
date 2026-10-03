from smc import load_config
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.control.control_loop_manager import ControlLoopManager
from smc.multiprocessing.process_manager import ProcessManager
from smc.control.optimal_control.abstract_croco_ocp import CrocoOCP
from smc.control.optimal_control.croco_path_following.mpc.base_and_single_arm_reference_mpc import (
    BaseAndEEPathFollowingOCP,
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

import numpy as np
from functools import partial
import types
from argparse import Namespace
from pinocchio import SE3, log6
from collections import deque


def initializePastData(
    cfg: MasterConfig, T_w_e: SE3, p_base: np.ndarray, max_base_v: float
) -> np.ndarray:
    # prepopulate past data to make base and cart be on the same path in the past
    # (which didn't actually happen because this just started)
    p_ee = T_w_e.translation[:2]
    straight_line_path = np.linspace(p_ee, p_base, cfg.past_window_size)
    # straight_line_path_timed = path2D_timed(cfg, straight_line_path, max_base_v)
    # return straight_line_path_timed # this one is shortened to cfg.n_knots! and we want the whole buffer
    return straight_line_path


def SingleArmCartPullingMPCControlLoop(
    ocp: CrocoOCP,
    path2D_base: np.ndarray,
    cfg: MasterConfig,
    robot: SingleArmWholeBodyInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    p = robot.T_w_b.translation[:2]

    # NOTE: this one is obtained as the future path from path planner
    max_base_v = np.linalg.norm(robot._max_v[:2])
    trajectory_base = path2D_to_trajectory2D(cfg, path2D_base, max_base_v)
    trajectory_base = np.hstack((trajectory_base, np.zeros((len(trajectory_base), 1))))

    trajectorySE3_handlebar = construct_EE_path(cfg, p, past_data["path2D"])

    if cfg.visualizer:
        if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
            robot.visualizer_manager.sendCommand({"path": trajectory_base})
            robot.visualizer_manager.sendCommand(
                {"frame_path": trajectorySE3_handlebar}
            )

    x0 = np.concatenate([robot.q, robot.v])
    ocp.warmstartAndReSolve(x0, data=(trajectory_base, trajectorySE3_handlebar))
    xs = np.array(ocp.solver.xs)
    v_cmd = xs[1, robot.model.nq :]

    # NOTE: if the base isn't moving, it's probably stuck in some local minimum
    # this happens with just the navigation reference
    if np.linalg.norm(v_cmd[:3]) < 0.05:
        print(t, "RESOLVING FOR ONLY FINAL PATH POINT")
        last_point_only = np.ones((len(trajectory_base), 2))
        last_point_only = np.hstack(
            (last_point_only, np.zeros((len(trajectory_base), 1)))
        )
        last_point_only = last_point_only * trajectory_base[-1]
        ocp.warmstartAndReSolve(x0, data=(last_point_only))
        xs = np.array(ocp.solver.xs)
        v_cmd = xs[1, robot.model.nq :]

    err_vector_ee = log6(robot.T_w_e.actInv(trajectorySE3_handlebar[0]))
    err_vector_base = np.linalg.norm(p - trajectory_base[0][:2])  # z axis is irrelevant
    log_item = {}
    log_item["err_vec_ee"] = err_vector_ee
    log_item["err_norm_ee"] = np.linalg.norm(err_vector_ee).reshape((1,))
    log_item["err_norm_base"] = np.linalg.norm(err_vector_base).reshape((1,))
    save_past_item = {"path2D": p}
    return v_cmd, save_past_item, log_item


def SingleArmCartPullingMPC(
    cfg: MasterConfig,
    robot: SingleArmWholeBodyInterface,
    path_planner: ProcessManager | types.FunctionType,
    run=True,
) -> None | ControlLoopManager:
    """
    BaseAndEEPathFollowingMPC
    -----
    run mpc for a point-to-point inverse kinematics.
    note that the actual problem is solved on
    a dynamics level, and velocities we command
    are actually extracted from the state x(q,dq).
    """

    robot._mode = SingleArmWholeBodyInterface.control_mode.whole_body
    T_w_e = robot.T_w_e
    x0 = np.concatenate([robot.q, robot.v])
    ocp = BaseAndEEPathFollowingOCP(cfg, robot, x0)
    ocp.solveInitialOCP(x0)

    max_base_v = np.linalg.norm(robot._max_v[:2])

    path2D_handlebar = initializePastData(cfg, T_w_e, robot.q[:2], float(max_base_v))

    get_position = lambda robot: robot.q[:2]
    controlLoop = partial(
        PathFollowingFromPlannerCtrllLoopTemplate,
        path_planner,
        get_position,
        ocp,
        SingleArmCartPullingMPCControlLoop,
        cfg,
        robot,
    )
    log_item = {
        "qs": np.zeros(robot.model.nq),
        "vs": np.zeros(robot.model.nv),
        "err_vec_ee": np.zeros((6,)),
        "err_norm_ee": np.zeros((1,)),
        "err_norm_base": np.zeros((1,)),
    }
    save_past_dict = {"path2D": T_w_e.translation[:2]}
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
