from smc import load_config
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.control.control_loop_manager import ControlLoopManager
from smc.multiprocessing.process_manager import ProcessManager
from smc.control.optimal_control.abstract_croco_ocp import CrocoOCP
from smc.control.optimal_control.croco_path_following.mpc.base_reference_mpc import (
    BasePathFollowingOCP,
)
from smc.path_generation.path_math.cart_pulling_path_math import (
    construct_EE_path,
)
from smc.path_generation.path_math.path_to_trajectory import path2D_to_trajectory2D
from smc.control.controller_templates.path_following_template import (
    PathFollowingFromPlannerCtrllLoopTemplate,
)
from smc.control.cartesian_space.ik_solvers import dampedPseudoinverse

from utils import initializePastData

import numpy as np
from functools import partial
import types
from argparse import Namespace
from pinocchio import SE3, log6
from collections import deque
from IPython import embed


def BaseMPCEECLIKPathFollowingFromPlannerMPCControlLoop(
    ocp: CrocoOCP,
    path2D_untimed_base: np.ndarray,
    cfg: MasterConfig,
    robot: SingleArmWholeBodyInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:

    robot._mode = SingleArmWholeBodyInterface.control_mode.whole_body
    p = robot.T_w_b.translation[:2]
    max_base_v = np.linalg.norm(robot._max_v[:2])
    path_base = path2D_to_trajectory2D(cfg, path2D_untimed_base, max_base_v)
    path_base = np.hstack((path_base, np.zeros((len(path_base), 1))))

    x0 = np.concatenate([robot.q, robot.v])
    ocp.warmstartAndReSolve(x0, data=(path_base))
    xs = np.array(ocp.solver.xs)
    v_cmd = xs[1, robot.model.nq :]

    pathSE3_handlebar = construct_EE_path(cfg, p, past_data["path2D_untimed"])
    robot._mode = SingleArmWholeBodyInterface.control_mode.upper_body

    T_w_e = robot.T_w_e
    # first check whether we're at the goal
    SEerror = T_w_e.actInv(pathSE3_handlebar[0])
    err_vector = log6(SEerror).vector
    J = robot.getJacobian()
    # compute the joint velocities based on controller you passed
    # qd = ik_solver(cfg, robot, err_vector, past_qd=past_data['vs_cmd'][-1])
    v_arm = dampedPseudoinverse(cfg, robot, err_vector)
    robot._mode = SingleArmWholeBodyInterface.control_mode.whole_body

    v_cmd[3:] = v_arm

    #    if cfg.visualizer:
    #        if t % cfg.viz_update_rate == 0:
    #            robot.visualizer_manager.sendCommand({"path": path_base})
    #            robot.visualizer_manager.sendCommand({"frame_path": pathSE3_handlebar})

    err_vector_ee = log6(robot.T_w_e.actInv(pathSE3_handlebar[0]))
    err_vector_base = np.linalg.norm(p - path_base[0][:2])  # z axis is irrelevant
    log_item = {}
    log_item["err_vec_ee"] = err_vector_ee
    log_item["err_norm_ee"] = np.linalg.norm(err_vector_ee).reshape((1,))
    log_item["err_norm_base"] = np.linalg.norm(err_vector_base).reshape((1,))
    return v_cmd, {"path2D_untimed": p}, log_item


def BaseMPCANDEECLIKCartPulling(
    cfg: MasterConfig,
    robot: SingleArmWholeBodyInterface,
    path_planner: ProcessManager | types.FunctionType,
) -> None:
    """
    BaseAndEEPathFollowingMPC
    -----
    run mpc for a point-to-point inverse kinematics.
    note that the actual problem is solved on
    a dynamics level, and velocities we command
    are actually extracted from the state x(q,dq).
    """

    T_w_e = robot.T_w_e
    x0 = np.concatenate([robot.q, robot.v])
    ocp = BasePathFollowingOCP(cfg, robot, x0)
    ocp.solveInitialOCP(x0)

    max_base_v = np.linalg.norm(robot._max_v[:2])

    path2D_handlebar = initializePastData(cfg, T_w_e, robot.q[:2], float(max_base_v))

    if type(path_planner) == types.FunctionType:
        raise NotImplementedError
    else:
        get_position = lambda robot: robot.q[:2]
        controlLoop = partial(
            PathFollowingFromPlannerCtrllLoopTemplate,
            path_planner,
            get_position,
            ocp,
            BaseMPCEECLIKPathFollowingFromPlannerMPCControlLoop,
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
    save_past_dict = {"path2D_untimed": T_w_e.translation[:2]}
    loop_manager = ControlLoopManager(
        robot, controlLoop, cfg, save_past_dict, log_item
    )

    # actually put past data into the past window
    loop_manager.past_data["path2D_untimed"].clear()
    loop_manager.past_data["path2D_untimed"].extend(
        path2D_handlebar[i] for i in range(cfg.past_window_size)
    )

    loop_manager.run()
