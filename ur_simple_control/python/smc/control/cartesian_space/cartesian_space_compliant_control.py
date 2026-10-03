from smc.bookkeeping.load_config import GlobalConfig
from smc.control.cartesian_space.cartesian_space_compliant_control_config import (
    ConfigCartesianSpaceCompliant,
)
from smc.control.cartesian_space.cartesian_space_config import (
    IKSolver,
)
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.controller_templates.point_to_point import EEP2PCtrlLoopTemplate
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.control.cartesian_space.ik_solvers import *
from smc.control.control_typing import InnerControlLoopReturn

from functools import partial
from pinocchio import SE3, log6
import numpy as np
import copy
from collections import deque


def controlLoopCompliantClik(
    ik_solver: IKSolver,
    T_w_goal: SE3,
    cfg: GlobalConfig,
    robot: ForceTorqueOnSingleArmWrist,
    _: int,
    past_data: dict[str, deque[np.ndarray]],
) -> InnerControlLoopReturn:
    """
    controlLoopCompliantClik
    ---------------
    CLIK with compliance in all directions
    """
    save_past_item = {}
    log_item = {}
    T_w_e = robot.T_w_e
    wrench = robot.wrench

    # we need to overcome noise if we want to converge
    # TODO: bad because you it introduces osciallations
    # and it doesn't work for higher accelerations
    # if np.linalg.norm(wrench) < cfg.minimum_detectable_force_norm:
    #    wrench = np.zeros(6)
    save_past_item["wrench"] = copy.deepcopy(wrench)

    # low pass filter for wrenches
    wrench = cfg.compliance.beta * wrench + (1 - cfg.compliance.beta) * np.average(
        np.array(past_data["wrench"]), axis=0
    )
    if not cfg.compliance.z_only:
        Z = np.diag(np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]))
    else:
        Z = np.diag(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    wrench = Z @ wrench

    SEerror = T_w_e.actInv(T_w_goal)
    err_vector = log6(SEerror).vector

    v_cartesian_body_cmd = (
        cfg.compliance.Kp * err_vector + cfg.compliance.alpha * wrench
    )
    v_cmd = ik_solver(cfg.cs, robot, v_cartesian_body_cmd)

    log_item["wrench"] = robot.wrench
    log_item["wrench_used"] = wrench
    return v_cmd, save_past_item, log_item


# add a threshold for the wrench
def compliantMoveL(
    cfg: GlobalConfig,
    robot: ForceTorqueOnSingleArmWrist,
    T_w_goal: SE3,
    run=True,
) -> None | ControlLoopManager:
    """
    compliantMoveL
    -----
    does compliantMoveL - a moveL, but with compliance achieved
    through f/t feedback
    send a SE3 object as goal point.
    if you don't care about rotation, make it np.zeros((3,3))
    """
    assert type(T_w_goal) is SE3
    ik_solver = getIKSolver(cfg.cs, robot)
    controlLoop = partial(
        EEP2PCtrlLoopTemplate,
        ik_solver,
        T_w_goal,
        controlLoopCompliantClik,
        cfg,
        robot,
    )
    # we're not using any past data or logging, hence the empty arguments
    log_item = {
        "qs": np.zeros(robot.model.nq),
        "err_norm": np.zeros(1),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "wrench": np.zeros(6),
        "wrench_used": np.zeros(6),
    }
    save_past_dict = {
        "wrench": np.zeros(6),
    }
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager
