from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
from smc.bookkeeping.load_config import GlobalConfig
from smc.control.control_loop_manager import ControlLoopManager
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.control.controller_templates.point_to_point import (
    EEP2PCtrlLoopTemplate,
    DualEEP2PCtrlLoopTemplate,
)
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.control.cartesian_space.ik_solvers import getIKSolver, dampedPseudoinverse
from functools import partial
from pinocchio import SE3, log6
import numpy as np
from collections import deque
from typing import Callable


def controlLoopClik(
    ik_solver: Callable[
        [ConfigCartesianSpace, SingleArmInterface, np.ndarray], np.ndarray
    ],
    T_w_goal: SE3,
    cfg: GlobalConfig,
    robot: SingleArmInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    controlLoopClik
    ---------------
    generic control loop for clik (handling error to final point etc).
    in some version of the universe this could be extended to a generic
    point-to-point motion control loop.
    """
    SEerror = robot.T_w_e.actInv(T_w_goal)
    err_vector = log6(SEerror).vector
    # compute the joint velocities based on controller you passed
    # qd = ik_solver(cfg, robot, err_vector, past_qd=past_data['vs_cmd'][-1])
    v_cmd = ik_solver(cfg.cs, robot, err_vector)
    if v_cmd is None:
        print(
            t,
            "the controller you chose produced None as output, using dampedPseudoinverse instead",
        )
        v_cmd = dampedPseudoinverse(cfg.cs, robot, err_vector)

    return v_cmd, {}, {}


def moveL(
    cfg: GlobalConfig, robot: SingleArmInterface, T_w_goal: SE3, run=True
) -> None | ControlLoopManager:
    """
    moveL
    -----
    does moveL.
    send a SE3 object as goal point.
    if you don't care about rotation, make it np.zeros((3,3))
    """
    assert type(T_w_goal) is SE3
    ik_solver = getIKSolver(cfg.cs, robot)
    controlLoop = partial(
        EEP2PCtrlLoopTemplate, ik_solver, T_w_goal, controlLoopClik, cfg, robot
    )
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "err_norm": np.zeros(1),
    }
    save_past_dict = {}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


def moveUntilContactControlLoop(
    cfg: GlobalConfig,
    robot: ForceTorqueOnSingleArmWrist,
    speed: np.ndarray,
    ik_solver: Callable[
        [ConfigCartesianSpace, SingleArmInterface, np.ndarray], np.ndarray
    ],
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    moveUntilContactControlLoop
    ---------------
    generic control loop for clik (handling error to final point etc).
    in some version of the universe this could be extended to a generic
    point-to-point motion control loop.
    """
    breakFlag = False
    # know where you are, i.e. do forward kinematics
    log_item = {}
    q = robot.q
    # break if wrench is nonzero basically
    # wrench = robot.getWrench()
    # you're already giving the speed in the EE i.e. body frame
    # so it only makes sense to have the wrench in the same frame
    # wrench = robot._getWrenchInEE()
    wrench = robot.wrench
    # and furthermore it's a reasonable assumption that you'll hit the thing
    # in the direction you're going in.
    # thus we only care about wrenches in those direction coordinates
    mask = speed != 0.0
    # NOTE: contact getting force is a magic number
    # it is a 100% empirical, with the goal being that it's just above noise.
    # so far it's worked fine, and it's pretty soft too.
    if np.linalg.norm(wrench[mask]) > cfg.contact_detecting_force:
        print("hit with", np.linalg.norm(wrench[mask]))
        breakFlag = True
        robot.sendVelocityCommand(np.zeros(robot.nv))
    if (not cfg.real) and (i > 500):
        print("let's say you hit something lule")
        breakFlag = True
    qd = ik_solver(cfg.cs, robot, speed)
    robot.sendVelocityCommand(qd)
    log_item["qs"] = q.reshape((robot.nq,))
    log_item["wrench"] = wrench.reshape((6,))
    return breakFlag, {}, log_item


def moveUntilContact(
    cfg: GlobalConfig, robot: ForceTorqueOnSingleArmWrist, speed: np.ndarray
) -> None:
    """
    moveUntilContact
    ----------------
    does clik until it feels something with the f/t sensor
    """
    assert type(speed) is np.ndarray
    ik_solver = getIKSolver(cfg.cs, robot)
    controlLoop = partial(moveUntilContactControlLoop, cfg, robot, speed, ik_solver)
    # we're not using any past data or logging, hence the empty arguments
    log_item = {"wrench": np.zeros(6)}
    log_item["qs"] = np.zeros((robot.nq,))
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, {}, log_item)
    loop_manager.run()
    print("Collision detected!!")


def controlLoopClikDualArm(
    ik_solver: Callable[
        [ConfigCartesianSpace, SingleArmInterface, np.ndarray], np.ndarray
    ],
    T_w_absgoal: SE3,
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    cfg: GlobalConfig,
    robot: DualArmInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    controlLoopClikDualArm
    ---------------
    do point to point motion for each arm and its goal.
    that goal is generated from a single goal that you pass,
    and an SE3  transformation on the goal for each arm
    """

    T_w_lgoal = T_w_absgoal.act(T_absgoal_l)
    T_w_rgoal = T_w_absgoal.act(T_absgoal_r)

    SEerror_left = robot.T_w_l.actInv(T_w_lgoal)
    SEerror_right = robot.T_w_r.actInv(T_w_rgoal)

    err_vector_left = log6(SEerror_left).vector
    err_vector_right = log6(SEerror_right).vector

    err_vector = np.concatenate((err_vector_left, err_vector_right))

    v_cmd = ik_solver(cfg.cs, robot, err_vector)
    if v_cmd is None:
        print(
            t,
            "the ik solver you chose produced None as output, using dampedPseudoinverse instead",
        )
        v_cmd = dampedPseudoinverse(cfg.cs, robot, err_vector)
    # else:
    #    if cfg.debug_prints:
    #        print(t, "ik solver success")
    return v_cmd, {}, {}


def moveLDualArm(
    cfg: GlobalConfig,
    robot: DualArmInterface,
    T_w_goal: SE3,
    T_abs_l: SE3,
    T_abs_r: SE3,
    run=True,
) -> None | ControlLoopManager:
    """
    moveLDualArm
    -----------
    """
    ik_solver = getIKSolver(cfg.cs, robot)
    controlLoop = partial(
        DualEEP2PCtrlLoopTemplate,
        ik_solver,
        T_w_goal,
        T_abs_l,
        T_abs_r,
        controlLoopClikDualArm,
        cfg,
        robot,
    )
    # we're not using any past data or logging, hence the empty arguments
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "l_err_norm": np.zeros(1),
        "r_err_norm": np.zeros(1),
        "manipulability": np.zeros(1),
    }
    save_past_dict = {}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


# def moveLWTraj(
#    cfg: ConfigCartesianSpace, robot: SingleArmInterface, T_w_goal: SE3, run=True
# ) -> None | ControlLoopManager:
#    """
#    moveLWTraj
#    ------------------
#    make a path from current to goal position, i.e.
#    just a straight line between them.
#    the question is what to do with orientations.
#    i suppose it makes sense to have one function that enforces/assumes
#    that the start and end positions have the same orientation.
#    then another version goes in a line and linearly updates the orientation
#    as it goes
#    """
#    raise NotImplementedError
#    assert type(T_w_goal) is SE3
#    ik_solver = getIKSolver(cfg, robot)
#    getCurrentPathReference = partial(
#        SLERPPointToPointInterpolation, robot.T_w_e.copy(), T_w_goal.copy()
#    )
#    # TODO: obviously finish this
#    T_final = 3
#    time_law = 3333
#
#    # TODO: finish
#    # i want to solve an opt problem for to get the time_law
#    # everything else is bs
#
#    controlLoop = partial(
#        cartesianTrajectoryTrackingControlLoop,
#    )
#    log_item = {
#        "qs": np.zeros(robot.nq),
#        "vs": np.zeros(robot.nv),
#        "vs_cmd": np.zeros(robot.nv),
#        "err_norm": np.zeros(1),
#    }
#    save_past_dict = {}
#    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
#    if run:
#        loop_manager.run()
#    else:
#        return loop_manager
