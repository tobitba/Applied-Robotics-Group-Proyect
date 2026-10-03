from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.control.control_loop_manager import ControlLoopManager
from smc.bookkeeping.base_config import ConfigBase
from smc.control.control_typing import ControlLoopReturn
from smc.bookkeeping.types import PastData

import numpy as np
from functools import partial
from smc.control.joint_space.joint_space_trajectory_following import (
    followParametrizedJointTrajectoryControlLoop,
)


def moveJPControlLoop(
    q_desired: np.ndarray,
    cfg: ConfigBase,
    robot: AbstractRobotManager,
    i: int,
    past_data: PastData,
) -> ControlLoopReturn:
    """
    moveJPControlLoop
    ---------------
    most basic P control for joint space point-to-point motion, actual control loop.
    """
    breakFlag = False
    q = robot.q

    # TODO: make a robot output a different model depending on control mode
    # to avoid having the issue of this not working (pinocchio::model
    # only works for the full body. if you want a different mode,
    # you need a different model, end of story.
    # this would entail preparing pinocchio::models corresponding to each
    # control mode, and then returning them accordingly.
    # you can do this by cutting away from the whole-body model you load).
    # q_error = pin.difference(robot.model, q, q_desired)
    # (this doen't work for mobile bases and i couldn't be bothered to fix that rn)
    q_error = q_desired - q

    err_norm = np.linalg.norm(q_error)
    if err_norm < 1e-3:
        breakFlag = True
    K = 320
    qd = K * q_error * robot.dt
    robot.sendVelocityCommand(qd)
    log_item = {
        "qs": robot.q,
        "vs": robot.v,
        "vs_cmd": qd,
        "err_norm": err_norm.reshape((1,)),
    }
    return breakFlag, {}, log_item


def moveJP(
    q_desired: np.ndarray, cfg: ConfigBase, robot: AbstractRobotManager, run=True
) -> ControlLoopManager | None:
    """
    moveJP
    ---------------
    most basic P control for joint space point-to-point motion.
    just starts the control loop without any logging.
    """
    assert type(q_desired) is np.ndarray
    assert len(q_desired) == len(robot.q)
    controlLoop = partial(moveJPControlLoop, q_desired, cfg, robot)
    # we're not using any past data or logging, hence the empty arguments
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "err_norm": np.zeros((1,)),
    }
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, {}, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


# TODO: untested! untuned!
def moveJPDControlLoop(
    q_desired: np.ndarray,
    cfg: ConfigBase,
    robot: AbstractRobotManager,
    i: int,
    past_data: PastData,
) -> ControlLoopReturn:
    """
    moveJPDControlLoop
    ---------------
    most basic P control for joint space point-to-point motion, actual control loop.
    """
    breakFlag = False
    q = robot.q

    q_error = q_desired - q

    err_norm = np.linalg.norm(q_error)
    if err_norm < 1e-3:
        breakFlag = True
    Kp = 20
    Kd = np.sqrt(Kp)
    qd = (Kp * q_error + Kd * robot.v) * robot.dt
    robot.sendVelocityCommand(qd)
    log_item = {
        "qs": robot.q,
        "vs": robot.v,
        "vs_cmd": qd,
        "err_norm": err_norm.reshape((1,)),
    }
    return breakFlag, {}, log_item


# TODO: untested! untuned!
def moveJPD(
    q_desired: np.ndarray, cfg: ConfigBase, robot: AbstractRobotManager, run=True
) -> ControlLoopManager | None:
    """
    moveJP
    ---------------
    most basic P control for joint space point-to-point motion.
    just starts the control loop without any logging.
    """
    assert type(q_desired) is np.ndarray
    assert len(q_desired) == len(robot.q)
    controlLoop = partial(moveJPDControlLoop, q_desired, cfg, robot)
    # we're not using any past data or logging, hence the empty arguments
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "err_norm": np.zeros((1,)),
    }
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, {}, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


def moveJPIControlLoop(
    q_desired: np.ndarray,
    cfg: ConfigBase,
    robot: AbstractRobotManager,
    i: int,
    past_data: PastData,
) -> ControlLoopReturn:
    """
    PID control for joint space point-to-point motion with approximated joint velocities.
    """

    # ================================
    # Initialization
    # ================================
    breakFlag = False
    save_past_dict = {}
    log_item = {}

    # ================================
    # Current Joint Positions
    # ================================
    q = robot.q

    # ================================
    # Compute Position Error
    # ================================
    q_error = q_desired - q  # Position error

    # ================================
    # Check for Convergence
    # ================================
    if np.linalg.norm(q_error) < 1e-3 and np.linalg.norm(robot.v) < 1e-3:
        breakFlag = True

    # ================================
    # Update Integral of Error
    # ================================
    integral_error = past_data["integral_error"][-1]
    integral_error = np.array(integral_error, dtype=np.float64).flatten()
    integral_error += q_error * robot.dt  # Accumulate error over time

    # Anti-windup: Limit integral error to prevent excessive accumulation
    max_integral = 10
    integral_error = np.clip(integral_error, -max_integral, max_integral)

    # ================================
    # Save Current States for Next Iteration
    # ================================
    save_past_dict["integral_error"] = integral_error  # Save updated integral error
    save_past_dict["q_prev"] = q  # Save current joint positions
    save_past_dict["e_prev"] = q_error  # Save current position error

    # ================================
    # Control Gains
    # ================================
    Kp = 7.0  # Proportional gain
    Ki = 0.0  # Integral gain

    # ================================
    # Compute Control Input (Joint Velocities)
    # ================================
    v_cmd = Kp * q_error + Ki * integral_error

    # ================================
    # Send Joint Velocities to the Robot
    # ================================
    robot.sendVelocityCommand(v_cmd)

    log_item["qs"] = q
    log_item["vs"] = robot.v
    log_item["integral_error"] = integral_error.flatten()  # Save updated integral error
    log_item["e_prev"] = q_error.flatten()  # Save current position error
    return breakFlag, save_past_dict, log_item


def moveJPI(
    q_desired: np.ndarray, cfg: ConfigBase, robot: AbstractRobotManager, run=True
) -> ControlLoopManager | None:
    assert isinstance(q_desired, np.ndarray)
    controlLoop = partial(moveJPIControlLoop, q_desired, cfg, robot)

    initial_q = robot.q
    save_past_dict = {
        "integral_error": np.zeros(robot.nq),
        "q_prev": initial_q,
        "e_prev": q_desired - initial_q,
    }

    log_item = {
        "qs": np.zeros(6),
        "vs": np.zeros(6),
        "integral_error": np.zeros(robot.nq),
        "e_prev": q_desired - initial_q,
    }

    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()

        if cfg.debug_prints:
            print("MoveJPI done: convergence achieved, reached destination!")
    else:
        return loop_manager


def moveJPWTraj(
    q_desired: np.ndarray, cfg: ConfigBase, robot: AbstractRobotManager, run=True
) -> ControlLoopManager | None:
    # TODO: throw an error if you're trying to run this
    # on a mobile base, or use pin.diff

    # most basic 3rd degree as we're only going to one point
    # s(t) = a_0 + a_t * t + a_2 * t**2 + a_3 * t**3
    # s_dot(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t**2

    # 1. heuristic estimate for total time T
    diff = np.abs(q_desired - robot.q)
    max_diff = np.max(diff)
    i_max = np.argmax(diff)
    v_max = robot.max_v[i_max]
    # s = t * v
    # t = s/v
    T_final = (max_diff / v_max) / 0.8

    # determine trajectory coefficients
    q_0 = robot.q

    # TODO: refactor with smc/trajectory_generation/cubic_traj.py
    q_ref = lambda t, q_0=q_0, q_desired=q_desired, T=T_final: q_0 + (
        ((3 * t**2) / T**2) - (2 * t**3) / T**3
    ) * (q_desired - q_0)
    v_ref = lambda t, q_0=q_0, q_desired=q_desired, T=T_final: (
        (6 * t) / T**2 - (6 * t**2) / T**3
    ) * (q_desired - q_0)

    control_loop = partial(
        followParametrizedJointTrajectoryControlLoop, cfg, robot, q_ref, v_ref, T_final
    )
    log_item = {
        "error_qs": np.zeros(robot.nq),
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "q_ref": np.zeros(robot.nq),
        "v_ref": np.zeros(robot.nv),
    }
    loop_manager = ControlLoopManager(robot, control_loop, cfg, {}, log_item)
    if run:
        loop_manager.run()

        if cfg.debug_prints:
            print("MoveJPI done: convergence achieved, reached destination!")
    else:
        return loop_manager
