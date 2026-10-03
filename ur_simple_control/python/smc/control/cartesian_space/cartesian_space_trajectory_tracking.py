from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.types import CartesianReference
from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
from smc.control.control_loop_manager import ControlLoopManager
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.control.cartesian_space.ik_solvers import getIKSolver
from smc.motion_planning.motion_planner import AbstractMotionPlanner

from functools import partial
from pinocchio import SE3, log6, rpy
import numpy as np
from collections import deque
from typing import Callable


def cartesianTrajectoryTrackingControlLoop(
    motion_planner: AbstractMotionPlanner[CartesianReference],
    ik_solver: Callable[
        [ConfigCartesianSpace, SingleArmInterface, np.ndarray], np.ndarray
    ],
    cfg: ConfigCartesianSpace,
    robot: SingleArmInterface,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    breakFlag = False
    log_item = {}
    save_past_item = {}

    # past data has to be a dict of deques of arrays
    s = past_data["s"][-1][0]
    t = i * robot.dt
    # V_s is expressed in the T_w_s frame!
    # since T_w_e is basically the same frame,
    # this is the velocity to be used with the body jacobian - our default.
    T_w_s, V_s, _ = motion_planner.getReference(t)
    J = robot.getJacobian()

    # feedforward control
    # feedback path position error, feedforward path derivative
    ee_fb_err = log6(robot.T_w_e.actInv(T_w_s)).vector
    V_cmd = cfg.K_fb * ee_fb_err + V_s
    v_cmd = ik_solver(cfg, robot, V_cmd)

    if s >= 1.0:
        v_cmd = np.zeros(robot.nv)
        breakFlag = True

    robot.sendVelocityCommand(v_cmd)

    save_past_item["s"] = np.array([s])
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["s"] = np.array([s]).reshape((1,))
    log_item["manipulability"] = robot.computeManipulabilityIndex().reshape((1,))
    log_item["ee_pose_error"] = ee_fb_err
    log_item["ee_velocity_error"] = V_s - J @ robot.v
    if issubclass(robot.__class__, SingleArmWholeBodyInterface):
        T_b_e = robot.T_w_b.actInv(robot.T_w_e)
        ebd = T_b_e.translation.copy()
        ebd[2] = rpy.matrixToRpy(T_b_e.rotation)[2]
        log_item["ebd_dist"] = ebd
    return breakFlag, save_past_item, log_item


def cartesianTrajectoryTracking(
    motion_planner: AbstractMotionPlanner[CartesianReference],
    cfg: GlobalConfig,
    robot: SingleArmInterface,
    run=True,
) -> ControlLoopManager | None:
    """
    fixedTimingFixedPathSingleArmCartesianPathFollowing
    ---------------------------------------------------
    You provide a Cartesian path to be followed by the robot's end-effector
    in the format of a list of SE(3) points.
    IT IS EXPECTED THE ROBOT IS AT THE BEGINNING OF THE PATH
    This function:
    1) uses an interpolated path (C^2 smooth line)
    2) uses a time law imposed on the interpolated path
    3) runs/returns a position-feedback, velocity-feedforward clik controller for the trajectory
    """
    ik_solver = getIKSolver(cfg.cs, robot)
    save_past_item = {"s": np.zeros(1)}
    log_item = {}
    log_item["qs"] = np.zeros(robot.nq)
    log_item["vs"] = np.zeros(robot.nv)
    log_item["s"] = np.zeros(
        1,
    )
    log_item["manipulability"] = np.zeros(
        1,
    )
    log_item["ee_pose_error"] = np.zeros(6)
    log_item["ee_velocity_error"] = np.zeros(6)
    if issubclass(robot.__class__, SingleArmWholeBodyInterface):
        log_item["ebd_dist"] = np.zeros(3)

    controlLoop = partial(
        cartesianTrajectoryTrackingControlLoop,
        motion_planner,
        ik_solver,
        cfg.cs,
        robot,
    )
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_item, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


def cartesianDualArmTrajectoryTrackingControlLoop(
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    motion_planner: AbstractMotionPlanner[CartesianReference],
    ik_solver: Callable[
        [ConfigCartesianSpace, SingleArmInterface, np.ndarray], np.ndarray
    ],
    cfg: ConfigCartesianSpace,
    robot: DualArmInterface,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    breakFlag = False
    log_item = {}
    save_past_item = {}

    # past data has to be a dict of deques of arrays
    s = past_data["s"][-1][0]
    t = i * robot.dt
    # V_s is expressed in the T_w_s frame!
    # since T_w_e is basically the same frame,
    # this is the velocity to be used with the body jacobian - our default.
    T_w_s, V_s, _ = motion_planner.getReference(t)

    # feedforward control
    # feedback path position error, feedforward path derivative
    T_w_s_l = T_w_s.act(T_absgoal_l)
    V_fb_cmd_left = log6(robot.T_w_l.actInv(T_w_s_l)).vector
    T_w_s_r = T_w_s.act(T_absgoal_r)
    V_fb_cmd_right = log6(robot.T_w_r.actInv(T_w_s_r)).vector

    V_fb_cmd_arms = np.concatenate((V_fb_cmd_left, V_fb_cmd_right))
    V_s_arms = np.concatenate((V_s, V_s))
    V_cmd_arms = cfg.K_fb * V_fb_cmd_arms + V_s_arms

    v_cmd = ik_solver(cfg, robot, V_cmd_arms)

    if s >= 1.0:
        v_cmd = np.zeros(robot.nv)
        breakFlag = True

    robot.sendVelocityCommandToReal(v_cmd)

    save_past_item["s"] = np.array([s])
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["s"] = np.array([s]).reshape((1,))
    log_item["manipulability"] = robot.computeManipulabilityIndex().reshape((1,))
    log_item["ee_pose_error"] = V_fb_cmd_arms
    log_item["ee_velocity_error"] = V_s_arms - robot.getJacobian() @ robot.v
    if issubclass(robot.__class__, DualArmWholeBodyInterface):
        T_b_e = robot.T_w_b.actInv(robot.T_w_e)
        ebd = T_b_e.translation.copy()
        # TODO: avoid rpy pls, we know better
        ebd[2] = rpy.matrixToRpy(T_b_e.rotation)[2]
        log_item["ebd_dist"] = ebd

    return breakFlag, save_past_item, log_item


def cartesianDualArmTrajectoryTracking(
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    motion_planner: AbstractMotionPlanner[CartesianReference],
    cfg: GlobalConfig,
    robot: DualArmInterface,
    run=True,
) -> ControlLoopManager | None:
    """
    fixedTimingFixedPathSingleArmCartesianPathFollowing
    ---------------------------------------------------
    You provide a Cartesian path to be followed by the robot's end-effector
    in the format of a list of SE(3) points.
    IT IS EXPECTED THE ROBOT IS AT THE BEGINNING OF THE PATH
    This function:
    1) uses an interpolated path (C^2 smooth line)
    2) uses a time law imposed on the interpolated path
    3) runs/returns a position-feedback, velocity-feedforward clik controller for the trajectory
    """
    ik_solver = getIKSolver(cfg.cs, robot)
    save_past_item = {"s": np.zeros(1)}
    log_item = {}
    log_item["qs"] = np.zeros(robot.nq)
    log_item["vs"] = np.zeros(robot.nv)
    log_item["s"] = np.zeros(
        1,
    )
    log_item["manipulability"] = np.zeros(
        1,
    )
    log_item["ee_pose_error"] = np.zeros(2 * 6)
    log_item["ee_velocity_error"] = np.zeros(2 * 6)
    if issubclass(robot.__class__, DualArmWholeBodyInterface):
        log_item["ebd_dist"] = np.zeros(3)

    controlLoop = partial(
        cartesianDualArmTrajectoryTrackingControlLoop,
        T_absgoal_l,
        T_absgoal_r,
        motion_planner,
        ik_solver,
        cfg.cs,
        robot,
    )
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_item, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager
