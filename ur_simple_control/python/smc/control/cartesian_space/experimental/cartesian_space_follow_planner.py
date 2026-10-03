from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
from smc.control.control_loop_manager import ControlLoopManager
from smc.multiprocessing import CollaborativeProcess
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.control.controller_templates.path_following_template import (
    PathFollowingFromPlannerCtrllLoopTemplate,
)
from smc.control.cartesian_space.ik_solvers import getIKSolver, dampedPseudoinverse
from smc.path_generation.path_math.path2d_to_6d import (
    path2D_to_SE3,
)
from smc.path_generation.path_math.path_to_trajectory import path2D_to_trajectory2D
from smc.trajectory_generation.SE3_quintic_trajectory import (
    extract_positions_and_rots,
    compute_times_from_positions,
    compute_desired_twist_from_quintic,
)

from functools import partial
from pinocchio import SE3, log6
import numpy as np
from collections import deque
from typing import Callable
import types


def cartesianDualArmPlannerFollowingControlLoop_quintic_slerp(
    rot_x: float,
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    ik_solver: Callable[[np.ndarray, np.ndarray], np.ndarray],
    path: list[SE3] | np.ndarray,
    cfg: ConfigCartesianSpace,
    robot,
    t: int,
    _: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:

    # NOTE: assuming the first path point coincides with current pose

    horizon_steps = 10
    min_allowed_horizon_steps = 3
    horizon_time_window = 0.4  # increase/decrease this?
    lookahead_time = 0.3

    len_path = len(path)

    if len_path < min_allowed_horizon_steps:
        v_cmd = np.zeros((17,))
        err_vector = np.zeros((12,))
        print(
            f"No. of points left in path ({len(path)}) is less than horizon length ({horizon_steps}), sending 0 velocities"
        )
    else:
        assert (
            lookahead_time <= horizon_time_window
        ), f"Look ahead time {lookahead_time} must be less than horizon time window {horizon_time_window}"

        if len_path < horizon_steps:
            print(f"Shrinking horizon from {horizon_steps} to {len_path}")
            horizon_steps = len_path

        if isinstance(path, np.ndarray):
            path = path2D_to_SE3(path[:, :2], cfg.path_height, rot_x)

        poses_left = [path[i].act(T_absgoal_l) for i in range(horizon_steps)]
        poses_right = [path[i].act(T_absgoal_r) for i in range(horizon_steps)]

        positions_left, rotations_left = extract_positions_and_rots(poses_left)
        positions_right, rotations_right = extract_positions_and_rots(poses_right)

        times_left = compute_times_from_positions(positions_left, horizon_time_window)
        times_right = compute_times_from_positions(positions_right, horizon_time_window)

        # if last two timestamps are the same, drop last timestamp
        if abs(times_left[-1] - times_left[-2]) <= 1e-6:
            times_left = times_left[:-1]
            positions_left = positions_left[:-1]
            rotations_left = rotations_left[:-1]
        if abs(times_right[-1] - times_right[-2]) <= 1e-6:
            times_right = times_right[:-1]
            positions_right = positions_right[:-1]
            rotations_right = rotations_right[:-1]

        desired_SE_left = compute_desired_twist_from_quintic(
            positions_left, rotations_left, times_left, lookahead_time
        )
        desired_SE_right = compute_desired_twist_from_quintic(
            positions_right, rotations_right, times_right, lookahead_time
        )

        SEerror_left = robot.T_w_l.actInv(desired_SE_left)
        SEerror_right = robot.T_w_r.actInv(desired_SE_right)
        err_vector_left = log6(SEerror_left).vector
        # the multiplication by 2 is per magic number in cartesianDualArmPathFollowingControlLoop()
        err_vector_left[3:] = err_vector_left[3:] * 2
        err_vector_right = log6(SEerror_right).vector
        err_vector_right[3:] = err_vector_right[3:] * 2
        err_vector = np.concatenate((err_vector_left, err_vector_right))

        # feedforward: https://gitlab.control.lth.se/marko-g/ur_simple_control/-/blob/slumi_interface/python/smc/control/cartesian_space/cartesian_space_trajectory_following.py?ref_type=heads#L51
        # ff_gain : float = 5.0
        # ff = np.concatenate((twist_left, twist_right))
        # err_vector = err_vector + ff_gain * ff

        J = robot.getJacobian()
        v_cmd = ik_solver(cfg, robot, err_vector)

        if cfg.visualizer:
            if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
                robot.visualizer_manager.sendCommand({"frame_path": path[:20]})

    return (
        v_cmd,
        {},
        {"err_vec_ee": err_vector},
    )


def cartesianDualArmPathFollowingWithPlanner(
    cfg: ConfigCartesianSpace,
    robot: DualArmInterface,
    path_planner: CollaborativeProcess | types.FunctionType,
    x_rot: float,
    T_absgoal_l: SE3,
    T_absgoal_r: SE3,
    run=True,
) -> None | ControlLoopManager:
    ik_solver = getIKSolver(cfg, robot)
    print(f"ik_solver {ik_solver.func}")
    get_position = lambda robot: robot.T_w_e.translation[:2]
    pathFollowingLoop = partial(
        cartesianDualArmPlannerFollowingControlLoop_quintic_slerp,
        x_rot,
        T_absgoal_l,
        T_absgoal_r,
    )
    controlLoop = partial(
        PathFollowingFromPlannerCtrllLoopTemplate,
        path_planner,
        get_position,
        ik_solver,
        pathFollowingLoop,
        cfg,
        robot,
    )
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "err_vec_ee": np.zeros(2 * 6),
    }
    save_past_item = {}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_item, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


# TODO: old horror without proper interpolation or traj tracking
# TODO: refactor this horror out of here
# this is a broken example, and should be treated as such
# (works if you have a gazillion points,
# but even more just shows what not to do,
# which might be pedagogical in an example)
# can't remove it before refactoring what it depends on though
# def cartesianPathFollowingControlLoop(
#    rot_x: float,
#    ik_solver: Callable[[np.ndarray, np.ndarray], np.ndarray],
#    path: list[SE3] | np.ndarray,
#    cfg: ConfigCartesianSpace,
#    robot: SingleArmInterface,
#    t: int,
#    _: dict[str, deque[np.ndarray]],
# ) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
#    """
#    cartesianPathFollowingControlLoop
#    -----------------------------
#    end-effector(s) follow their path(s) according to what a 2D path-planner spits out
#    """
#
#    # TODO: refactor this horror out of here
#    if type(path) == np.ndarray:
#        # TODO: this would be cool but i can't unfortunatelly
#        # velocity = cfg.max_v_percentage
#        # traj = path2D_to_trajectory2D(cfg, path, velocity)
#        # path = path2D_to_SE3(traj[:, :2], 0.0, rot_x)
#        path = path2D_to_SE3(path[:, :2], cfg.path_height, rot_x)
#    # TODO: arbitrary bs, read a book and redo this
#    # NOTE: assuming the first path point coincides with current pose
#    SEerror = robot.T_w_e.actInv(path[1])
#    err_vector = log6(SEerror).vector
#    # if np.linalg.norm(err_vector) < 0.2:
#    #     V_path = 5 * log6(path[1].actInv(path[2])).vector
#    #     err_vector += V_path
#    # err_vector[3:] = err_vector[3:] * 2
#    J = robot.getJacobian()
#    v_cmd = ik_solver(cfg, robot, err_vector)
#
#    if v_cmd is None:
#        print(
#            t,
#            "the controller you chose produced None as output, using dampedPseudoinverse instead",
#        )
#        v_cmd = dampedPseudoinverse(cfg, robot, err_vector)
#    #    else:
#    #        if cfg.debug_prints:
#    #            print(t, "ik solver success")
#
#    # maybe visualize the closest path point instead? the path should be handled
#    # by the path planner
#    if cfg.visualizer:
#        if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
#            robot.visualizer_manager.sendCommand({"frame_path": path[:20]})
#
#    return (
#        v_cmd,
#        {},
#        {"err_vec_ee": err_vector},
#    )
#
#
# def cartesianPathFollowingWithPlanner(
#    cfg: ConfigCartesianSpace,
#    robot: SingleArmInterface,
#    path_planner: CollaborativeProcess | types.FunctionType,
#    x_rot: float,
#    run=True,
# ) -> None | ControlLoopManager:
#    ik_solver = getIKSolver(cfg, robot)
#    get_position = lambda robot: robot.T_w_e.translation[:2]
#    pathFollowingLoop = partial(cartesianPathFollowingControlLoop, x_rot)
#    controlLoop = partial(
#        PathFollowingFromPlannerCtrllLoopTemplate,
#        path_planner,
#        get_position,
#        ik_solver,
#        pathFollowingLoop,
#        cfg,
#        robot,
#    )
#    log_item = {
#        "qs": np.zeros(robot.nq),
#        "vs": np.zeros(robot.nv),
#        "err_vec_ee": np.zeros(6),
#    }
#    save_past_item = {}
#    loop_manager = ControlLoopManager(
#        robot, controlLoop, cfg, save_past_item, log_item
#    )
#    if run:
#        loop_manager.run()
#    else:
#        return loop_manager
#
## TODO: old horror without proper interpolation or traj tracking
# def cartesianDualArmPathFollowingControlLoop(
#    rot_x: float,
#    T_absgoal_l: SE3,
#    T_absgoal_r: SE3,
#    ik_solver: Callable[[np.ndarray, np.ndarray], np.ndarray],
#    path: list[SE3] | np.ndarray,
#    cfg: ConfigCartesianSpace,
#    robot: DualArmInterface,
#    t: int,
#    _: dict[str, deque[np.ndarray]],
# ) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
#    """
#    cartesianPathFollowingControlLoop
#    -----------------------------
#    end-effector(s) follow their path(s) according to what a 2D path-planner spits out
#    """
#
#    if type(path) == np.ndarray:
#        path = path2D_to_SE3(path[:, :2], cfg.path_height, rot_x)
#
#    # NOTE: assuming the first path point coincides with current pose
#    # path is a list of 13 SE3
#
#    T_w_lgoal = path[1].act(T_absgoal_l)
#    T_w_rgoal = path[1].act(T_absgoal_r)
#
#    SEerror_left = robot.T_w_l.actInv(T_w_lgoal)
#    SEerror_right = robot.T_w_r.actInv(T_w_rgoal)
#
#    err_vector_left = log6(SEerror_left).vector
#    err_vector_left[3:] = err_vector_left[3:] * 2
#    err_vector_right = log6(SEerror_right).vector
#    err_vector_right[3:] = err_vector_right[3:] * 2
#    err_vector = np.concatenate((err_vector_left, err_vector_right))
#
#    if np.linalg.norm(err_vector) < 0.2:
#        V_path = 5 * log6(path[1].actInv(path[2])).vector
#        err_vector += np.concatenate((V_path, V_path))
#    J = robot.getJacobian()
#    v_cmd: np.ndarray = ik_solver(cfg, robot, err_vector)  # shape for slumi: (17,)
#
#    # maybe visualize the closest path point instead? the path should be handled
#    # by the path planner
#    if cfg.visualizer:
#        if t % int(np.ceil(cfg.ctrl_freq / 25)) == 0:
#            robot.visualizer_manager.sendCommand({"frame_path": path[:20]})
#
#    # print(f"v_cmd ({v_cmd.shape}): {v_cmd}")
#    return (
#        v_cmd,
#        {},
#        {"err_vec_ee": err_vector},
#    )
#
#
