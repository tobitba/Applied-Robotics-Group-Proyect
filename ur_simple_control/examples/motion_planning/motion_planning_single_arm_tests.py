from smc import load_config
from smc.control.cartesian_space import (
    getClikArgs,
)
from smc.control.cartesian_space.ik_solvers import dampedPseudoinverse, getIKSolver
from smc.motion_planning.motion_planner import MotionPlannerUpdatingPath
from smc.control.control_loop_manager import ControlLoopManager
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.robots.utils import getRobotFromConfig, load_config

from smc.motion_planning.path_planning.test_planners.test_path_generator import (
    getTestPathPlanningArgs,
    TestSE3PathGenerator,
)
from smc.motion_planning.trajectory_generation.end_effector_velocity_limit_scaling import (
    computeTimeScalingEEVelConstraint,
    uniformTimeLaw,
)

import numpy as np
from functools import partial
import pinocchio as pin
import argparse
from typing import Callable
from collections import deque


def get_cfg() -> argparse.Namespace:
    parser = load_config()
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getTestPathPlanningArgs(parser)
    parser.add_argument(
        "--map-width",
        type=float,
        help="width of the map in meters (x-axis) - only used for drawing of the path",
        default=3.0,
    )
    parser.add_argument(
        "--map-height",
        type=float,
        help="height of the map in meters (y-axis) - only used for drawing of the path",
        default=3.0,
    )
    parser.add_argument(
        "--K-fb",
        type=float,
        help="weight for feedback in path following",
        default=1.0,
    )
    # TODO: remove
    parser.add_argument(
        "--K-ff",
        type=float,
        help="weight for feedforward in path following",
        default=0.05,
    )
    cfg = parser.parse_cfg()
    return cfg


# NOTE: as far as i'm concerned this does not work.
# as in we do need a sensible timing law
# TODO: this won't work for diff drive or other underactuated!!
# it also doesn't work in general lmao
def parameterTrackingHackTimeLaw(
    robot: AbstractRobotManager, V_cmd: np.ndarray, v_cmd: np.ndarray
) -> float:
    v_cmd_to_real = np.clip(v_cmd, -1 * robot._max_v, robot._max_v)

    # how much faster is the real v_cmd compared to the one needed
    # to follow this path?
    # --> we update s with this factor, call it k.
    # we use an ik solver which DOES NOT respect velocity contraints
    # to get this estimate
    v_cmd_uncostrained = dampedPseudoinverse(cfg, robot, V_cmd)
    k = min(1.0, np.min(np.abs(v_cmd_to_real / v_cmd_uncostrained)))
    s_update = robot.dt * k
    return s_update


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)

    path_planner = TestSE3PathGenerator(cfg)

    T_list: list[pin.SE3] = path2D_to_SE3(path3D, cfg.path_height, cfg.path_rotx)
    T_llist = []
    # NOTE: bullshit filtering, should be done based on distances between points
    # TODO: do it based on distances between points
    for i, T in enumerate(T_list):
        if i % 5 == 0:
            T_llist.append(T)
    T_list = T_llist
    C_segments, s_list = fit_SE3_C2_bezier_spline(T_list)
    # we need a function that spits out T_s, V_s,
    # the controller does not care for its internal variables (as it shouldn't)
    getCurrentPathReference = partial(getTAndVFromBSpline, C_segments, s_list)

    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": T_list[0]})
        viz_Ts = []
        ss = np.linspace(0, 1, 500)
        for s in ss:
            viz_Ts.append(getTAndVFromBSpline(C_segments, s_list, s)[0])
        robot.visualizer_manager.sendCommand({"framepath": viz_Ts})

    # go to start
    print("going to path start")
    moveL(cfg, robot, T_list[0])

    # construct timing law
    T_total = computeTimeScalingEEVelConstraint(
        C_segments,
        s_list,
        v_max=np.ones(6) * 0.5,
        a_min=np.ones(6) * -1.0,
        a_max=np.ones(6) * 1.0,
    )
    T_total = T_total * 4
    time_law = partial(uniformTimeLaw, T_total)

    print("starting to follow path")
    cartesianTrajectoryTracking(getCurrentPathReference, time_law, cfg, robot)

    robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
