from smc import load_config
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.robots.interfaces import DualArmInterface
from smc.control.cartesian_space import (
    getClikArgs,
    moveLDualArm,
    cartesianDualArmPathFollowingWithPlanner,
)
from smc.robots.utils import getRobotFromConfig, load_config
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.path_generation.planner import starPlanner, getPlanningArgs
from smc.multiprocessing import ProcessManager

from smc.path_generation.fixed_path_planner import contructPath, fixedPathPlanner

import time
import numpy as np
from functools import partial
import pinocchio as pin
import argparse


def get_cfg() -> argparse.Namespace:
    parser = load_config()
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getPlanningArgs(parser)
    parser.add_argument(
        "--path-height",
        type=float,
        default=0.5,
        help="heigh of the path",
    )
    parser.add_argument(
        "--path-rotx",
        type=float,
        default=0.0,
        help="x-axis rotation along the path",
    )
    parser.add_argument(
        "--base-to-handlebar-preferred-distance",
        type=float,
        default=0.5,
        help="prefered path arclength from mobile base position to handlebar",
    )
    parser.add_argument(
        "--planner",
        action=argparse.BooleanOptionalAction,
        help="if on, you're in a pre-set map and a planner produce a plan to navigate. if off, you draw the path to be followed",
        default=True,
    )
    parser.add_argument(
        "--draw-new",
        action=argparse.BooleanOptionalAction,
        help="are you drawing a new path or reusing the previous one",
        default=False,
    )
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
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    assert issubclass(robot.__class__, DualArmInterface)

    print(f"cfg.planner = {cfg.planner}")
    print(f"cfg.goal_error = {cfg.goal_error}")
    # cfg.goal_error = 0.45

    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15

    if cfg.planner:
        robot._step()
        robot._q[0] = 1.0
        robot._q[1] = 0.6
        robot._step()
        x0 = np.concatenate([robot.q, robot.v])
        goal = np.array([5.5, 5.0])
        planning_function = partial(starPlanner, goal)
        # here we're following T_w_e reference so that's what we send
        path_planner = ProcessManager(
            cfg, planning_function, robot.T_w_e.translation[:2], 3, None
        )
        _, map_as_list = createSampleStaticMap()
        # TODO: put in real lab map
        if cfg.visualizer:
            robot.sendRectangular2DMapToVisualizer(map_as_list)

        data = None
        while data is None:
            path_planner.sendCommand(robot.T_w_e.translation[:2])
            data = path_planner.getData()
            time.sleep(1)

        _, path2D = data
        path2D = np.array(path2D).reshape((-1, 2))
        path2D = np.hstack((path2D, np.zeros((len(path2D), 1))))

    else:
        path2D = contructPath(cfg, robot)
        path_planner = partial(fixedPathPlanner, [0], path2D)

    pathSE3 = path2D_to_SE3(path2D, cfg.path_height, cfg.path_rotx)
    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": pathSE3[0]})

    # go to start
    print(f"path length = {len(pathSE3)}")
    print(f"Moving to path start at {pathSE3[0]} with robot type {type(robot)}")
    moveLDualArm(cfg, robot, pathSE3[0], T_absgoal_l, T_absgoal_r)
    print("Reached path start.")

    # follow the path
    print(f"IK solver = {cfg.ik_solver}")
    print(f"path.rotx = {cfg.path_rotx}")
    print(f"T_absgoal_l:\n{T_absgoal_l}")
    print(f"T_absgoal_r:\n{T_absgoal_r}")
    cartesianDualArmPathFollowingWithPlanner(
        cfg, robot, path_planner, cfg.path_rotx, T_absgoal_l, T_absgoal_r
    )

    if cfg.real:
        print("Stopping the robot...")
        robot.stopRobot()
        print("Robot stopped.")
    else:
        print("Not real, nothing to stop")

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
