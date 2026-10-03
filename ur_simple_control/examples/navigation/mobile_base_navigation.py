from smc import getRobotFromConfig, load_config
from smc.control.control_loop_manager import ControlLoopManager
from smc.robots.interfaces import MobileBaseInterface
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.control.optimal_control.util import get_OCP_cfg
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_trajectory_following import (
    cartesianPathFollowingWithPlanner,
)
from smc.path_generation.planner import starPlanner, getPlanningArgs
from smc.control.optimal_control.croco_point_to_point.mpc.base_reference_mpc import (
    CrocoBaseP2PMPC,
)
from smc.control.optimal_control.croco_path_following.mpc.base_reference_mpc import (
    CrocoBasePathFollowingMPC,
)
from smc.multiprocessing import ProcessManager
from smc.util.draw_path import drawPath

import numpy as np
from functools import partial
import argparse
from collections import deque
import matplotlib


def get_cfg() -> argparse.Namespace:
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getPlanningArgs(parser)
    parser.add_argument(
        "--controller",
        type=str,
        help="choose between path following mpc or cartesian path following",
        choices=["cartesian", "mpc"],
        default="cartesian",
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
        "--path-height",
        type=float,
        help="this is pointless for navigation, don't use it",
        default=0.0,
    )
    parser.add_argument(
        "--map-height",
        type=float,
        help="height of the map in meters (y-axis) - only used for drawing of the path",
        default=3.0,
    )
    cfg = parser.parse_cfg()
    return cfg


def fixedPathPlanner(
    path_parameter: list[int], path2D: np.ndarray, p_base: np.ndarray
) -> np.ndarray:
    """
    fixedPathPlanner
    ----------------
    we can assume robot is following this path and that it started the following controller at the first point.
    to make the controller make sense (without a sense of timing), we need to find the closest point on the path,
    and delete the already traversed portion of the path.
    the assumption is that the controller constructs a trajectory from the path, starting with the first point.

    the cleanest way to get this done is associate robot's current position with the path parameter s,
    but i'll just implement it as quickly as possible, not in the most correct or the most efficient manner.

    NOTE: path is (N,3) in shape, last column is 0, because subotimal code structure
    """

    p_base_3 = np.array([p_base[0], p_base[1], 0.0])
    distances = np.linalg.norm(p_base_3 - path2D, axis=1)
    index = np.argmin(distances)
    if (index - path_parameter[0]) > 0 and (index - path_parameter[0]) < 10:
        path_parameter[0] += 1
    path2D = path2D[path_parameter[0] :]
    return path2D


def MPCWithCLIKFallbackControlLoop(
    loop_manager_mpc: ControlLoopManager,
    loop_manager_clik: ControlLoopManager,
    cfg: argparse.Namespace,
    robot: MobileBaseInterface,
    t: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    breakFlag = False
    if np.linalg.norm(robot.v) < 1e-2:
        print("clicking")
        breakFlag = loop_manager_clik.run_one_iter(t)
    else:
        print("mpcing")
        breakFlag = loop_manager_mpc.run_one_iter(t)

    return breakFlag, {}, {"qs": robot.q, "vs": robot.v}


def MPCWithCLIKFallback(
    cfg: argparse.Namespace, robot: MobileBaseInterface, path_planner
) -> None:
    loop_manager_mpc = CrocoBasePathFollowingMPC(
        cfg, robot, x0, path_planner, run=False
    )
    loop_manager_clik = cartesianPathFollowingWithPlanner(
        cfg, robot, path_planner, run=False
    )
    control_loop = partial(
        MPCWithCLIKFallbackControlLoop, loop_manager_mpc, loop_manager_clik, cfg, robot
    )

    log_item = {}
    log_item["qs"] = np.zeros(robot.nq)
    log_item["vs"] = np.zeros(robot.nv)
    loop_manager = ControlLoopManager(robot, control_loop, cfg, {}, log_item)
    loop_manager.run()


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    assert issubclass(robot.__class__, MobileBaseInterface)
    robot._step()
    if cfg.planner:
        robot._q[0] = 0.5
        robot._q[1] = 0.0
        robot._step()
    x0 = np.concatenate([robot.q, robot.v])
    goal = np.array([0.5, 5.5])
    T_w_b = robot.T_w_b

    if cfg.planner:
        planning_function = partial(starPlanner, goal)
        # here we're following T_w_e reference so that's what we send
        path_planner = ProcessManager(
            cfg, planning_function, T_w_b.translation[:2], 3, None
        )
        _, map_as_list = createSampleStaticMap()
        if cfg.visualizer:
            robot.sendRectangular2DMapToVisualizer(map_as_list)
            # time.sleep(5)
    else:
        if not cfg.draw_new:
            pixel_path_file_path = "./parameters/path_in_pixels.csv"
            path2D = np.genfromtxt(pixel_path_file_path, delimiter=",")
        else:
            matplotlib.use("tkagg")
            path2D = drawPath(cfg)
            matplotlib.use("qtagg")
        path2D[:, 0] = path2D[:, 0] * cfg.map_width
        path2D[:, 1] = path2D[:, 1] * cfg.map_height
        path2D = np.hstack((path2D, np.zeros((len(path2D), 1))))
        robot.updateViz({"fixed_path": path2D})
        # CrocoBaseP2PMPC(cfg, robot, path2D[0])
        path_planner = partial(fixedPathPlanner, [0], path2D)

    if cfg.controller == "mpc":
        print("running mpc")
        CrocoBasePathFollowingMPC(cfg, robot, x0, path_planner)
    if cfg.controller == "cartesian":
        print("running cartesian")
        loop = cartesianPathFollowingWithPlanner(
            cfg, robot, path_planner, 0.0, run=False
        )
        breakFlag = False
        import time

        while not breakFlag:
            loop.run_one_iter(loop.current_iteration)
            time.sleep(robot._dt)
        # MPCWithCLIKFallback(cfg, robot, path_planner)

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
