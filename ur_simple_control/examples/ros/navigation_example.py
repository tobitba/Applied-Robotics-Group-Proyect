from smc import load_config
#!/usr/bin/env python3


import rclpy
from rclpy.executors import MultiThreadedExecutor
from smc.multiprocessing.smc_yumi_node import get_cfg, SMCMobileYuMiNode

import numpy as np
import argparse
from smc.robots.implementations.mobile_yumi import RealMobileYumiRobotManager
from smc import load_config
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.cartesian_space import getClikArgs, getIKSolver
from smc.control.joint_space.joint_space_point_to_point import moveJP
from smc.control.cartesian_space.cartesian_space_point_to_point import (
    controlLoopClik,
    moveL,
    moveLDualArm,
    moveL,
)
from smc.control.optimal_control.croco_point_to_point.mpc.base_reference_mpc import (
    CrocoBaseP2PMPC,
)
from smc.control.optimal_control.croco_path_following.mpc.base_reference_mpc import (
    CrocoBasePathFollowingMPC,
)
from smc.util.draw_path import drawPath
from smc.control.cartesian_space.cartesian_space_trajectory_following import (
    cartesianPathFollowingWithPlanner,
)

import pinocchio as pin
import matplotlib
from functools import partial

import numpy as np


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


# def main(cfg=None):
def main(cfg=None):
    # evil and makes cfg unusable but what can you do
    cfg_smc = get_cfg()
    assert cfg_smc.robot == "myumi"
    robot = RealMobileYumiRobotManager(cfg_smc)
    # you can't do terminal input with ros
    # goal = robot.defineGoalPointCLI()
    T_w_absgoal = pin.SE3.Identity()
    T_w_absgoal.translation = np.array([0.5, 0.0, 0.5])
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation = np.array([0.0, 0.2, 0.0])
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation = np.array([0.0, -0.2, 0.0])
    # loop_manager = CrocoIKMPC(cfg, robot, goal, run=False)
    # robot._mode = robot.control_mode.upper_body
    # loop_manager = moveJP(robot._comfy_configuration, cfg_smc, robot, run=False)
    # loop_manager = moveLDualArm(
    #    cfg_smc, robot, T_w_absgoal, T_absgoal_l, T_absgoal_r, run=False
    # )
    # loop_manager = moveL(cfg_smc, robot, T_w_absgoal, run=False)
    robot._mode = robot.control_mode.whole_body
    robot.setInitialPose()
    robot._step()
    if not cfg_smc.draw_new:
        pixel_path_file_path = "./parameters/path_in_pixels.csv"
        path2D = np.genfromtxt(pixel_path_file_path, delimiter=",")
    else:
        matplotlib.use("tkagg")
        path2D = drawPath(cfg_smc)
        matplotlib.use("qtagg")
    path2D[:, 0] = path2D[:, 0] * cfg_smc.map_width
    path2D[:, 1] = path2D[:, 1] * cfg_smc.map_height
    path2D = np.hstack((path2D, np.zeros((len(path2D), 1))))
    robot.updateViz({"fixed_path": path2D})
    path_planner = partial(fixedPathPlanner, [0], path2D)

    # x0 = np.concatenate([robot.q, robot.v])
    # loop_manager = CrocoBasePathFollowingMPC(
    #    cfg_smc, robot, x0, path_planner, run=False
    # )
    robot._mode = robot.control_mode.base_only
    loop_manager = cartesianPathFollowingWithPlanner(
        cfg_smc, robot, path_planner, 0.0, run=False
    )

    rclpy.init(cfg=cfg)

    executor = MultiThreadedExecutor()
    node = SMCMobileYuMiNode(cfg_smc, robot, loop_manager)
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
