from smc import load_config
from smc.robots.utils import getRobotFromConfig
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.control.joint_space.joint_space_point_to_point import moveJPWTraj
from smc.control.cartesian_space.pink_p2p import (
    DualArmIKSelfAvoidanceViaEndEffectorSpheres,
)
from smc.control.cartesian_space import (
    moveL,
    moveLDualArm,
)
from smc.control.cartesian_space.cartesian_space_follow_planner import (
    cartesianDualArmPathFollowingWithPlanner,
)
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.path_generation.fixed_path_planner import trucatePathToClosestPathPoint

from utils import get_cfg, constructInitialT_w_abs

import time
import numpy as np
from functools import partial
import pinocchio as pin

if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    assert issubclass(robot.__class__, DualArmWholeBodyInterface)
    # the bases are different so the concrete cartesian points are different
    assert cfg.robot == "slumi"
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15

    # N_pts = 5000
    N_pts = 100
    linear = np.linspace(0.0, 8.5, N_pts).reshape((N_pts, 1))
    path_ee_2D = np.hstack((linear, 0.0 * np.sin(linear * 2)))
    path_ee_3D = np.hstack((path_ee_2D, np.zeros((len(path_ee_2D), 1))))
    path_planner = partial(trucatePathToClosestPathPoint, [0], path_ee_3D)

    pathSE3 = path2D_to_SE3(path_ee_3D, cfg.path_height, np.pi)
    T_w_absgoal = constructInitialT_w_abs(cfg, path_ee_3D, pathSE3[0].rotation)

    if cfg.visualizer:
        viz_path = [frame for i, frame in enumerate(pathSE3) if i % 5 == 0]
        robot.visualizer_manager.sendCommand({"frame_path": viz_path})
        robot.visualizer_manager.sendCommand({"Mgoal": pathSE3[0]})

    # NOTE: T_w_absgoal's x-axis points toward the path.
    # z-axis points down by an undocumented choice
    # rotation_init_rpy = pin.rpy.matrixToRpy(T_w_absgoal.rotation)
    # rotation_init = pin.rpy.rpyToMatrix(0.0, 0.0, rotation_init_rpy[2] - np.pi / 2)
    robot.mode = robot.control_mode.base_only
    p_base_goal = pathSE3[0].copy().translation
    p_base_goal[2] = 0.0
    T_w_bgoal = pin.SE3(np.eye(3), +np.array([0.7, 0.0, 0.0]))
    moveL(cfg, robot, T_w_bgoal)

    print("putting arms to a comfy configuration")
    robot.mode = robot.control_mode.upper_body
    moveJPWTraj(robot.comfy_configuration, cfg, robot)

    ################################################
    # 4. move arms to beginning of path
    ###############################################
    # robot.mode = robot.control_mode.whole_body
    # cfg.ik_solver = "QPWithPosture"

    print("moveL to start of path")
    robot.mode = robot.control_mode.upper_body
    cfg.ik_solver = "QPWithPosture"
    moveLDualArm(cfg, robot, pathSE3[0], T_absgoal_l, T_absgoal_r)

    robot._mode = robot.control_mode.whole_body
    # cfg.ik_solver = "QPWithPostureAndEBDistance"
    cfg.ik_solver = "QPWithPosture"
    cartesianDualArmPathFollowingWithPlanner(
        cfg, robot, path_planner, np.pi, T_absgoal_l, T_absgoal_r
    )

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
