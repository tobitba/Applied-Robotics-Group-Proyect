from smc import load_config
#!/usr/bin/env python3


from smc.multiprocessing.smc_yumi_node import get_cfg, SMCMobileYuMiNode
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.path_generation.planner import starPlanner
from smc.multiprocessing import ProcessManager
from smc.robots.abstract_robotmanager import AbstractRobotManager

import numpy as np
from smc.robots.implementations.mobile_yumi import RealMobileYumiRobotManager
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
from smc.control.cartesian_space.cartesian_space_trajectory_following import (
    cartesianPathFollowingWithPlanner,
)
from smc.control.cartesian_space.pink_p2p import (
    DualArmIKSelfAvoidanceViaEndEffectorSpheres,
)

from fixed_path_planner import contructPath, fixedPathPlanner
from utils import constructInitialT_w_abs
from dual_arm_cart_pulling_control_loop import DualArmCartPulling

import pinocchio as pin
import matplotlib
from functools import partial
import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
from copy import deepcopy


# def main(cfg=None):
def main(cfg=None):
    # evil and makes cfg unusable but what can you do
    cfg_smc = get_cfg()
    assert cfg_smc.robot == "myumi"
    robot = RealMobileYumiRobotManager(cfg_smc)
    robot._step()
    modes_and_loops = []
    # you can't do terminal input with ros
    # goal = robot.defineGoalPointCLI()
    # T_w_absgoal = pin.SE3.Identity()
    # T_w_absgoal.translation = np.array([0.5, 0.0, 0.5])
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation = np.array([0.0, -0.2, 0.0])
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation = np.array([0.0, 0.2, 0.0])

    goal = np.array([2.5, 0.0])

    ######################################################
    # 1. initialize path planner, or instantiate pre-made path
    #    and make that a path planner, for the base
    #################################################
    if cfg_smc.planner:
        planning_function = partial(starPlanner, goal)
        # here we're following T_w_e reference so that's what we send
        path_planner = ProcessManager(
            cfg_smc, planning_function, robot.T_w_b.translation[:2], 3, None
        )
        _, map_as_list = createSampleStaticMap()
        # TODO: put in real lab map
        if cfg_smc.visualizer:
            robot.sendRectangular2DMapToVisualizer(map_as_list)

        data = None
        while data is None:
            path_planner.sendCommand(robot.T_w_b.translation[:2])
            data = path_planner.getData()
            time.sleep(1)

        _, path_base = data
        path_base = np.array(path_base).reshape((-1, 2))
        path_base = np.hstack((path_base, np.zeros((len(path_base), 1))))
    else:
        path_base = contructPath(cfg_smc, robot)
        path_planner = partial(fixedPathPlanner, [0], path_base)

    ################################################
    # 2. construct initial T_w_abs (starting middle of handlebar pose)
    ################################################
    pathSE3 = path2D_to_SE3(path_base, cfg_smc.handlebar_height, np.pi)
    init_rot = pathSE3[0].rotation @ pin.rpy.rpyToMatrix(np.pi, 0.0, 0.0)
    init_rot = init_rot @ pin.rpy.rpyToMatrix(0.0, 0.0, np.pi)
    # T_w_absgoal = constructInitialT_w_abs(cfg, path_base, init_rot)
    T_w_absgoal = constructInitialT_w_abs(cfg_smc, path_base, pathSE3[0].rotation)

    if cfg_smc.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": pathSE3[0]})

    ##################################################
    # 3. get arms into comfy position
    ####################################################
    # print("putting arms to a comfy configuration")
    mode_1 = AbstractRobotManager.control_mode.upper_body
    loop_manager_jp = moveJP(robot._comfy_configuration, cfg_smc, robot, run=False)
    modes_and_loops.append((mode_1, loop_manager_jp))

    ###################################################
    # 4. get base to starting pose
    ###################################################
    # print("getting base to start of path")
    p_basegoal = path_base[0]
    # NOTE: requires passing a reduced model to work because crocoddyl needs the model to formulate itself
    # robot.mode = AbstractRobotManager.control_mode.base_only

    ##########################################
    # orient base to look at the goal (yes this should've be handled with basep2p)
    ##########################################
    # NOTE: gets stuck on real for some unknown reason
    mode_2 = robot.control_mode.base_only
    T_w_bgoal = pin.SE3(init_rot, p_basegoal)
    cfg_smc_1 = deepcopy(cfg_smc)
    cfg_smc_1.goal_error = 10
    loop_manager_moveL_1 = moveL(cfg_smc_1, robot, T_w_bgoal.copy(), run=False)
    modes_and_loops.append((mode_2, loop_manager_moveL_1))

    # NOTE: alternative hack didn't work
    # q_desired = np.zeros(robot.model.nq)
    # q_desired[:2] = p_basegoal[:2]
    # rp = pin.rpy.matrixToRpy(init_rot)
    # q_desired[2] = np.cos(rp[2] + np.pi)
    # q_desired[3] = np.sin(rp[2] + np.pi)
    # q_desired[4:] = robot._comfy_configuration[4:]
    # loop_manager_jp2 = moveJP(q_desired, cfg_smc, robot, run=False)
    # modes_and_loops.append((mode_1, loop_manager_jp2))

    ##########################################
    # go down and grasp
    #######################################
    mode_3 = AbstractRobotManager.control_mode.upper_body
    T_w_absgoal_1 = T_w_absgoal.copy()
    T_w_absgoal_2 = T_w_absgoal.copy()
    T_w_absgoal_1.translation[2] += 0.1
    # print("going above handlebar")
    loop_manager_ik_1 = DualArmIKSelfAvoidanceViaEndEffectorSpheres(
        T_w_absgoal, T_absgoal_l, T_absgoal_r, cfg_smc, robot, run=False
    )
    modes_and_loops.append((mode_3, loop_manager_ik_1))

    # DualArmMoveL down to handlebar
    # print("going down to handlebar")
    # T_w_absgoal.translation[2] -= 0.1
    loop_manager_ik_2 = DualArmIKSelfAvoidanceViaEndEffectorSpheres(
        T_w_absgoal_2, T_absgoal_l, T_absgoal_r, cfg_smc, robot, run=False
    )
    modes_and_loops.append((mode_3, loop_manager_ik_2))

    mode_4 = robot.control_mode.whole_body
    loop_manager_pulling = DualArmCartPulling(
        cfg_smc, robot, path_planner, T_absgoal_l, T_absgoal_r, run=False
    )
    modes_and_loops.append((mode_4, loop_manager_pulling))

    rclpy.init(cfg=cfg)

    executor = MultiThreadedExecutor()
    node = SMCMobileYuMiNode(cfg_smc, robot, modes_and_loops)
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
