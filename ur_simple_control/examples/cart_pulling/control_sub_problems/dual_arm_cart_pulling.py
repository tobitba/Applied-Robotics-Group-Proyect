from smc import load_config
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.robots.utils import getRobotFromConfig
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.path_generation.planner import starPlanner
from smc.multiprocessing import ProcessManager
from smc.control.optimal_control.croco_point_to_point.mpc.base_reference_mpc import (
    CrocoBaseP2PMPC,
)
from smc.control.joint_space.joint_space_point_to_point import moveJP
from smc.control.cartesian_space.pink_p2p import (
    DualArmIKSelfAvoidanceViaEndEffectorSpheres,
)
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)

from utils import get_cfg, constructInitialT_w_abs
from dual_arm_cart_pulling_control_loop import DualArmCartPulling, DualArmCartPullingMPC
from fixed_path_planner import contructPath, fixedPathPlanner

import time
import numpy as np
from functools import partial
import pinocchio as pin

if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    assert issubclass(robot.__class__, DualArmWholeBodyInterface)
    # the bases are different so the concrete cartesian points are different
    assert cfg.robot == "myumi"
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15

    # TODO: before running on the real robot change this to just reading the current position!!
    if cfg.planner:
        robot._step()
        robot._q[0] = 1.0
        robot._q[1] = 0.6
        robot._step()
        x0 = np.concatenate([robot.q, robot.v])
        goal = np.array([5.5, 5.0])

    ######################################################
    # 1. initialize path planner, or instantiate pre-made path
    #    and make that a path planner, for the base
    #################################################
    if cfg.planner:
        planning_function = partial(starPlanner, goal)
        # here we're following T_w_e reference so that's what we send
        path_planner = ProcessManager(
            cfg, planning_function, robot.T_w_b.translation[:2], 3, None
        )
        _, map_as_list = createSampleStaticMap()
        # TODO: put in real lab map
        if cfg.visualizer:
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
        path_base = contructPath(cfg, robot)
        path_planner = partial(fixedPathPlanner, [0], path_base)

    ################################################
    # 2. construct initial T_w_abs (starting middle of handlebar pose)
    ################################################
    pathSE3 = path2D_to_SE3(path_base, cfg.handlebar_height, np.pi)
    init_rot = pathSE3[0].rotation @ pin.rpy.rpyToMatrix(np.pi, 0.0, 0.0)
    init_rot = init_rot @ pin.rpy.rpyToMatrix(0.0, 0.0, np.pi)
    # T_w_absgoal = constructInitialT_w_abs(cfg, path_base, init_rot)
    T_w_absgoal = constructInitialT_w_abs(cfg, path_base, pathSE3[0].rotation)

    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": pathSE3[0]})

    ##################################################
    # 3. get arms into comfy position
    ####################################################
    print("putting arms to a comfy configuration")
    robot.mode = AbstractRobotManager.control_mode.upper_body
    moveJP(robot.comfy_configuration, cfg, robot)

    ###################################################
    # 4. get base to starting pose
    ###################################################
    print("getting base to start of path")
    p_basegoal = path_base[0]
    # NOTE: requires passing a reduced model to work because crocoddyl needs the model to formulate itself
    # robot.mode = AbstractRobotManager.control_mode.base_only
    robot.mode = robot.control_mode.whole_body
    # CrocoBaseP2PMPC(cfg, robot, p_basegoal)

    # NOTE: alternative: navigate to start point and position arms simulatenously
    # NOTE: does not work well :( (would need finessing which is too time-consuming for what it's worth)
    # CrocoDualEEAndBaseP2PMPC(cfg, robot, pathSE3[0], T_absgoal_l, T_absgoal_r, p_base)

    ##########################################
    # orient base to look at the goal (yes this should've be handled with basep2p)
    ##########################################
    robot.mode = robot.control_mode.base_only
    # NOTE: T_w_absgoal's x-axis points toward the path.
    # z-axis points down by an undocumented choice
    # rotation_init_rpy = pin.rpy.matrixToRpy(T_w_absgoal.rotation)
    # rotation_init = pin.rpy.rpyToMatrix(0.0, 0.0, rotation_init_rpy[2] - np.pi / 2)
    # T_w_bgoal = pin.SE3(rotation_init, p_basegoal)
    T_w_bgoal = pin.SE3(init_rot, p_basegoal)
    moveL(cfg, robot, T_w_bgoal)

    ###################################################
    # grasp handlebar
    ###################################################
    # DualArmMoveL 10 cm above handlebar
    robot.mode = AbstractRobotManager.control_mode.upper_body
    T_w_absgoal.translation[2] += 0.1
    print("going above handlebar")
    DualArmIKSelfAvoidanceViaEndEffectorSpheres(
        T_w_absgoal, T_absgoal_l, T_absgoal_r, cfg, robot
    )
    # DualArmMoveL down to handlebar
    print("going down to handlebar")
    T_w_absgoal.translation[2] -= 0.1
    DualArmIKSelfAvoidanceViaEndEffectorSpheres(
        T_w_absgoal, T_absgoal_l, T_absgoal_r, cfg, robot
    )
    ###################################################
    # TODO: (5) grip handlebar with gripper (sleep before, or wait for keyboard input idk)
    ###################################################
    # time.sleep(5)

    # TODO: for final demo replace this with disjoint controller
    robot._mode = robot.control_mode.whole_body
    DualArmCartPulling(cfg, robot, path_planner, T_absgoal_l, T_absgoal_r)
    # DualArmCartPullingMPC(cfg, robot, path_planner, T_absgoal_l, T_absgoal_r)

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
