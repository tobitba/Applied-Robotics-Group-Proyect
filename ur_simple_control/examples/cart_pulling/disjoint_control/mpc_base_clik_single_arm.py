from smc import load_config
from smc import getRobotFromConfig
from smc import load_config
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.control.optimal_control.util import get_OCP_cfg
from smc.control.cartesian_space import getClikArgs
from smc.path_generation.planner import starPlanner, getPlanningArgs
from smc.control.optimal_control.croco_point_to_point.mpc.base_and_single_arm_reference_mpc import (
    CrocoEEAndBaseP2PMPC,
)
from smc.multiprocessing import ProcessManager
from mpc_base_clik_single_arm_control_loop import BaseMPCANDEECLIKCartPulling

import time
import numpy as np
from functools import partial
import pinocchio as pin


def get_cfg():
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getPlanningArgs(parser)
    parser.add_argument(
        "--handlebar-height",
        type=float,
        default=0.5,
        help="heigh of handlebar of the cart to be pulled",
    )
    parser.add_argument(
        "--base-to-handlebar-preferred-distance",
        type=float,
        default=0.5,
        help="prefered path arclength from mobile base position to handlebar",
    )
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    # TODO: HOW IS IT POSSIBLE THAT T_W_E IS WRONG WITHOUT STEP CALLED HERE?????????????????
    robot._step()
    T_w_e = robot.T_w_e
    robot._q[0] = 9.0
    robot._q[1] = 4.0
    robot._step()
    x0 = np.concatenate([robot.q, robot.v])
    goal = np.array([0.5, 5.5])

    planning_function = partial(starPlanner, goal)
    # here we're following T_w_e reference so that's what we send
    path_planner = ProcessManager(
        cfg, planning_function, T_w_e.translation[:2], 3, None
    )
    _, map_as_list = createSampleStaticMap()
    if cfg.visualizer:
        robot.sendRectangular2DMapToVisualizer(map_as_list)
        # time.sleep(5)

    T_w_e = robot.T_w_e
    data = None
    # get first path
    ##########################################3
    #                initialize
    ###########################################
    while data is None:
        path_planner.sendCommand(T_w_e.translation[:2])
        data = path_planner.getData()
        # time.sleep(1 / cfg.ctrl_freq)
        time.sleep(1)

    _, path2D = data
    path2D = np.array(path2D).reshape((-1, 2))
    pathSE3 = path2D_to_SE3(path2D, cfg.handlebar_height, np.pi)
    if cfg.visualizer:
        # TODO: document this somewhere
        robot.visualizer_manager.sendCommand({"Mgoal": pathSE3[0]})
    if np.linalg.norm(pin.log6(T_w_e.actInv(pathSE3[0]))) > 1e-2:
        print("going to initial path position")
        p_base = pathSE3[0].translation.copy()
        p_base[0] -= cfg.base_to_handlebar_preferred_distance
        p_base[2] = 0.0
        print(pathSE3[0].translation)
        print(p_base)
        # TODO: UNCOMMENT
        CrocoEEAndBaseP2PMPC(cfg, robot, pathSE3[0], p_base)
    print("initialized!")
    BaseMPCANDEECLIKCartPulling(cfg, robot, path_planner)

    print("final position:", robot.T_w_e)

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
