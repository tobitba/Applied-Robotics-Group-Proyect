from smc.robots.utils import getRobotFromConfig, load_config
from smc.control.cartesian_space import (
    moveL,
    cartesianPathFollowingWithPlanner,
)
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.path_generation.planner import starPlanner, getPlanningArgs
from smc.multiprocessing import ProcessManager

from smc.path_generation.fixed_path_planner import (
    contructPath,
    trucatePathToClosestPathPoint,
)

import time
import numpy as np
from functools import partial
import pinocchio as pin

if __name__ == "__main__":

    raise NotImplementedError
    cfg = load_config()
    robot = getRobotFromConfig(cfg)

    print(f"cfg.planner = {cfg.planner}")

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
        path_planner = partial(trucatePathToClosestPathPoint, [0], path2D)

    pathSE3: list[pin.SE3] = path2D_to_SE3(path2D, cfg.path_height, cfg.path_rotx)
    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": pathSE3[0]})

    # go to start
    print(f"path length = {len(pathSE3)}")
    print(f"Moving to path start at {pathSE3[0]} with robot type {type(robot)}")
    moveL(cfg, robot, pathSE3[0])
    print("Reached path start.")

    # follow the path
    print(f"IK solver = {cfg.ik_solver}")
    print(f"path.rotx = {cfg.path_rotx}")
    cartesianPathFollowingWithPlanner(cfg, robot, path_planner, cfg.path_rotx)

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
