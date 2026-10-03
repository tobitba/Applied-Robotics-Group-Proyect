from smc import load_config
from smc import getRobotFromConfig
from smc.control.optimal_control.croco_path_following.mpc.base_and_single_arm_reference_mpc import (
    BaseAndEEPathFollowingMPC,
)
from os.path import exists
from smc.control.optimal_control.croco_point_to_point.mpc.base_and_single_arm_reference_mpc import (
    CrocoEEAndBaseP2PMPC,
)
from smc.path_generation.planner import starPlanner
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.multiprocessing import ProcessManager

from pinocchio import SE3
import numpy as np
from functools import partial
import matplotlib
from smc.util.draw_path import drawPath


# NOTE: lil bit of evil with passing the integer by reference -> make it a list
# to make this non-evil this should be a method in a class but i ain't gonna bother with that now
def fixedPathPlanner(
    path_parameter: list[int],
    path2D: np.ndarray,
    rotation: np.ndarray,
    base_offset: np.ndarray,
    T_w_e: SE3,
) -> tuple[list[SE3], list[np.ndarray]]:
    p_ee = T_w_e.translation
    distances = np.linalg.norm(p_ee - path2D, axis=1)
    index = np.argmin(distances)
    # if index > path_parameter[0]:
    # NOTE: bullshit heuristic to deal with the path intersecting with itself.
    # i can't be bothered with anything better
    if (index - path_parameter[0]) > 0 and (index - path_parameter[0]) < 10:
        path_parameter[0] += 1
    path2D = path2D[path_parameter[0] :]
    # NOTE: this is of course horribly inefficient,
    # but since it's just for testing who cares
    path_SE3 = []
    path_base = []
    for translation in path2D:
        path_base.append(translation + base_offset)
        path_SE3.append(SE3(rotation, translation))
    return path_base, path_SE3


if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    # TODO: make this go away
    robot._step()
    assert issubclass(robot.__class__, SingleArmInterface)
    assert issubclass(robot.__class__, MobileBaseInterface)
    x0 = np.concatenate([robot.q, robot.v])
    rotation = SE3.Random().rotation
    height = 0.5
    base_offset = np.array([0.0, 0.4, 0.0])
    goal = np.array([0.5, 5.5])
    T_w_e = robot.T_w_e

    if cfg.planner:
        planning_function = partial(starPlanner, goal)
        # here we're following T_w_e reference so that's what we send
        path_planner = ProcessManager(
            cfg, planning_function, T_w_e.translation[:2], 3, None
        )
        _, map_as_list = createSampleStaticMap()
        if cfg.visualizer:
            robot.sendRectangular2DMapToVisualizer(map_as_list)
            # time.sleep(5)
    else:
        if not cfg.draw_new:
            assert exists("./parameters/path_in_pixels.csv")
            pixel_path_file_path = "./parameters/path_in_pixels.csv"
            path2D = np.genfromtxt(pixel_path_file_path, delimiter=",")
        else:
            matplotlib.use("tkagg")
            path2D = drawPath(cfg)
            matplotlib.use("qtagg")
        path2D[:, 0] = path2D[:, 0] * cfg.map_width
        path2D[:, 1] = path2D[:, 1] * cfg.map_height
        path2D = np.hstack((path2D, height * np.ones((len(path2D), 1))))
        robot.updateViz({"fixed_path": path2D})
        T_w_goal = SE3(rotation, path2D[0])
        p_basegoal = T_w_goal.copy().translation.copy() + base_offset
        p_basegoal[2] = 0.0
        CrocoEEAndBaseP2PMPC(cfg, robot, T_w_goal, p_basegoal)
        # note: making it a list is a lil'bit of evil to make it persistent in memory
        # jesus i really need to switch to cpp to pass by reference or value at will
        path_parameter = [0]
        path_planner = partial(
            fixedPathPlanner, path_parameter, path2D, rotation, base_offset
        )

    BaseAndEEPathFollowingMPC(cfg, robot, path_planner)

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
