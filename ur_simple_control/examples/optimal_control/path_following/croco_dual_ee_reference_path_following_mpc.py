from os.path import exists
from smc import getRobotFromConfig
from smc.control.cartesian_space.cartesian_space_point_to_point import moveLDualArm
from smc.control.optimal_control.croco_path_following.mpc.dual_arm_reference_mpc import (
    CrocoDualArmEEPathFollowingMPC,
)
from smc.path_generation.planner import starPlanner
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.multiprocessing import CollaborativeProcess

from pinocchio import SE3
import numpy as np
from functools import partial
import matplotlib
from smc.util.draw_path import drawPath


# NOTE: lil bit of evil with passing the integer by reference -> make it a list
# to make this non-evil this should be a method in a class but i ain't gonna bother with that now
def fixedPathPlanner(
    path_parameter: list[int], path2D: np.ndarray, rotation: np.ndarray, T_w_e: SE3
) -> list[SE3]:
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
    for translation in path2D:
        path_SE3.append(SE3(rotation, translation))
    return path_SE3


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    # TODO: make this go away
    robot._step()
    assert issubclass(robot.__class__, DualArmInterface)
    if cfg.planner:
        robot._q[0] = 9.0
        robot._q[1] = 4.0
        robot._step()
    x0 = np.concatenate([robot.q, robot.v])
    # rotation = SE3.Random().rotation
    rotation = SE3.Identity().rotation
    height = 0.5
    goal = np.array([0.5, 5.5])
    T_w_abs = robot.T_w_abs
    T_absgoal_l = SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = SE3.Identity()
    T_absgoal_r.translation[1] = -0.15

    if cfg.planner:
        planning_function = partial(starPlanner, goal)
        # we define the path using the T_w_abs frame,
        # from which the controller constructs T_w_l and T_w_r references to follow
        path_planner = CollaborativeProcess(
            cfg, planning_function, T_w_abs.translation[:2], 3, None
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
        T_w_absgoal = SE3(rotation, path2D[0])
        moveLDualArm(cfg, robot, T_w_absgoal, T_absgoal_l, T_absgoal_r)
        # and if that doesn't work either use pink:
        # DualArmIKSelfAvoidanceViaEndEffectorSpheres(
        #    T_w_absgoal, T_absgoal_l, T_absgoal_r, cfg, robot
        # )
        # NOTE: if this doesn't work just switch to moveL dual arm:
        # CrocoDualEEP2PMPC(cfg, robot, T_w_goal, T_absgoal_l, T_absgoal_r)

        # note: making it a list is a lil'bit of evil to make it persistent in memory
        # jesus i really need to switch to cpp to pass by reference or value at will
        path_parameter = [0]
        path_planner = partial(fixedPathPlanner, path_parameter, path2D, rotation)

    CrocoDualArmEEPathFollowingMPC(
        cfg, robot, T_absgoal_l, T_absgoal_r, x0, path_planner
    )

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
