from smc import load_config
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.util.draw_path import drawPath

from argparse import Namespace
import matplotlib
import numpy as np


def contructPath(cfg: MasterConfig, robot: AbstractRobotManager):
    if not cfg.draw_new:
        pixel_path_file_path = "./parameters/path_in_pixels.csv"
        path_base = np.genfromtxt(pixel_path_file_path, delimiter=",")
    else:
        matplotlib.use("tkagg")
        path_base = drawPath(cfg)
        matplotlib.use("qtagg")
    path_base[:, 0] = path_base[:, 0] * cfg.map_width
    path_base[:, 1] = path_base[:, 1] * cfg.map_height
    path_base = np.hstack((path_base, np.zeros((len(path_base), 1))))
    robot.updateViz({"fixed_path": path_base})
    return path_base


def fixedPathPlanner(
    path_parameter: list[int],
    path_base: np.ndarray,
    p_base: np.ndarray,
) -> np.ndarray:
    if p_base.shape[0] == 2:
        p_base = np.append(p_base, 0.0)
    distances = np.linalg.norm(p_base - path_base, axis=1)
    index = np.argmin(distances)
    # NOTE: bullshit heuristic to deal with the path intersecting with itself.
    # i can't be bothered with anything better
    if (index - path_parameter[0]) > 0 and (index - path_parameter[0]) < 10:
        path_parameter[0] += 1
    path_base = path_base[path_parameter[0] :]
    return path_base
