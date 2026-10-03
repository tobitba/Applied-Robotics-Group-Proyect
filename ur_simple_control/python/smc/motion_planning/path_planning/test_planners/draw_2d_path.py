from smc.util.draw_path import drawPath
from smc.motion_planning.interpolation.preprocessing import reduce2DPathPoints
from smc.motion_planning.path_planning.test_planners.test_planner_config import (
    ConfigTestPathGenerator,
)

import matplotlib
import numpy as np


def drawOrLoad2DPath(cfg: ConfigTestPathGenerator):
    if not cfg.draw_new:
        pixel_path_file_path = "./parameters/path_in_pixels.csv"
        path_base = np.genfromtxt(pixel_path_file_path, delimiter=",")
    else:
        matplotlib.use("tkagg")
        path_base = drawPath()
        matplotlib.use("qtagg")
    path_base[:, 0] = path_base[:, 0] * cfg.map_width
    path_base[:, 1] = path_base[:, 1] * cfg.map_height
    path_base = reduce2DPathPoints(cfg.minimum_distance_between_points, path_base)
    path_base = np.hstack((path_base, np.zeros((len(path_base), 1))))
    return path_base
