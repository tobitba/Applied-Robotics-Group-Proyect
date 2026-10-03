# TODO: the same idea as signal generator in planar manipulation.
# basically the same, except now these curves represent paths instead
# of control inputs. you can basically copy-paste that here.
# have few path classes, like line and sinusoid.
# for sinusoid you decide
# add optional noise to that so that path filtering actually has something to do.
# this is important so that we can deal with rrt* outputs for example

from smc.motion_planning.path_planning.path_planner import AbstractPathPlanner
from smc.motion_planning.path_planning.test_planners.test_planner_config import (
    ConfigTestPathGenerator,
    SignalShape,
)

import numpy as np
from pinocchio import SE3, rpy
from copy import deepcopy

from smc.bookkeeping.types import (
    CartesianReference,
    Points2D,
    Points3D,
    PointsSE3,
    Point2D,
    Point3D,
)


class Test2DPathGenerator(AbstractPathPlanner[Points2D]):
    def __init__(self, cfg: ConfigTestPathGenerator):
        self.signal_shape: SignalShape
        self.path_points: np.ndarray
        if cfg.test_path_shape == "straight":
            self.signal_shape = SignalShape.straight
        if cfg.test_path_shape == "sinusoidal":
            self.signal_shape = SignalShape.sinusoidal
        self.test_path_point_density = cfg.test_path_point_density
        self.test_path_length = cfg.test_path_length
        self.computePath(np.zeros(2))

    def computePath(self, start_point: Point2D):
        angle = np.random.random() * 2 * np.pi
        line1D = np.linspace(0.0, self.test_path_length, self.test_path_point_density)
        if self.signal_shape == SignalShape.straight:
            self.path_points = (
                start_point
                + np.vstack((np.cos(angle) * line1D, np.sin(angle) * line1D)).T
            )
        # NOTE: these vary quite a bit in arclength.
        # since i'm not sure how important this is, i'll just leave it as is for how
        if self.signal_shape == SignalShape.sinusoidal:
            amplitude = np.random.random() * 2
            frequency = np.random.random() * 4
            offset = np.random.random() * 2 * np.pi * frequency
            # TODO: idk if it's times frequency
            str8 = line1D.copy()
            line1D = amplitude * np.sin(line1D * frequency + offset)
            line1D = line1D - line1D[0]
            R = np.array(
                [[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]]
            )
            self.path_points = start_point + np.vstack((line1D, str8)).T
            for i in range(len(self.path_points)):
                self.path_points[i] = R @ self.path_points[i]
            # + np.vstack((np.cos(angle) * line1D, np.sin(angle) * line1D)).T

    def getPathPoints(self):
        return deepcopy(self.path_points)


# NOTE: now it's the motion planner's task to
# apply interpolation to this
class TestSE3PathGenerator(AbstractPathPlanner[PointsSE3]):
    def __init__(self, cfg: ConfigTestPathGenerator):
        # TODO: make heights, widths, arclength etc parameters/arguments
        self.T_list: PointsSE3 = []

    # TODO: this should have a heading also
    def computePath(self, start_point: SE3):
        self.T_list.clear()
        self.T_list.append(start_point.copy())
        for i in range(10):
            rpy_rot = np.random.random(3) * 1.5
            rpy_rot[0] += np.pi
            R = rpy.rpyToMatrix(rpy_rot)
            translation = np.zeros(3)
            translation[0] = (i + 1) / 5 + np.random.random() * 0.1
            translation[1] = np.random.random() * ((i + 1) / 5)
            translation[2] = 1.1 + np.random.random() * 0.1
            self.T_list.append(SE3(R, translation))

    def getPathPoints(self) -> PointsSE3:
        return deepcopy(self.T_list)


if __name__ == "__main__":
    from smc import load_config
    import meshcat_shapes
    from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
    import time

    visualizer = MeshcatVisualizer()
    cfg = load_config()
    path_generator2D = Test2DPathGenerator(cfg.test_path)
    path_points = path_generator2D.getPathPoints()
    path_points = np.hstack((path_points, np.zeros((cfg.test_path.point_density, 1))))
    visualizer.addPath("path", path_points)
    time.sleep(3)

    for i in range(100):
        path_generator2D.generatePath(np.zeros(2))
        path_points = path_generator2D.getPathPoints()
        path_points = np.hstack(
            (path_points, np.zeros((cfg.test_path.point_density, 1)))
        )
        visualizer.addPath("path", path_points)
        time.sleep(3)
