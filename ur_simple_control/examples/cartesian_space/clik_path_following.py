from smc import load_config
from smc.motion_planning import motion_planner
from smc.robots.utils import getRobotFromConfig
from smc.control.cartesian_space import (
    moveL,
)
from smc.control.cartesian_space.cartesian_space_trajectory_tracking import (
    cartesianTrajectoryTracking,
)
from smc.motion_planning.utilities.path2d_to_6d import path2D_to_SE3
from smc.motion_planning.interpolation.bsplinesSE3 import CubicBezierSplineSE3

from smc.motion_planning.path_planning.test_planners.draw_2d_path import (
    drawOrLoad2DPath,
)
from smc.motion_planning.trajectory_generation.end_effector_velocity_limit_scaling import (
    UniformTimeLawWithEEContraints,
)
from smc.motion_planning.motion_planner import MotionPlannerFixedPath

import numpy as np
from pinocchio import SE3

if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)

    path3D = drawOrLoad2DPath(cfg.test_path)

    T_list: list[SE3] = path2D_to_SE3(
        path3D, cfg.test_path.path_height, cfg.test_path.path_rotx
    )
    V_start = np.zeros(6)
    V_end = np.zeros(6)
    smooth_path = CubicBezierSplineSE3(T_list, V_start, V_end)

    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": T_list[0]})
        viz_Ts = []
        ss = np.linspace(0, 1, 500)
        for s in ss:
            viz_Ts.append(smooth_path.getPoint(s))
        robot.visualizer_manager.sendCommand({"framepath": viz_Ts})

    # construct timing law
    time_law = UniformTimeLawWithEEContraints(
        smooth_path,
    )
    time_law.T_total = time_law.T_total * 4

    motion_planner = MotionPlannerFixedPath(smooth_path, time_law)

    # go to start
    print("going to path start")
    moveL(cfg, robot, T_list[0])

    print("starting to follow path")
    cartesianTrajectoryTracking(motion_planner, cfg, robot)

    robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
