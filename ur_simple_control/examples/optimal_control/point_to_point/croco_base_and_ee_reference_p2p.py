from smc import load_config
from smc import getRobotFromConfig
from smc import load_config
from smc.control.optimal_control.util import get_OCP_cfg
from smc.control.cartesian_space import getClikArgs
from smc.control.optimal_control.croco_point_to_point.mpc.base_and_single_arm_reference_mpc import (
    CrocoEEAndBaseP2PMPC,
)
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.interfaces.single_arm_interface import SingleArmInterface

from pinocchio import SE3
import numpy as np


def get_cfg():
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    assert issubclass(robot.__class__, MobileBaseInterface)
    assert issubclass(robot.__class__, SingleArmInterface)
    # TODO: put this back for nicer demos
    # Mgoal = defineGoalPointCLI()
    T_w_goal = SE3.Random()
    p_basegoal = T_w_goal.translation.copy()
    p_basegoal += np.random.random(3) / 4
    p_basegoal[2] = 0.0

    if cfg.visualizer:
        # TODO: document defined viz messages somewhere
        robot.updateViz({"fixed_path": p_basegoal.reshape((1, 3))})
        robot.visualizer_manager.sendCommand({"Mgoal": T_w_goal})

    CrocoEEAndBaseP2PMPC(cfg, robot, T_w_goal, p_basegoal)

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
