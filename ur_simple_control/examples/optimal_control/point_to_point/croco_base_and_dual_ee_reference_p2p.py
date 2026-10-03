from smc import load_config
from smc import getRobotFromConfig
from smc import load_config
from smc.control.optimal_control.util import get_OCP_cfg
from smc.control.cartesian_space import getClikArgs
from smc.control.optimal_control.croco_point_to_point.mpc.base_and_dual_arm_reference_mpc import (
    CrocoDualEEAndBaseP2PMPC,
)
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.util.define_random_goal import getRandomlyGeneratedGoal

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
    assert issubclass(robot.__class__, DualArmInterface)
    # TODO: put this back for nicer demos
    # Mgoal = defineGoalPointCLI()
    T_w_absgoal = getRandomlyGeneratedGoal(cfg, robot)
    T_absgoal_l = SE3.Identity()
    T_absgoal_l.translation[1] = 0.1
    T_absgoal_r = SE3.Identity()
    T_absgoal_r.translation[1] = -0.1
    p_basegoal = T_w_absgoal.translation.copy()
    p_basegoal += np.random.random(3) / 4
    p_basegoal[2] = 0.0

    if cfg.visualizer:
        # TODO: document defined viz messages somewhere
        robot.updateViz({"fixed_path": p_basegoal.reshape((1, 3))})
        robot.visualizer_manager.sendCommand({"Mgoal": T_w_absgoal})

    CrocoDualEEAndBaseP2PMPC(
        cfg, robot, T_w_absgoal, T_absgoal_l, T_absgoal_r, p_basegoal
    )

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
