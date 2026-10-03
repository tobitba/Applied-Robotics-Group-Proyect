from smc import load_config
from smc import load_config, getRobotFromConfig
from smc.control.optimal_control.croco_point_to_point.mpc.dual_arm_reference_mpc import (
    CrocoDualEEP2PMPC,
)
from smc.control.optimal_control.util import get_OCP_cfg
from smc.control.cartesian_space import getClikArgs
from smc.util.define_random_goal import getRandomlyGeneratedGoal
import pinocchio as pin
import argcomplete


def get_cfg():
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    argcomplete.autocomplete(parser)
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    T_w_absgoal = getRandomlyGeneratedGoal(cfg, robot)
    T_absgoal_lgoal = pin.SE3.Identity()
    T_absgoal_lgoal.translation[1] = 0.1
    T_absgoal_rgoal = pin.SE3.Identity()
    T_absgoal_rgoal.translation[1] = -0.1

    if cfg.visualizer:
        # TODO: document this somewhere
        robot.visualizer_manager.sendCommand({"Mgoal": T_w_absgoal})

    CrocoDualEEP2PMPC(cfg, robot, T_w_absgoal, T_absgoal_lgoal, T_absgoal_rgoal)

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()
