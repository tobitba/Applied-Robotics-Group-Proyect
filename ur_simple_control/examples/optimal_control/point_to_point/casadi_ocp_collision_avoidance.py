from smc import load_config
# PYTHON_ARGCOMPLETE_OK
import pinocchio as pin
import numpy as np
import time
import argparse
from functools import partial
from smc.managers import load_config, ControlLoopManager, RobotManager
from smc.optimal_control.create_pinocchio_casadi_ocp import (
    createCasadiIKObstacleAvoidanceOCP,
)
from smc.optimal_control.get_ocp_cfg import get_OCP_cfg
from smc.basics.basics import followKinematicJointTrajP
import argcomplete


def get_cfg():
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser.description = "optimal control in pinocchio.casadi for obstacle avoidance"
    # add more arguments here from different Simple Manipulator Control modules
    argcomplete.autocomplete(parser)
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    robot = RobotManager(cfg)
    # T_goal = robot.defineGoalPointCLI()
    # T_goal = pin.SE3.Random()
    T_goal = robot.defineGoalPointCLI()
    T_goal.rotation = robot.getT_w_e().rotation
    if cfg.visualize_manipulator:
        robot.updateViz({"Mgoal": T_goal})
    reference, opti = createCasadiIKObstacleAvoidanceOCP(cfg, robot, T_goal)
    followKinematicJointTrajP(cfg, robot, reference)

    # get expected behaviour here (library can't know what the end is - you have to do this here)
    if not cfg.pinocchio_only:
        robot.stopRobot()

    if cfg.save_log:
        robot.log_manager.plotAllControlLoops()

    if cfg.visualize_manipulator:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot.log_manager.saveLog()
