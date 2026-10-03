from smc import load_config, getRobotFromConfig
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI
from smc.control.cartesian_space.cartesian_space_point_to_point import moveLDualArm

import numpy as np
import pinocchio as pin

if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    rotation = pin.rpy.rpyToMatrix(np.array([np.pi, 0.0, 0.0]))
    if cfg.randomly_generate_goal:
        T_w_absgoal = getRandomlyGeneratedGoal(cfg, robot)
        if cfg.visualizer:
            robot.visualizer_manager.sendCommand({"Mgoal": T_w_absgoal})
    else:
        T_w_absgoal = defineGoalPointCLI(robot)
        T_w_absgoal.rotation = rotation
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15
    moveLDualArm(cfg, robot, T_w_absgoal, T_absgoal_l, T_absgoal_r)
    print("moveLDualArm done")

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()
