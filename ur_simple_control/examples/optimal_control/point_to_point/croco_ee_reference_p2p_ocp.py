from smc import load_config, getRobotFromConfig
from smc.control.optimal_control.croco_point_to_point.ocp.single_arm_reference_ocp import (
    SingleArmIKOCP,
)
from smc.control.joint_space import followKinematicJointTrajP
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI

import numpy as np

if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    if (not cfg.real) and cfg.randomly_generate_goal:
        T_w_goal = getRandomlyGeneratedGoal(cfg, robot)
        if cfg.visualizer:
            robot.visualizer_manager.sendCommand({"Mgoal": T_w_goal})
    else:
        if cfg.real and cfg.randomly_generate_goal:
            print("Ain't no way you're going to a random goal on the real robot!")
            print("Look at the current pose, define something appropriate manually")
        T_w_goal = defineGoalPointCLI(robot)

    # create and solve the optimal control problem of
    # getting from current to goal end-effector position.
    # reference is position and velocity reference (as a dictionary),
    # while solver is a crocoddyl object containing a lot more information
    # starting state
    x0 = np.concatenate([robot.q, robot.v])
    # this shouldn't really depend on x0 but i can't be bothered
    ocp = SingleArmIKOCP(cfg.ocp, robot, x0, T_w_goal)
    ocp.solveInitialOCP(x0)
    reference = ocp.getSolvedReference()

    # NOTE: IF YOU PLOT SOMETHING OTHER THAN REAL-TIME PLOTTING FIRST IT BREAKS EVERYTHING
    #    if cfg.solver == "boxfddp":
    #        log = solver.getCallbacks()[1]
    #        crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=True)

    # we need a way to follow the reference trajectory,
    # both because there can be disturbances,
    # and because it is sampled at a much lower frequency
    followKinematicJointTrajP(cfg, robot, reference, True)

    # print("final position:")
    # print(robot.T_w_e)

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()
