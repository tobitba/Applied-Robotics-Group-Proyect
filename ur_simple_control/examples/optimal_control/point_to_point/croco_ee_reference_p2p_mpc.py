from smc import load_config, getRobotFromConfig
from smc.control.optimal_control.croco_point_to_point.mpc.single_arm_reference_mpc import (
    CrocoEEP2PMPC,
)
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI

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

    if cfg.visualizer:
        # TODO: document this somewhere
        robot.visualizer_manager.sendCommand({"Mgoal": T_w_goal})

    CrocoEEP2PMPC(cfg, robot, T_w_goal)

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()
