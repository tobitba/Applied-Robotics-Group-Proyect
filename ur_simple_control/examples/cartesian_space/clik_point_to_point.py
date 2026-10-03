from smc import load_config
from smc import getRobotFromConfig
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL

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
    moveL(cfg, robot, T_w_goal)
    robot.closeGripper()
    robot.openGripper()

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
