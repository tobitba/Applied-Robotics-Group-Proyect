from smc import load_config, getRobotFromConfig
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI
from smc.control.cartesian_space.cartesian_space_compliant_control import compliantMoveL
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)

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
    robot.closeGripper()
    robot.openGripper()
    compliantMoveL(cfg, robot, T_w_goal)

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
