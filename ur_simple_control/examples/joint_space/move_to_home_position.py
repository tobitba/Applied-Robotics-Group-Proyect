from smc import load_config, getRobotFromConfig
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.control.joint_space.joint_space_point_to_point import moveJPWTraj

if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)

    # NOTE: don't use joint-space control for mobile robots
    robot.mode = AbstractRobotManager.control_mode.upper_body
    # moveJP(robot.comfy_configuration, cfg, robot)
    moveJPWTraj(robot.comfy_configuration, cfg, robot)
    # moveJPWTraj(robot.calib_pose[4:], cfg, robot)

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
