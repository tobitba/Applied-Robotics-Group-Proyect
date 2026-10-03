from smc import load_config, getRobotFromConfig
from smc.control.freedrive import freedriveUntilKeyboard
import pickle

if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)

    pose_n_q_dict = freedriveUntilKeyboard(cfg, robot)
    file = open("./pose_n_q_dict.pickle", "wb")
    pickle.dump(pose_n_q_dict, file)
    file.close()
    print(pose_n_q_dict)

    # get expected behaviour here (library can't know what the end is - you have to do this here)
    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
