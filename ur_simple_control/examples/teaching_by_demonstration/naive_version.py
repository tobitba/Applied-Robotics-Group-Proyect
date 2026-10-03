from smc import load_config
from smc import load_config, getRobotFromConfig
from smc.control.freedrive import freedriveUntilKeyboard
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL
from smc.control.cartesian_space import getClikArgs
import pickle


def get_cfg():
    parser = load_config()
    parser = getClikArgs(parser)
    parser.description = (
        "freedrive with keyboard bindings to save poses and joint angles"
    )
    # add more arguments here from different Simple Manipulator Control modules
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)

    try:
        file = open("./pose_n_q_dict.pickle", "rb")
        pose_n_q_dict = pickle.load(file)
        file.close()
    except FileNotFoundError:
        pose_n_q_dict = freedriveUntilKeyboard(cfg, robot)
        file = open("./pose_n_q_dict.pickle", "wb")
        pickle.dump(pose_n_q_dict, file)
        file.close()
    print(pose_n_q_dict)

    for T_w_goal in pose_n_q_dict["T_w_es"]:
        moveL(cfg, robot, T_w_goal)

    # get expected behaviour here (library can't know what the end is - you have to do this here)
    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
