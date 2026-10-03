from smc import load_config, getRobotFromConfig
from smc.bookkeeping.load_config import GlobalConfig
from smc.vision.vision import processCamera
from smc.control.control_loop_manager import ControlLoopManager
from smc.multiprocessing.producer_process import ProducerProcess
from smc.robots.abstract_robotmanager import AbstractRobotManager
import numpy as np
from functools import partial


def controlLoopWithCamera(
    camera_manager: ProducerProcess,
    cfg: GlobalConfig,
    robot: AbstractRobotManager,
    i,
    past_data,
):
    """
    controlLoopWithCamera
    -----------------------------
    do nothing while getting dummy camera input
    """
    breakFlag = False
    log_item = {}
    save_past_dict = {}
    q = robot.q

    camera_output = camera_manager.getData()
    # print(camera_output)

    qd_cmd = np.zeros(robot.model.nv)

    robot.sendVelocityCommand(qd_cmd)

    log_item["qs"] = q.reshape((robot.model.nq,))
    log_item["vs"] = robot.v
    log_item["camera_output"] = np.array([camera_output["x"], camera_output["y"]])
    return breakFlag, save_past_dict, log_item


if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)

    init_value = {"x": np.random.randint(0, 10), "y": np.random.randint(0, 10)}
    camera_manager = ProducerProcess(cfg, processCamera, {}, init_value)

    log_item = {}
    log_item["qs"] = np.zeros((robot.model.nq,))
    log_item["vs"] = np.zeros((robot.model.nv,))
    log_item["camera_output"] = np.zeros(2)
    controlLoop = partial(controlLoopWithCamera, camera_manager, cfg, robot)
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, {}, log_item)
    loop_manager.run()

    camera_manager.terminateProcess()

    # get expected behaviour here (library can't know what the end is - you have to do this here)
    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()
