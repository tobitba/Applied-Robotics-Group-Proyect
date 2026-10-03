from smc import load_config
from smc.bookkeeping.types import PastData
from smc.multiprocessing.networking.client import client_receiver
from smc.multiprocessing import NetworkClientProcess
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.robots.utils import getRobotFromConfig
from smc.control import ControlLoopManager

import numpy as np
from functools import partial


def controlLoopExternalQ(
    robot: AbstractRobotManager,
    receiver: NetworkClientProcess,
    _: int,
    __: PastData,
):
    """
    controlLoop
    -----------------------------
    controller description
    """
    breakFlag = False
    log_item = {}
    save_past_dict = {}

    q = robot.q

    q_desired = receiver.getData()["q"]
    q_error = q_desired - q
    K = 10
    qd_cmd = K * q_error

    robot.sendVelocityCommand(qd_cmd)

    log_item["qs"] = q.reshape((robot.model.nq,))
    log_item["vs"] = robot.v.reshape((robot.model.nv,))
    log_item["q_error"] = q_error.reshape((robot.model.nq,))
    return breakFlag, save_past_dict, log_item


if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)

    # get expected behaviour here (library can't know what the end is - you have to do this here)
    if not cfg.real:
        robot.stopRobot()

    # VERY important that the first q we'll pass as desired is the current q, meaning the robot won't move
    # this is set with init_value
    side_cli = partial(client_receiver, cfg.net.host, cfg.net.port)
    receiver = NetworkClientProcess(
        cfg,
        side_cli,
        {"q": robot.q.copy()},
    )
    log_item = {
        "qs": np.zeros((robot.model.nq,)),
        "vs": np.zeros((robot.model.nv,)),
        "q_error": np.zeros((robot.model.nq,)),
    }
    control_loop = partial(controlLoopExternalQ, robot, receiver)
    loop_manager = ControlLoopManager(robot, control_loop, cfg, {}, log_item)
    loop_manager.run()

    if cfg.save_log:
        robot.log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot.log_manager.saveLog()
    # loop_manager.stopHandler(None, None)
