from smc import load_config
from argparse import Namespace
from smc.multiprocessing.networking.client import client_receiver, client_sender
from smc.multiprocessing.networking.server import server_sender
from smc.multiprocessing.process_manager import ProcessManager
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.control.cartesian_space.ik_solvers import dampedPseudoinverse, getIKSolver
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL
from smc.control.cartesian_space.utils import getClikArgs
from smc.control.control_loop_manager import ControlLoopManager
from smc import (
    load_config,
    getRobotFromConfig,
)

import pinocchio as pin
import numpy as np
from functools import partial
from collections import deque


def get_cfg():
    parser = load_config()
    parser = getClikArgs(parser)
    parser.description = "the robot will received joint angles from a socket and go to them in joint space"
    # add more arguments here from different Simple Manipulator Control modules
    parser.add_argument("--host", type=str, help="host ip address", default="127.0.0.1")
    parser.add_argument("--port-wrench", type=int, help="host's port", default=6666)
    parser.add_argument("--port-command", type=int, help="host's port", default=7777)
    # put your own default here
    # it's translation + orientation as a quarternion
    parser.add_argument(
        "--init-pose", type=float, ncfg=7, default=[0.3, 0.3, 0.3, 0.0, 0.0, 0.0, 1.0]
    )
    cfg = parser.parse_cfg()
    return cfg


def controlLoopClikExternalGoal(
    ik_solver,
    receiver: ProcessManager,
    sender: ProcessManager,
    cfg: MasterConfig,
    robot: ForceTorqueOnSingleArmWrist,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    controlLoop
    -----------------------------
    controller description
    """
    breakFlag = False
    log_item = {}
    save_past_dict = {}

    T_w_e = robot.T_w_e
    data = receiver.getData()
    T_goal = data["T_goal"]
    SEerror = T_w_e.actInv(T_goal)
    err_vector = pin.log6(SEerror).vector
    K = np.diag([30, 30, 30, 15, 15, 15])

    qd_cmd = (
        J.T
        @ np.linalg.inv(J @ J.T + np.eye(J.shape[0], J.shape[0]) * cfg.tikhonov_damp)
        @ (K @ err_vector + data["v"])
    )
    if qd_cmd is None:
        print("the controller you chose didn't work, using dampedPseudoinverse instead")
        qd_cmd = dampedPseudoinverse(cfg, robot, err_vector)

    robot.sendVelocityCommand(qd_cmd)
    # NOTE: for the specific use-case we're sending wrench in robot's base frame
    sender.sendCommand({"wrench": robot.wrench})

    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["err_vector"] = err_vector
    return breakFlag, save_past_dict, log_item


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    translation = np.array(cfg.init_pose[:3])
    orientation = pin.Quaternion(np.array(cfg.init_pose[3:]))
    init_goal = pin.SE3(orientation, translation)
    moveL(cfg, robot, init_goal)
    print("arrived at initial position")

    # VERY important that the first target we'll pass as desired
    # is the current pose, meaning the robot won't move
    # this is set via init_value
    # command_sender: 7777
    receiver_fun = partial(client_receiver, cfg.host, cfg.port_command)
    receiver = ProcessManager(
        cfg,
        receiver_fun,
        {"T_goal": pin.SE3.Identity(), "v": np.zeros(6)},
        4,
        init_value={"T_goal": robot.T_w_e, "v": np.zeros(6)},
    )
    # wrench_sender: 6666
    sender_fun = partial(server_sender, cfg.host, cfg.port_wrench)
    sender = ProcessManager(cfg, sender_fun, {"wrench": np.zeros(6)}, 0)
    log_item = {
        "qs": np.zeros((robot.model.nq,)),
        "vs": np.zeros((robot.model.nv,)),
        "err_vector": np.zeros((6,)),
    }
    clik_controller = getIKSolver(cfg, robot)
    control_loop = partial(
        controlLoopClikExternalGoal,
        clik_controller,
        receiver,
        sender,
        cfg,
        robot,
    )
    loop_manager = ControlLoopManager(robot, control_loop, cfg, {}, log_item)
    loop_manager.run()

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
