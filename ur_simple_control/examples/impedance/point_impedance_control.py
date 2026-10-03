from smc import load_config
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc import load_config, getRobotFromConfig
from smc.control import ControlLoopManager
from smc.control.cartesian_space import (
    getClikArgs,
    getIKSolver,
)

import pinocchio as pin
import numpy as np
import copy
import argparse
from functools import partial
from typing import Any, Callable
from collections import deque


def getArgs():
    parser = load_config()
    parser = getClikArgs(parser)
    parser.add_argument(
        "--cartesian-space-impedance",
        action=argparse.BooleanOptionalAction,
        help="is the impedance computed and added in cartesian or in joint space",
        default=False,
    )

    cfg = parser.parse_cfg()
    return cfg


# control loop to be passed to ControlLoopManager
def JointSpacePointImpedanceControlLoop(
    q_init: np.ndarray,
    cfg: argparse.Namespace,
    robot: ForceTorqueOnSingleArmWrist,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    breakFlag = False
    # TODO rename this into something less confusing
    save_past_dict = {}
    log_item = {}
    q = robot.q
    wrench = robot.wrench
    log_item["wrench_raw"] = wrench
    # deepcopy for good coding practise (and correctness here)
    save_past_dict["wrench"] = copy.deepcopy(wrench)
    # rolling average
    # wrench = np.average(np.array(past_data['wrench']), axis=0)
    # first-order low pass filtering instead
    # beta is a smoothing coefficient, smaller values smooth more, has to be in [0,1]
    # wrench = cfg.beta * wrench + (1 - cfg.beta) * past_data['wrench'][-1]
    wrench = cfg.beta * wrench + (1 - cfg.beta) * np.average(
        np.array(past_data["wrench"]), axis=0
    )
    if not cfg.z_only:
        Z = np.diag(np.array([1.0, 1.0, 1.0, 5.0, 5.0, 5.0]))
    else:
        Z = np.diag(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))

    wrench = Z @ wrench

    J = robot.getJacobian()
    tau = J.T @ wrench
    # compute control law:
    # - feedback the position
    # kv is not needed if we're running velocity control
    # pin.difference is here for the mobile base case where nq =/= nv
    # output is what takes you from q to q_init (i.e. q_init \circminus q))
    v_cmd = cfg.kp * pin.difference(robot.model, q, q_init) + cfg.alpha * tau
    # immediatelly stop if something weird happened (some non-convergence)
    robot.sendVelocityCommand(v_cmd)

    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["wrench_used"] = wrench
    return breakFlag, save_past_dict, log_item


def controlLoopCartesianPointImpedance(
    ik_solver: Callable[[np.ndarray, np.ndarray], np.ndarray],
    Mtool_init: pin.SE3,
    cfg: argparse.Namespace,
    robot: ForceTorqueOnSingleArmWrist,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:

    breakFlag = False
    save_past_dict = {}
    log_item = {}
    T_w_e = robot.T_w_e
    wrench = robot.wrench
    log_item["wrench_raw"] = wrench.reshape((6,))
    save_past_dict["wrench"] = copy.deepcopy(wrench)

    wrench = cfg.beta * wrench + (1 - cfg.beta) * np.average(
        np.array(past_data["wrench"]), axis=0
    )
    # good generic values
    if not cfg.z_only:
        Z = np.diag(np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]))
    else:
        Z = np.diag(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    wrench = Z @ wrench

    SEerror = T_w_e.actInv(Mtool_init)
    err_vector = pin.log6(SEerror).vector
    v_cartesian_body_cmd = cfg.kp * err_vector + cfg.alpha * wrench

    J = robot.getJacobian()
    v_cmd = ik_solver(cfg, robot, v_cartesian_body_cmd)

    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(v_cmd[0]):
        breakFlag = True
        v_cmd = np.zeros(robot.nv)
    robot.sendVelocityCommand(v_cmd)

    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["wrench_used"] = wrench.reshape((6,))
    return breakFlag, save_past_dict, log_item


if __name__ == "__main__":
    cfg = getArgs()
    robot = getRobotFromConfig(cfg)
    robot._step()
    assert issubclass(robot.__class__, ForceTorqueOnSingleArmWrist)
    ik_solver = getIKSolver(cfg, robot)

    # NOTE: the weight, TCP and inertial matrix needs to be set on the robot
    # you already found an API in rtde_control for this, just put it in initialization
    # under using/not-using gripper parameters
    # NOTE: to use this you need to change the version inclusions in
    # ur_rtde due to a bug there in the current ur_rtde + robot firmware version
    # (the bug is it works with the firmware verion, but ur_rtde thinks it doesn't)
    # here you give what you're saving in the rolling past window
    # it's initial value.
    # controlLoopManager will populate the queue with these initial values
    save_past_dict = {
        "wrench": np.zeros(6),
    }
    # here you give it it's initial value
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "wrench_raw": np.zeros(6),
        "wrench_used": np.zeros(6),
    }
    q_init = robot.q
    T_w_einit = robot.T_w_e

    if not cfg.cartesian_space_impedance:
        controlLoop = partial(JointSpacePointImpedanceControlLoop, q_init, cfg, robot)
    else:
        controlLoop = partial(
            controlLoopCartesianPointImpedance, ik_solver, T_w_einit, cfg, robot
        )

    loop_manager = ControlLoopManager(
        robot, controlLoop, cfg, save_past_dict, log_item
    )
    loop_manager.run()

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
