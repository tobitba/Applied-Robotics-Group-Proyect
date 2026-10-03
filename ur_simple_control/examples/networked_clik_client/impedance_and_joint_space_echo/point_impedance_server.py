from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.types import PastData
from smc.control import ControlLoopManager
from smc.control.cartesian_space import (
    getIKSolver,
)

from smc import load_config, getRobotFromConfig
from smc.control.cartesian_space.cartesian_space_config import IKSolver
from smc.multiprocessing.consumer_process import ConsumerProcess
from smc.multiprocessing.networking.server import server_sender

import pinocchio as pin
import numpy as np
import copy
from functools import partial

from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)


# feedforward velocity, feedback position and force for impedance
def controller():
    pass


def controlLoopPointImpedanceServer(
    cfg: GlobalConfig,
    q_init: np.ndarray,
    sender: ConsumerProcess,
    ik_solver: IKSolver,
    robot: ForceTorqueOnSingleArmWrist,
    i: int,
    past_data: PastData,
):
    breakFlag = False
    save_past_dict = {}
    log_item = {}
    q = robot.q
    wrench = robot.wrench
    log_item["wrench_raw"] = wrench.reshape((6,))
    # deepcopy for good coding practise (and correctness here)
    save_past_dict["wrench"] = copy.deepcopy(wrench)
    # rolling average
    # wrench = np.average(np.array(past_data['wrench']), axis=0)
    # first-order low pass filtering instead
    # beta is a smoothing coefficient, smaller values smooth more, has to be in [0,1]
    # wrench = cfg.beta * wrench + (1 - cfg.beta) * past_data['wrench'][-1]
    wrench = cfg.compliance.beta * wrench + (1 - cfg.compliance.beta) * np.average(
        np.array(past_data["wrench"]), axis=0
    )
    if not cfg.compliance.z_only:
        Z = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    else:
        Z = np.diag(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))

    wrench = Z @ wrench

    J = robot.getJacobian()
    tau = J.T @ wrench
    tau = tau[:6].reshape((6, 1))
    # compute control law:
    # - feedback the position
    # kv is not needed if we're running velocity control
    vel_cmd = (
        cfg.compliance.Kp * (q_init[:6].reshape((6, 1)) - q[:6].reshape((6, 1)))
        + cfg.compliance.alpha * tau
    )
    # vel_cmd = np.zeros(6)
    robot.sendVelocityCommand(vel_cmd)
    sender.sendCommand({"q": q})

    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(vel_cmd[0]):
        breakFlag = True

    # log what you said you'd log
    # TODO fix the q6 situation (hide this)
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["wrench_used"] = wrench.reshape((6,))

    return breakFlag, save_past_dict, log_item


def controlLoopCartesianPointImpedanceServer(
    cfg: GlobalConfig,
    T_w_e_init: pin.SE3,
    sender: ConsumerProcess,
    ik_solver: IKSolver,
    robot: ForceTorqueOnSingleArmWrist,
    i: int,
    past_data,
):
    breakFlag = False
    # TODO rename this into something less confusing
    save_past_dict = {}
    log_item = {}
    q = robot.q
    Mtool = robot.T_w_e
    wrench = robot.wrench
    log_item["wrench_raw"] = wrench.reshape((6,))
    save_past_dict["wrench"] = copy.deepcopy(wrench)
    # wrench = cfg.beta * wrench + (1 - cfg.beta) * past_data['wrench'][-1]
    wrench = cfg.compliance.beta * wrench + (1 - cfg.compliance.beta) * np.average(
        np.array(past_data["wrench"]), axis=0
    )
    # good generic values
    # Z = np.diag(np.array([1.0, 1.0, 2.0, 1.0, 1.0, 1.0]))
    # but let's stick to the default for now
    if not cfg.compliance.z_only:
        Z = np.diag(np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]))
    else:
        Z = np.diag(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    # Z = np.diag(np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]))
    # Z = np.diag(np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))

    wrench = Z @ wrench

    J = robot.getJacobian()

    SEerror = Mtool.actInv(T_w_e_init)
    err_vector = pin.log6(SEerror).vector
    v_cartesian_body_cmd = cfg.kp * err_vector + cfg.alpha * wrench

    vel_cmd = ik_solver(cfg.cs, robot, v_cartesian_body_cmd)
    robot.sendVelocityCommand(vel_cmd)
    sender.sendCommand({"q": q})

    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(vel_cmd[0]):
        breakFlag = True

    # log what you said you'd log
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["wrench_used"] = wrench.reshape((6,))

    return breakFlag, save_past_dict, log_item


if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    robot._step()
    clikController = getIKSolver(cfg, robot)

    # TODO: and NOTE the weight, TCP and inertial matrix needs to be set on the robot
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
        "vs": np.zeros(robot.nq),
        "wrench_raw": np.zeros(6),
        "wrench_used": np.zeros(6),
    }
    q_init = robot.q
    Mtool_init = robot.T_w_e
    side_sender = partial(server_sender, cfg.net.host, cfg.net.port)
    sender = ConsumerProcess(cfg, side_sender, {"q": np.zeros(6)})

    if not cfg.cartesian_space_impedance:
        controlLoop = partial(
            controlLoopPointImpedanceServer, cfg, q_init, sender, controller, robot
        )
    else:
        controlLoop = partial(
            controlLoopCartesianPointImpedanceServer,
            cfg,
            Mtool_init,
            sender,
            clikController,
            robot,
        )

    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    loop_manager.run()

    if not cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot.log_manager.saveLog()
        robot.log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
