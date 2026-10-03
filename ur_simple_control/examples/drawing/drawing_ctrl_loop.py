from smc import load_config
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.control.dmp.dmp import (
    DMP,
    NoTC,
    TCVelAccConstrained,
)
from smc.control.cartesian_space import (
    moveL,
    compliantMoveL,
)
from smc.control import ControlLoopManager

import pinocchio as pin
import numpy as np
from argparse import Namespace
from functools import partial
from collections import deque


def controlLoopWriting(
    cfg: MasterConfig,
    robot: ForceTorqueOnSingleArmWrist,
    dmp: DMP,
    tc: NoTC | TCVelAccConstrained,
    i: int,
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    controlLoopWriting
    -----------------------
    dmp reference on joint path + compliance
    """
    breakFlag = False
    save_past_dict = {}
    log_item = {}
    dmp.step(robot.dt)
    # temporal coupling step
    tau_dmp = dmp.tau + tc.update(dmp, robot.dt) * robot.dt
    dmp.set_tau(tau_dmp)
    q = robot.q
    Z = np.diag(np.array([0.0, 0.0, 1.0, 0.5, 0.5, 0.0]))

    wrench = robot.wrench
    save_past_dict["wrench"] = wrench.copy()
    # rolling average
    # wrench = np.average(np.array(past_data['wrench']), axis=0)

    # first-order low pass filtering instead
    # beta is a smoothing coefficient, smaller values smooth more, has to be in [0,1]
    # wrench = cfg.beta * wrench + (1 - cfg.beta) * past_data['wrench'][-1]
    wrench = cfg.beta * wrench + (1 - cfg.beta) * np.average(
        np.array(past_data["wrench"]), axis=0
    )

    wrench = Z @ wrench
    J = robot.getJacobian()
    # get joint
    tau = J.T @ wrench
    tau = tau.reshape((-1, 1))
    # compute control law:
    # - feedforward the velocity and the force reading
    # - feedback the position
    v_cmd = dmp.vel + cfg.kp * (dmp.pos - q.reshape((-1, 1))) + cfg.alpha * tau
    v_cmd = v_cmd.reshape((-1,))

    # tau0 is the minimum time needed for dmp
    # 500 is the frequency
    # so we need tau0 * 500 iterations minimum
    if (np.linalg.norm(dmp.vel) < 0.01) and (i > int(cfg.tau0 * 500)):
        breakFlag = True
        v_cmd = np.zeros(robot.nv)

    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(v_cmd[0]):
        print("GO NAN FROM INTO VEL_CMD!!! EXITING!!")
        breakFlag = True
        v_cmd = np.zeros(robot.nv)

    robot.sendVelocityCommand(v_cmd)
    log_item["qs"] = robot.q
    log_item["dmp_qs"] = dmp.pos.reshape((6,))
    log_item["vs"] = robot.v
    log_item["dmp_vs"] = dmp.vel.reshape((6,))
    log_item["wrench"] = wrench.reshape((6,))
    log_item["tau"] = tau.reshape((6,))

    return breakFlag, save_past_dict, log_item


def write(
    joint_trajectory: np.ndarray,
    cfg: MasterConfig,
    robot: ForceTorqueOnSingleArmWrist,
) -> None:
    dmp = DMP(joint_trajectory, a_s=1.0)
    if not cfg.temporal_coupling:
        tc = NoTC()
    else:
        v_max_ndarray = np.ones(robot.nq) * robot._max_v
        a_max_ndarray = np.ones(robot.nq) * cfg.acceleration
        tc = TCVelAccConstrained(
            cfg.gamma_nominal, cfg.gamma_a, v_max_ndarray, a_max_ndarray, cfg.eps_tc
        )

    print("going to starting write position")
    dmp.step(1 / 500)
    first_q = dmp.pos.reshape((6,))
    # move to initial pose
    T_w_goal = robot.computeT_w_e(first_q)
    # start a bit above
    go_away_from_plane_transf = pin.SE3.Identity()
    go_away_from_plane_transf.translation[2] = -1 * cfg.mm_into_board
    T_w_goal = T_w_goal.act(go_away_from_plane_transf)
    if not cfg.board_wiping:
        compliantMoveL(T_w_goal, cfg, robot)
    else:
        moveL(cfg, robot, T_w_goal)

    save_past_dict = {
        "wrench": np.zeros(6),
    }
    # here you give it it's initial value
    log_item = {
        "qs": np.zeros(robot.nq),
        "dmp_qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "dmp_vs": np.zeros(robot.nq),
        "wrench": np.zeros(6),
        "tau": np.zeros(robot.nv),
    }
    # moveJ(cfg, robot, dmp.pos.reshape((6,)))
    controlLoop = partial(controlLoopWriting, cfg, robot, dmp, tc)
    loop_manager = ControlLoopManager(
        robot, controlLoop, cfg, save_past_dict, log_item
    )
    # and now we can actually run
    loop_manager.run()

    print("move a bit back")
    T_w_e = robot.T_w_e
    go_away_from_plane_transf = pin.SE3.Identity()
    go_away_from_plane_transf.translation[2] = -0.1
    goal = T_w_e.act(go_away_from_plane_transf)
    compliantMoveL(goal, cfg, robot)
