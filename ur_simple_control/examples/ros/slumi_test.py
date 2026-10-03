from smc import load_config
import numpy as np
import argparse
from smc import load_config, getRobotFromConfig
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.cartesian_space import getClikArgs, getIKSolver
from smc.control.joint_space.joint_space_point_to_point import moveJP

# from smc.robots.implementations.slumi.slumi_real import getSLuMiConnectionArgs

import pinocchio as pin
import matplotlib
from functools import partial


def get_cfg():
    parser = load_config()
    #    parser = getSLuMiConnectionArgs(parser)
    cfg = parser.parse_cfg()
    return cfg


def backAndForthControlLoop(cfg, robot, i, past_data):
    """
    controlLoop
    -----------------------------
    controller description
    """
    breakFlag = False
    log_item = {}
    save_past_dict = {}

    T = 1.0
    t = i * robot.dt
    v_cmd = np.zeros(robot.nv)
    index = i // 1500
    index = min(index, robot.nv - 1)
    v_cmd[index] = 0.2 * np.sin(t / T)
    # v_cmd[-1] = 0.3 * np.sin(t / T)

    robot.sendVelocityCommand(v_cmd)

    log_item["qs"] = robot.q.reshape((robot.nq,))
    log_item["vs"] = robot.v.reshape((robot.nv,))
    log_item["v_cmd"] = v_cmd.reshape((robot.nv,))
    return breakFlag, save_past_dict, log_item


def backAndForthTest(cfg, robot, run=False) -> None | ControlLoopManager:
    log_item = {}
    log_item["qs"] = np.zeros(
        robot.nq,
    )
    log_item["vs"] = np.zeros(
        robot.nv,
    )
    log_item["v_cmd"] = np.zeros(
        robot.nv,
    )
    save_past_dict = {}

    loop = partial(backAndForthControlLoop, cfg, robot)
    loop_manager = ControlLoopManager(robot, loop, cfg, save_past_dict, log_item)

    if run:
        loop_manager.run()
    else:
        return loop_manager


def main():
    cfg = get_cfg()
    print(cfg)
    assert cfg.robot == "slumi"
    robot = getRobotFromConfig(cfg)
    robot._mode = robot.control_mode.whole_body
    robot._step()

    backAndForthTest(cfg, robot, run=True)


if __name__ == "__main__":
    main()
