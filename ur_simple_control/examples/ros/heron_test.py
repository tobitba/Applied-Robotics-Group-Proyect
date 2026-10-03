from smc import load_config
import rclpy
from rclpy.executors import MultiThreadedExecutor
from smc.robots.implementations.heron_real import get_cfg, RealHeronRobotManagerNode

import numpy as np
import argparse
from smc import load_config
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.cartesian_space import getClikArgs, getIKSolver
from smc.control.joint_space.joint_space_point_to_point import moveJP

import pinocchio as pin
import matplotlib
from functools import partial

import numpy as np


def backAndForthControlLoop(cfg, robot, i, past_data):
    """
    controlLoop
    -----------------------------
    controller description
    """
    breakFlag = False
    log_item = {}
    save_past_dict = {}

    T = 2.0
    t = i * robot.dt
    v_cmd = np.zeros(robot.nv)
    v_cmd[0] = 0.1 * np.sin(t / T)
    v_cmd[-1] = 0.3 * np.sin(t / T)

    robot.sendVelocityCommand(v_cmd)

    log_item["qs"] = robot.q.reshape((robot.nq,))
    log_item["vs"] = robot.v.reshape((robot.nv,))
    return breakFlag, save_past_dict, log_item


def backAndForthTest(cfg, robot, run=False) -> None | ControlLoopManager:
    robot._mode = robot.control_mode.base_only
    log_item = {}
    log_item["qs"] = np.zeros(
        robot.nq,
    )
    log_item["vs"] = np.zeros(
        robot.nv,
    )
    save_past_dict = {}

    loop = partial(backAndForthControlLoop, cfg, robot)
    loop_manager = ControlLoopManager(robot, loop, cfg, save_past_dict, log_item)

    if run:
        loop_manager.run()
    else:
        return loop_manager


def main(cfg=None):
    rclpy.init(cfg=cfg)
    # evil and makes cfg unusable but what can you do
    cfg_smc = get_cfg()
    assert cfg_smc.robot == "heron"
    cfg_smc.publish_commands = False
    cfg_smc.robot_ip = "192.168.0.4"
    # cfg_smc.publish_commands = True
    modes_and_loops = []
    robot = RealHeronRobotManagerNode(cfg_smc)
    robot._mode = robot.control_mode.whole_body
    robot._step()

    loop = backAndForthTest(cfg, robot, run=False)
    modes_and_loops.append((robot.control_mode.whole_body, loop))

    # NOTE: at this point you pass the modes_and_loops list
    robot.setModesAndLoops(modes_and_loops)

    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    executor.spin()


if __name__ == "__main__":
    main()
