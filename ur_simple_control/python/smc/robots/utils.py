from smc.bookkeeping.load_config import GlobalConfig
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.robots.implementations import *
from smc.robots.implementations.ur5e import RealUR5eRobotManager


import numpy as np
from pinocchio import rpy
import argparse


# TODO: get the visual thing you did in ivc project with sliders also.
# it's just text input for now because it's totally usable, just not superb.
# but also you do want to have both options. obviously you go for the sliders
# in the case you're visualizing, makes no sense otherwise.
def defineGoalPointCLI(robot):
    """
    defineGoalPointCLI
    ------------------
    get a nice TUI-type prompt to put in a frame goal for p2p motion.
    --> best way to handle the goal is to tell the user where the gripper is
        in both UR tcp frame and with pinocchio and have them
        manually input it when running.
        this way you force the thinking before the moving,
        but you also get to view and analyze the information first
    """
    robot._step()
    robot._step()
    # define goal
    T_w_goal = robot.T_w_e
    print("current T_w_e:", robot.T_w_e)
    print("You can only specify the translation right now.")
    if robot.cfg.real:
        print(
            "In the following, first 3 numbers are x,y,z position, and second 3 are r,p,y angles"
        )
        print(
            "Here's where the robot is currently. Ensure you know what the base frame is first."
        )
        print(
            "base frame end-effector pose from pinocchio:\n",
            *robot.T_w_e.translation.round(4),
            *rpy.matrixToRpy(robot.T_w_e.rotation).round(4),
        )
    # remain with the current orientation
    # TODO: add something, probably rpy for orientation because it's the least number
    # of numbers you need to type in
    # this is a reasonable way to do it too, maybe implement it later
    # T_w_goal.translation = T_w_goal.translation + np.array([0.0, 0.0, -0.1])
    # do a while loop until this is parsed correctly
    while True:
        goal = input(
            "Please enter the target end-effector position in the x.x,y.y,z.z format: "
        )
        try:
            e = "ok"
            goal_list = goal.split(",")
            for i in range(len(goal_list)):
                goal_list[i] = float(goal_list[i])
        except:
            e = "incorrent"
            print("The input is not in the expected format. Try again.")
        if e == "ok":
            T_w_goal.translation = np.array(goal_list)
            break
    print("this is goal pose you defined:\n", T_w_goal)

    if robot.cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": T_w_goal})
    return T_w_goal


# TODO: finish
def getRobotFromConfig(cfg: GlobalConfig) -> AbstractRobotManager:
    if cfg.robot == "ur5e":
        if cfg.real:
            return RealUR5eRobotManager(cfg)
        else:
            return SimulatedUR5eRobotManager(cfg)
    if cfg.robot == "heron":
        if cfg.real:
            pass
            # TODO: finish it
            # return RealHeronRobotManager(cfg)
        else:
            return SimulatedHeronRobotManager(cfg, True)
    if cfg.robot == "heronfullyactuatedbase":
        if cfg.real:
            raise BaseException(
                "there is no heron with fully actuated base, this is just for testing"
            )
        else:
            return SimulatedHeronRobotManager(cfg, False)
    if cfg.robot == "yumi":
        if cfg.real:
            raise NotImplementedError
            # TODO: finish it
            # return RealYuMiRobotManager(cfg)
        else:
            return SimulatedYuMiRobotManager(cfg)
    if cfg.robot == "myumi":
        if cfg.real:
            return RealMobileYuMiRobotManager(cfg)
        else:
            return SimulatedMobileYuMiRobotManager(cfg)
    if cfg.robot == "mir":
        if cfg.real:
            # TODO: finish it
            # return RealMirRobotManager(cfg)
            raise NotImplementedError(
                "have to extract mir parts from heron to make this work, soz"
            )
        else:
            return SimulatedMirRobotManager(cfg, True)
    if cfg.robot == "mirfullyactuated":
        if cfg.real:
            raise BaseException(
                "real mir is underactuated (diff drive), so this is illegal!"
            )
        else:
            return SimulatedMirRobotManager(cfg, False)
    if cfg.robot == "slumi":
        if cfg.real:
            return RealSLuMiRobotManager(cfg)
        else:
            return SimulatedSLuMiRobotManager(cfg)
    raise NotImplementedError(
        f"robot {cfg.robot} is not supported! run the script you ran with --help to see what's available"
    )
