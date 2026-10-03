from smc import load_config
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.control.dmp import followDMP

from argparse import Namespace
import pickle
import numpy as np
import time


def getMarker(cfg: MasterConfig, robot: SingleArmInterface) -> None:
    """
    getMarker
    ---------
    get marker from user in a blind handover
    RUN THIS IN SIM FIRST TO SEE IF IT MAKES SENSE.
    if not, generate a different joint trajectory for your situation.
    """
    # load traj
    try:
        file = open("./data/from_writing_to_handover.pickle_save", "rb")
    except FileNotFoundError:
        print(
            "there is not file on path './data/from_writing_to_handover.pickle_save'!"
        )
        print("(1) check from which directory you're running the script")
        print(
            "(2) if you don't wave a from_writing_to_handover.pickle_save, you can create it by teaching by demostration available within freedrive_v2.0 found in convenience_tool_box (details on it are there)"
        )
        exit()
    point_dict = pickle.load(file)
    file.close()
    #####################
    #  go toward user   #
    #####################
    # this is more than enough, and it will be the same for both
    tau0 = 5
    # setting up array for dmp
    # TODO: make the dmp general already
    for i in range(len(point_dict["qs"])):
        point_dict["qs"][i] = point_dict["qs"][i][:6]
    qs = np.array(point_dict["qs"])

    followDMP(cfg, robot, qs, tau0)
    robot.sendVelocityCommand(np.zeros(robot.nv))

    ##########################################
    #  blind handover (open/close on timer)  #
    ##########################################
    robot.openGripper()
    time.sleep(5)
    robot.closeGripper()
    time.sleep(3)

    #############
    #  go back  #
    #############
    point_dict["qs"].reverse()
    # setting up array for dmp
    qs = np.array(point_dict["qs"])
    followDMP(cfg, robot, qs, tau0)
