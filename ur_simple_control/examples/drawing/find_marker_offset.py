from smc import load_config
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.control.cartesian_space import (
    moveUntilContact,
    compliantMoveL,
)

import pinocchio as pin
import numpy as np
from argparse import Namespace


def findMarkerOffset(
    plane_pose: pin.SE3, cfg: MasterConfig, robot: ForceTorqueOnSingleArmWrist
) -> float:
    """
    findMarkerOffset
    ---------------
    This relies on having the correct orientation of the plane
    and the correct translation vector for top-left corner.
    Idea is you pick up the marker, go to the top corner,
    touch it, and see the difference between that and the translation vector.
    Obviously it's just a hacked solution, but it works so who cares.
    """
    above_starting_write_point = pin.SE3.Identity()
    above_starting_write_point.translation[2] = -0.2
    above_starting_write_point = plane_pose.act(above_starting_write_point)
    print("going to above plane pose point", above_starting_write_point)
    compliantMoveL(above_starting_write_point, cfg, robot)

    # this is in the end-effector frame, so this means going straight down
    # because we are using the body jacobians in our clik
    speed = np.zeros(6)
    speed[2] = 0.02
    moveUntilContact(cfg, robot, speed)
    # we use the pin coordinate system because that's what's
    # the correct thing long term accross different robots etc
    current_translation = robot.T_w_e.translation
    # i only care about the z because i'm fixing the path atm
    # but, let's account for the possible milimiter offset 'cos why not
    marker_offset = np.linalg.norm(plane_pose.translation - current_translation)

    print("going back")
    compliantMoveL(above_starting_write_point, cfg, robot)
    return marker_offset
