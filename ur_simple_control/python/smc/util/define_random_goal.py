from smc.bookkeeping.base_config import ConfigBase
import numpy as np
from pinocchio import SE3
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)


def getRandomlyGeneratedGoal(cfg: ConfigBase, robot):
    T_w_goal = SE3.Random()
    # has to be close
    translation = np.random.random(3) * 0.8 - 0.4
    translation[2] = np.abs(translation[2])
    if issubclass(robot.__class__, SingleArmWholeBodyInterface) or issubclass(
        robot.__class__, DualArmWholeBodyInterface
    ):
        translation[0] = np.random.random() * 5 - 2.5
        translation[1] = np.random.random() * 5 - 2.5
        translation[2] += 0.75
    translation = translation + np.ones(3) * 0.1
    T_w_goal.translation = translation
    if cfg.debug_prints:
        print(T_w_goal)
    return T_w_goal
