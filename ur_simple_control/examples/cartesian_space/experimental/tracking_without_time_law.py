from smc.bookkeeping.load_config import GlobalConfig
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.control.cartesian_space.ik_solvers import dampedPseudoinverse
import numpy as np


# NOTE: as far as i'm concerned this does not work.
# as in we do need a sensible timing law
# TODO: this won't work for diff drive or other underactuated!!
# it also doesn't work in general lmao
def parameterTrackingHackTimeLaw(
    cfg: GlobalConfig, robot: AbstractRobotManager, V_cmd: np.ndarray, v_cmd: np.ndarray
) -> float:
    v_cmd_to_real = np.clip(v_cmd, -1 * robot._max_v, robot._max_v)

    # how much faster is the real v_cmd compared to the one needed
    # to follow this path?
    # --> we update s with this factor, call it k.
    # we use an ik solver which DOES NOT respect velocity contraints
    # to get this estimate
    v_cmd_uncostrained = dampedPseudoinverse(cfg.cs, robot, V_cmd)
    k = min(1.0, np.min(np.abs(v_cmd_to_real / v_cmd_uncostrained)))
    s_update = robot.dt * k
    return s_update
