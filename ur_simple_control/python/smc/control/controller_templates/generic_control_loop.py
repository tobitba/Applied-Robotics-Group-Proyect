from smc.bookkeeping.load_config import GlobalConfig
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.control.control_typing import ControlLoopReturn, ControlLoop
from smc.bookkeeping.types import PastData
from typing import Any


def GenericControlLoopTemplate(
    X: Any,
    control_loop: ControlLoop,
    cfg: GlobalConfig,
    robot: AbstractRobotManager,
    t: int,  # will be float eventually
    past_data: PastData,
) -> ControlLoopReturn:
    breakFlag = False
    log_item = {}
    save_past_item = {}

    v_cmd, past_item_inner, log_item_inner = control_loop(X, cfg, robot, t, past_data)

    robot.sendVelocityCommand(v_cmd)

    log_item.update(log_item_inner)
    save_past_item.update(past_item_inner)
    return breakFlag, save_past_item, log_item
