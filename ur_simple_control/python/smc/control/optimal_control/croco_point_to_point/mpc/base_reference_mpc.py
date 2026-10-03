from smc.control.optimal_control.oc_config import ConfigOptimalControl
from smc.control.controller_templates.point_to_point import (
    BaseP2PCtrlLoopTemplate,
)
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.control.optimal_control.abstract_croco_ocp import CrocoOCP
from smc.control.optimal_control.croco_point_to_point.ocp.base_reference_ocp import (
    BaseIKOCP,
)
from smc.control.control_loop_manager import ControlLoopManager

import numpy as np
from functools import partial
from collections import deque


def CrocoBaseP2PMPCControlLoop(
    ocp: CrocoOCP,
    p_basegoal: np.ndarray,
    cfg: ConfigOptimalControl,
    robot: MobileBaseInterface,
    _: int,
    __: dict[str, deque[np.ndarray]],
) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    CrocoIKMPCControlLoop
    ---------------------
    """
    # set initial state from sensor
    x0 = np.concatenate([robot.q, robot.v])
    # embed()
    ocp.warmstartAndReSolve(x0)
    xs = np.array(ocp.solver.xs)
    # NOTE: for some reason the first value is always some wild bs
    v_cmd = xs[1, robot.model.nq :]
    # NOTE: dirty hack to work if wholebody mode, don't ask
    v_cmd[3:] = 0.0
    return v_cmd, {}, {}


def CrocoBaseP2PMPC(
    cfg: ConfigOptimalControl,
    robot: MobileBaseInterface,
    p_basegoal: np.ndarray,
    run=True,
):
    """
    CrocoBaseP2PMPC
    -----
    base point-to-point task
    """
    x0 = np.concatenate([robot.q, robot.v])
    ocp = BaseIKOCP(cfg, robot, x0, p_basegoal)
    ocp.solveInitialOCP(x0)

    controlLoop = partial(
        BaseP2PCtrlLoopTemplate,
        ocp,
        p_basegoal,
        CrocoBaseP2PMPCControlLoop,
        cfg,
        robot,
    )
    log_item = {
        "qs": np.zeros(robot.nq),
        "base_err_norm": np.zeros(1),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
    }
    save_past_dict = {}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager
