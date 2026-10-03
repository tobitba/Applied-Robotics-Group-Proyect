from smc.control.optimal_control.oc_config import ConfigOptimalControl
from smc.control.optimal_control.croco_point_to_point.ocp.single_arm_reference_ocp import (
    SingleArmIKOCP,
)
from smc.robots.interfaces.single_arm_interface import SingleArmInterface

import numpy as np
from argparse import Namespace


class CrocoEEPathFollowingOCP(SingleArmIKOCP):
    """
    createCrocoEEPathFollowingOCP
    -------------------------------
    creates a path following problem with a single end-effector reference.
    it is instantiated to just to stay at the current position.
    NOTE: the path MUST be time indexed with the SAME time used between the knots
    """

    def __init__(
        self, cfg: ConfigOptimalControl, robot: SingleArmInterface, x0: np.ndarray
    ):
        goal = robot.T_w_e
        super().__init__(cfg, robot, x0, goal)

    raise NotImplementedError
    # TODO: first of all the mpc should optimize over the path parameter
    # and not points.
    # but if you do want to do it with points,
    # they should come from interpolation.
    # and this needs to be a function in motion planner.
    # since i don't need this now, i'll just leave it broken
    # until i get around to fixing optimal control overall

    def updateCosts(self, data):
        for i, runningModel in enumerate(self.solver.problem.runningModels):
            runningModel.differential.costs.costs[
                "ee_pose" + str(i)
            ].cost.residual.reference = data[i]

        self.solver.problem.terminalModel.differential.costs.costs[
            "ee_pose" + str(self.cfg.n_knots)
        ].cost.residual.reference = data[-1]
