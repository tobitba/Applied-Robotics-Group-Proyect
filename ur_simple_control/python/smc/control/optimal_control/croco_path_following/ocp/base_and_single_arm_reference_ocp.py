from smc.control.optimal_control.oc_config import ConfigOptimalControl
from smc.control.optimal_control.croco_point_to_point.ocp.base_and_single_arm_reference_ocp import (
    BaseAndSingleArmIKOCP,
)
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)

from argparse import Namespace


class BaseAndEEPathFollowingOCP(BaseAndSingleArmIKOCP):
    """
    createBaseAndEEPathFollowingOCP
    -------------------------------
    creates a path following problem.
    it is instantiated to just to stay at the current position.
    NOTE: the path MUST be time indexed with the SAME time used between the knots
    """

    def __init__(
        self, cfg: ConfigOptimalControl, robot: SingleArmWholeBodyInterface, x0
    ):
        goal = (robot.T_w_e, robot.T_w_b.translation.copy())
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
        path_base = data[0]
        pathSE3 = data[1]
        for i, runningModel in enumerate(self.solver.problem.runningModels):
            runningModel.differential.costs.costs[
                "base_translation" + str(i)
            ].cost.residual.reference = path_base[i]
            runningModel.differential.costs.costs[
                "ee_pose" + str(i)
            ].cost.residual.reference = pathSE3[i]

        # idk if that's necessary
        self.solver.problem.terminalModel.differential.costs.costs[
            "base_translation" + str(self.cfg.n_knots)
        ].cost.residual.reference = path_base[-1]
        self.solver.problem.terminalModel.differential.costs.costs[
            "ee_pose" + str(self.cfg.n_knots)
        ].cost.residual.reference = pathSE3[-1]
