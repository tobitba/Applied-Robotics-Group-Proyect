from smc.control.optimal_control.oc_config import ConfigOptimalControl
from smc.control.optimal_control.abstract_croco_ocp import CrocoOCP
from smc.robots.interfaces.single_arm_interface import SingleArmInterface

import numpy as np
from pinocchio import SE3
import crocoddyl


class SingleArmIKOCP(CrocoOCP):
    def __init__(
        self,
        cfg: ConfigOptimalControl,
        robot: SingleArmInterface,
        x0: np.ndarray,
        T_w_goal: SE3,
    ):
        super().__init__(cfg, robot, x0, T_w_goal)

    def constructTaskCostsValues(self):
        self.ee_pose_cost_values = np.linspace(
            self.cfg.ee_pose_cost,
            self.cfg.ee_pose_cost * self.cfg.linearly_increasing_task_cost_factor,
            self.cfg.n_knots + 1,
        )

    def constructTaskObjectiveFunction(self, goal) -> None:
        T_w_goal = goal
        framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
            self.state,
            self.robot.ee_frame_id,
            T_w_goal.copy(),
            self.state.nv,
        )
        goalTrackingCost = crocoddyl.CostModelResidual(
            self.state, framePlacementResidual
        )
        # TODO: final velocity costs only make sense if you're running a single ocp, but not mpc!!
        # TODO: have an argument or something to include or not include them!
        # frameVelocityResidual = crocoddyl.ResidualModelFrameVelocity(
        #    self.state,
        #    self.robot.ee_frame_id,
        #    pin.Motion(np.zeros(6)),
        #    pin.ReferenceFrame.WORLD,
        # )
        # frameVelocityCost = crocoddyl.CostModelResidual(
        #    self.state, frameVelocityResidual
        # )
        for i in range(self.cfg.n_knots):
            self.runningCostModels[i].addCost(
                "ee_pose" + str(i), goalTrackingCost, self.ee_pose_cost_values[i]
            )
        self.terminalCostModel.addCost(
            "ee_pose" + str(self.cfg.n_knots),
            goalTrackingCost,
            self.ee_pose_cost_values[-1],
        )
        # self.terminalCostModel.addCost("velFinal", frameVelocityCost, 1e3)

    # there is nothing to update in a point-to-point task
    def updateCosts(self, data):
        pass

    def updateGoalInModels(self, goal) -> None:
        T_w_goal = goal
        for i, runningModel in enumerate(self.solver.problem.runningModels):
            runningModel.differential.costs.costs[
                "ee_pose" + str(i)
            ].cost.residual.reference = T_w_goal
        self.solver.problem.terminalModel.differential.costs.costs[
            "ee_pose" + str(self.cfg.n_knots)
        ].cost.residual.reference = T_w_goal
