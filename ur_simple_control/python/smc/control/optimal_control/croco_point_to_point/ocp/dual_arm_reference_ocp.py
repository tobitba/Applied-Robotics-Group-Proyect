from smc.control.optimal_control.oc_config import ConfigOptimalControl
from smc.control.optimal_control.abstract_croco_ocp import CrocoOCP
from smc.robots.interfaces.dual_arm_interface import DualArmInterface

import numpy as np
import pinocchio as pin
import crocoddyl


class DualArmIKOCP(CrocoOCP):
    def __init__(
        self,
        cfg: ConfigOptimalControl,
        robot: DualArmInterface,
        x0: np.ndarray,
        goal: pin.SE3,
    ):
        super().__init__(cfg, robot, x0, goal)
        raise NotImplementedError(
            "dual arm ocps are broken because the jacobian use by the solver is wrong"
        )

    def constructTaskCostsValues(self):
        self.ee_pose_cost_values = np.linspace(
            self.cfg.ee_pose_cost,
            self.cfg.ee_pose_cost * self.cfg.linearly_increasing_task_cost_factor,
            self.cfg.n_knots + 1,
        )

    def constructTaskObjectiveFunction(self, goal) -> None:
        T_w_lgoal, T_w_rgoal = goal
        framePlacementResidual_l = crocoddyl.ResidualModelFramePlacement(
            self.state,
            self.robot.l_ee_frame_id,
            T_w_lgoal.copy(),
            self.state.nv,
        )
        framePlacementResidual_r = crocoddyl.ResidualModelFramePlacement(
            self.state,
            self.robot.r_ee_frame_id,
            T_w_rgoal.copy(),
            self.state.nv,
        )
        goalTrackingCost_l = crocoddyl.CostModelResidual(
            self.state, framePlacementResidual_l
        )
        goalTrackingCost_r = crocoddyl.CostModelResidual(
            self.state, framePlacementResidual_r
        )
        # TODO: final velocity costs only make sense if you're running a single ocp, but not mpc!!
        # TODO: have an argument or something to include or not include them!

        if self.cfg.stop_at_final:
            frameVelocityResidual_l = crocoddyl.ResidualModelFrameVelocity(
                self.state,
                self.robot.l_ee_frame_id,
                pin.Motion(np.zeros(6)),
                pin.ReferenceFrame.WORLD,
            )
            frameVelocityCost_l = crocoddyl.CostModelResidual(
                self.state, frameVelocityResidual_l
            )
            frameVelocityResidual_r = crocoddyl.ResidualModelFrameVelocity(
                self.state,
                self.robot.r_ee_frame_id,
                pin.Motion(np.zeros(6)),
                pin.ReferenceFrame.WORLD,
            )
            frameVelocityCost_r = crocoddyl.CostModelResidual(
                self.state, frameVelocityResidual_r
            )
        for i in range(self.cfg.n_knots):
            self.runningCostModels[i].addCost(
                "l_ee_pose" + str(i),
                goalTrackingCost_l,
                self.ee_pose_cost_values[i],
            )
            self.runningCostModels[i].addCost(
                "r_ee_pose" + str(i),
                goalTrackingCost_r,
                self.ee_pose_cost_values[i],
            )
        self.terminalCostModel.addCost(
            "l_ee_pose" + str(self.cfg.n_knots),
            goalTrackingCost_l,
            self.ee_pose_cost_values[-1],
        )
        self.terminalCostModel.addCost(
            "r_ee_pose" + str(self.cfg.n_knots),
            goalTrackingCost_r,
            self.ee_pose_cost_values[-1],
        )
        if self.cfg.stop_at_final:
            self.terminalCostModel.addCost("velFinal_l", frameVelocityCost_l, 1e3)
            self.terminalCostModel.addCost("velFinal_r", frameVelocityCost_r, 1e3)

    # there is nothing to update in a point-to-point task
    def updateCosts(self, data):
        pass

    def updateGoalInModels(self, goal) -> None:
        T_w_lgoal, T_w_rgoal = goal
        for i, runningModel in enumerate(self.solver.problem.runningModels):
            runningModel.differential.costs.costs[
                "l_ee_pose" + str(i)
            ].cost.residual.reference = T_w_lgoal
            runningModel.differential.costs.costs[
                "r_ee_pose" + str(i)
            ].cost.residual.reference = T_w_rgoal

        self.solver.problem.terminalModel.differential.costs.costs[
            "l_ee_pose" + str(self.cfg.n_knots)
        ].cost.residual.reference = T_w_lgoal
        self.solver.problem.terminalModel.differential.costs.costs[
            "r_ee_pose" + str(self.cfg.n_knots)
        ].cost.residual.reference = T_w_rgoal
