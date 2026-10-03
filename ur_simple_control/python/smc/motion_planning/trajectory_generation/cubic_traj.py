import numpy as np
from smc.motion_planning.trajectory_generation.trajectory_generator import (
    AbstractTrajectoryGenerator,
)
from smc.bookkeeping.types import (
    JointPositions,
    JointVelocities,
    JointAccelerations,
    JointReference,
)


class CubicJointTrajectory(AbstractTrajectoryGenerator):
    def __init__(self, q_0: np.ndarray, q_goal: np.ndarray, T_final: float, t: float):
        self.q_0 = q_0
        self.q_goal = q_goal
        self.startEndDifference = q_goal - q_0
        self.T_final = T_final
        self.T_final_pow2 = T_final**2
        self.T_final_pow3 = T_final**3

    def computeInterpolation(self):
        pass

    def getPoint(self, t: float) -> JointPositions:
        return (
            self.q_0
            + (((3 * t**2) / self.T_final_pow2) - (2 * t**3) / self.T_final_pow3)
            * self.startEndDifference
        )

    def getVelocity(self, t: float) -> JointVelocities:
        return (
            (6 * t) / self.T_final_pow2 - (6 * t**2) / self.T_final_pow3
        ) * self.startEndDifference

    def getAcceleration(self, t: float) -> JointAccelerations:
        return (
            6 / self.T_final_pow2 - (12 * t) / self.T_final_pow3
        ) * self.startEndDifference

    def getReference(self, t: float) -> JointReference:
        return self.getPoint(t), self.getVelocity(t), None


# TODO: quintic


# NOTE: this is not what i want.
# i just want the time derivative of a cubic
# to get the prescribed velocity from 0 to 0.
# i.e. i want s_dot, provided s.
# but it doesn't give me T_final which is the biggeest problem

# def cubicSE3Trajectory(
#    T_w_start: np.ndarray, T_w_goal: np.ndarray, s: float
# ) -> tuple[np.ndarray, np.ndarray]:
#
#    s_final = 1.0
#
#    T_w_s = q_0 + (((3 * s**2) / s_final**2) - (2 * s**3) / s_final**3) * (q_goal - q_0)
#    V_s = ((6 * s) / s_final**2 - (6 * s**2) / s_final**3) * (q_goal - q_0)
#
#    return T_w_s, V_s
