from smc.bookkeeping.base_config import ConfigBase
from smc.robots.abstract_robotmanager import AbstractRobotManager
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.interfaces.single_arm_interface import SingleArmInterface

import pinocchio as pin
import numpy as np

# from copy import deepcopy

# TODO: put back in when python3.12 will be safe to use
# from typing import override


class SingleArmWholeBodyInterface(SingleArmInterface, MobileBaseInterface):
    """
    SingleArmWholeBodyInterface
    ---------------------------
    exists to provide either:
    1) whole body values
    2) base only values
    3) arm only values

    what you get depends on the mode you set - they're enumerate as above
    """

    def __init__(self, cfg: ConfigBase):
        if cfg.debug_prints:
            print("SingleArmWholeBodyInterface init")
        self._mode: AbstractRobotManager.control_mode
        self._available_modes: list[AbstractRobotManager.control_mode] = [
            AbstractRobotManager.control_mode.whole_body,
            AbstractRobotManager.control_mode.base_only,
            AbstractRobotManager.control_mode.upper_body,
        ]
        super().__init__(cfg)

    # TODO: override model property to produce the reduced version
    # depending on the control mode.
    # you might want to instantiate all of them in advance for easy switching later
    # NOTE: that this is currently only important for ocp construction,
    # even though it's obviously the correct move either way

    #    @AbstractRobotManager.mode.setter
    #    def mode(self, mode: AbstractRobotManager.control_mode) -> None:
    #        assert type(mode) in self._available_modes
    #        self._mode = mode

    # TODO: put back in when python3.12 will be safe to use
    #    @override
    @property
    def q(self) -> np.ndarray:
        if self._mode == self.control_mode.base_only:
            return self._q[:4]

        if self._mode == self.control_mode.upper_body:
            return self._q[4:]

        return self._q.copy()

    @property
    def nq(self):
        if self._mode == self.control_mode.base_only:
            return 4

        if self._mode == self.control_mode.upper_body:
            return self.model.nq - 4

        return self.model.nq

    @property
    def v(self) -> np.ndarray:
        if self._mode == self.control_mode.base_only:
            return self._v[:3]

        if self._mode == self.control_mode.upper_body:
            return self._v[3:]

        return self._v.copy()

    @property
    def nv(self):
        if self._mode == self.control_mode.base_only:
            return 3

        if self._mode == self.control_mode.upper_body:
            return self.model.nv - 3

        return self.model.nv

    @property
    def comfy_configuration(self):
        if self._mode == self.control_mode.base_only:
            return self._comfy_configuration[:4]

        if self._mode == self.control_mode.upper_body:
            return self._comfy_configuration[4:]

        return self._comfy_configuration

    # TODO: put back in when python3.12 will be safe to use
    #    @override
    @property
    def max_v(self) -> np.ndarray:
        if self._mode == self.control_mode.base_only:
            return self._max_v[:3]
        if self._mode == self.control_mode.upper_body:
            return self._max_v[3:]
        return self._max_v.copy()

    # NOTE: lil' bit of evil to access cartesian controllers just for the base without changing the controller
    @property
    def T_w_e(self):
        if self.mode == self.control_mode.upper_body:
            return self._T_w_e.copy()
        if self.mode == self.control_mode.base_only:
            return self._T_w_b.copy()
        return self._T_w_e.copy()

    # TODO: put back in when python3.12 will be safe to use
    #    @override
    def forwardKinematics(self) -> None:
        pin.forwardKinematics(
            self.model,
            self.data,
            self._q,
        )
        pin.updateFramePlacement(self.model, self.data, self._ee_frame_id)
        pin.updateFramePlacement(self.model, self.data, self._base_frame_id)
        self._T_w_e = self.data.oMf[self._ee_frame_id].copy()
        self._T_w_b = self.data.oMf[self._base_frame_id].copy()

    # TODO: put back in when python3.12 will be safe to use
    #    @override
    def getJacobian(self) -> np.ndarray:
        J_full = pin.computeFrameJacobian(
            self.model, self.data, self._q, self._ee_frame_id
        )
        if self._mode == self.control_mode.base_only:
            return J_full[:, :3]

        if self._mode == self.control_mode.upper_body:
            return J_full[:, 3:]

        return J_full

    # TODO: put back in when python3.12 will be safe to use
    #    @override
    def sendVelocityCommand(self, v) -> None:
        """
        sendVelocityCommand
        -------------------
        1) saturate the command to comply with hardware limits or smaller limits you've set
        2) send it via the particular robot's particular communication interface
        """
        assert type(v) == np.ndarray

        if self._mode == self.control_mode.base_only:
            v = np.hstack((v, np.zeros(self.model.nv - 3)))

        if self._mode == self.control_mode.upper_body:
            v = np.hstack((np.zeros(3), v))

        assert len(v) == self.model.nv
        v = np.clip(v, -1 * self._max_v, self._max_v)
        self.sendVelocityCommandToReal(v)

    # TODO: put back in when python3.12 will be safe to use
    #    @override
    # TODO: we need to have presaved truncated models per mode
    # to do this computation as expected.
    # now it's just a hack to remove the base
    def computeManipulabilityIndexQDerivative(self) -> np.ndarray:
        joint_index = self.model.frames[self._ee_frame_id].parentJoint
        J = pin.computeJointJacobian(self.model, self.data, self._q, joint_index)
        Jp = J.T @ np.linalg.inv(
            J @ J.T + np.eye(J.shape[0], J.shape[0]) * self.cfg.tikhonov_damp
        )
        # res = np.zeros(self.nv)
        # v0 = np.zeros(self.nv)
        res = np.zeros(self.model.nv)
        v0 = np.zeros(self.model.nv)
        for k in range(6):
            pin.computeForwardKinematicsDerivatives(
                self.model,
                self.data,
                self._q,
                Jp[:, k],
                v0,
                # self.model,
                # self.data,
                # self._q,
                # v0,
                # np.zeros(self.model.nv),
            )
            JqJpk = pin.getJointVelocityDerivatives(
                self.model, self.data, joint_index, pin.LOCAL
            )[0]
            res += JqJpk[k, :]
        res *= self.computeManipulabilityIndex()
        if self.mode == self.control_mode.whole_body:
            return res
        if self.mode == self.control_mode.upper_body:
            return res[3:]
