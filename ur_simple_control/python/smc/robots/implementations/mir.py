from smc.bookkeeping.base_config import ConfigBase
from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.abstract_simulated_robotmanager import AbstractSimulatedRobotManager
from smc.robots.abstract_robotmanager import (
    AbstractRobotManager,
)
from smc.control.control_loop_manager import ControlLoopManager


import pinocchio as pin
import coal
import numpy as np


class AbstractMirRobotManager(MobileBaseInterface):
    def __init__(self, cfg: ConfigBase, diff_drive: bool):
        if cfg.debug_prints:
            print("AbstractMirRobotManager init")
        self._model, self._collision_model, self._visual_model, self._data = (
            mir_approximation(diff_drive)
        )
        self._base_frame_id = self.model.getFrameId("mobile_base")
        self._MAX_ACCELERATION = 1.7  # const
        self._MAX_QD = 3.14  # const
        self._q = np.zeros(self.model.nq)
        self._q[2] = 1.0
        super().__init__(cfg)


#    def setInitialPose(self):
#        self._q = np.zeros(4)
#        self._q[0] = np.random.random()
#        self._q[1] = np.random.random()
#        theta = np.random.random() * 2 * np.pi - np.pi
#        self._q[2] = np.cos(theta)
#        self._q[3] = np.sin(theta)


class SimulatedMirRobotManager(AbstractMirRobotManager, AbstractSimulatedRobotManager):
    def __init__(self, cfg: ConfigBase, diff_drive: bool):
        if cfg.debug_prints:
            print("SimulatedMirRobotManager init")
        super().__init__(cfg, diff_drive)
        # self._q = np.zeros(4)
        # self._q[2] = 1.0
        self.setInitialPose()

    def setInitialPose(self):
        self._q = np.zeros(4)
        self._q[0] = np.random.random()
        self._q[1] = np.random.random()
        theta = np.random.random() * 2 * np.pi - np.pi
        self._q[2] = np.cos(theta)
        self._q[3] = np.sin(theta)


def mir_approximation(
    diff_drive: bool,
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel, pin.Data]:
    # mobile base as planar joint (there's probably a better
    # option but whatever right now)
    model_mobile_base = pin.Model()
    model_mobile_base.name = "mobile_base"
    geom_model_mobile_base = pin.GeometryModel()
    joint_name = "mobile_base_planar_joint"
    parent_id = 0
    # TEST
    joint_placement = pin.SE3.Identity()
    MOBILE_BASE_JOINT_ID = model_mobile_base.addJoint(
        parent_id, pin.JointModelPlanar(), joint_placement.copy(), joint_name
    )
    # we should immediately set velocity limits.
    # there are no position limit by default and that is what we want.
    # TODO: put in heron's values
    # TODO: make these parameters the same as in mpc_params in the planner
    model_mobile_base.velocityLimit[0] = 2
    model_mobile_base.velocityLimit[2] = 2
    # TODO: i have literally no idea what reasonable numbers are here
    model_mobile_base.effortLimit[0] = 200
    model_mobile_base.effortLimit[2] = 200
    if diff_drive:
        model_mobile_base.effortLimit[1] = 0
        model_mobile_base.velocityLimit[1] = 0
    else:
        model_mobile_base.effortLimit[1] = 200
        model_mobile_base.velocityLimit[1] = 2
    # print("OBJECT_JOINT_ID",OBJECT_JOINT_ID)
    # body_inertia = pin.Inertia.FromBox(cfg.box_mass, box_dimensions[0],
    #        box_dimensions[1], box_dimensions[2])

    # pretty much random numbers
    # TODO: find heron (mir) numbers
    body_inertia = pin.Inertia.FromBox(30, 0.8, 0.5, 0.872)
    # maybe change placement to sth else depending on where its grasped
    model_mobile_base.appendBodyToJoint(
        MOBILE_BASE_JOINT_ID, body_inertia, pin.SE3.Identity()
    )
    # this is the heron box, not just the platform
    box_shape = coal.Box(0.8, 0.5, 0.872)
    body_placement = pin.SE3.Identity()
    body_placement.translation[2] += 0.436
    geometry_mobile_base = pin.GeometryObject(
        "box_shape", MOBILE_BASE_JOINT_ID, box_shape, body_placement.copy()
    )

    geometry_mobile_base.meshColor = np.array([1.0, 0.1, 0.1, 0.3])
    geom_model_mobile_base.addGeometryObject(geometry_mobile_base)

    # have to add the frame manually
    # it's tool0 because that's used everywhere
    model_mobile_base.addFrame(
        pin.Frame(
            "mobile_base",
            MOBILE_BASE_JOINT_ID,
            0,
            joint_placement.copy(),
            pin.FrameType.JOINT,
        )
    )

    data = model_mobile_base.createData()

    return (
        model_mobile_base,
        geom_model_mobile_base.copy(),
        geom_model_mobile_base.copy(),
        data,
    )
