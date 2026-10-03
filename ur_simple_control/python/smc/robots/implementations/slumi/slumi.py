from smc.robots.abstract_simulated_robotmanager import AbstractSimulatedRobotManager
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.robots.implementations.yumi import get_yumi_model
from smc.bookkeeping.load_config import GlobalConfig

from importlib.util import find_spec
import pinocchio as pin
import numpy as np
from os import path
from functools import partial

# for startfromcurrentpose
from smc.multiprocessing import NetworkClientProcess
import coal


# TODO: write ForceTorqueDualArmInterface
# to be able to use the force-torque sensors
class AbstractSLuMiRobotManager(DualArmWholeBodyInterface):

    # def __init__(self, cfg: GlobalConfig, diff_drive: bool):
    def __init__(self, cfg: GlobalConfig):
        if cfg.debug_prints:
            print("AbstractSLuMiRobotManager init")
        self.cfg = cfg
        # TODO: implement this function
        # TODO: in it, write options for diff drive or no (ofc the real is diff drive,
        # but adding full actuation is useful in sim to compare shit (trust))
        self._model, self._collision_model, self._visual_model, self._data = (
            get_slumi_model()
        )

        if path.isfile(
            "/opt/ros/humble/share/abb_irb14000_description/meshes/irb14000_05_50/visual/base_link.stl"
        ):
            self._l_ee_frame_name = "robl_tool0"
            self._r_ee_frame_name = "robr_tool0"
        else:
            self._l_ee_frame_name = "gripper_l_joint"
            self._r_ee_frame_name = "gripper_r_joint"
        self._l_ee_frame_id = self.model.getFrameId(self._l_ee_frame_name)
        self._r_ee_frame_id = self.model.getFrameId(self._r_ee_frame_name)
        self._base_frame_id = self._model.getFrameId("mobile_base")
        # TODO: CHANGE THIS TO REAL VALUES
        self._MAX_ACCELERATION = 1.7  # const
        self._MAX_QD = 3.14  # const

        self._mode: DualArmWholeBodyInterface.control_mode = (
            DualArmWholeBodyInterface.control_mode.whole_body
        )
        self._comfy_configuration = np.array(
            [
                0.0,  # x
                0.0,  # y
                1.0,  # cos(theta)
                0.0,  # sin(theta)
                -1.0,
                -2.0,
                1.2,
                0.6,
                2.0,
                1.0,
                0.0,
                1.0,
                -2.0,
                -1.2,
                0.6,
                -2.0,
                1.0,
                0.0,
            ]
        )
        # reset configuration
        self.reset_pose = np.array(
            [
                0.0,  # x
                0.0,  # y
                1.0,  # cos(theta)
                0.0,  # sin(theta)
                -0.7,
                -1.7,
                0.8,
                1.0,
                2.2,
                1.0,
                0.0,
                0.7,
                -1.7,
                -0.8,
                1.0,
                -2.2,
                1.0,
                0.0,
            ]
        )

        # calibration configuration
        self.calib_pose = np.array(
            [
                0.0,  # x
                0.0,  # y
                1.0,  # cos(theta)
                0.0,  # sin(theta)
                0.0,
                -2.270,
                2.356,
                0.524,
                0.0,
                0.670,
                0.0,
                0.0,
                -2.270,
                -2.356,
                0.524,
                0.0,
                0.670,
                0.0,
            ]
        )

        # TODO: remove once we're doing carting around
        self.addCartIntoVisualizationAndCollision()
        super().__init__(cfg)

    def addCartIntoVisualizationAndCollision(self):
        # 1. Define the box shape and its relative placement to the EE frame
        box_shape = coal.Box(0.86, 0.35, 0.96)
        self.cart_shape = (0.86, 0.35, 0.96)
        ee_frame_id = self._model.getFrameId("gripper_l_joint")
        ee_frame = self._model.frames[ee_frame_id]
        # NOTE: this one is perfect for where the cart is.
        # but it catches the collision with the gripper.
        # i don't want to fix this now,
        # so i'll just lower it
        # TODO: instead, make viz accurate, and remove that collision pair.
        # placement = pin.SE3(np.eye(3), np.array([0.7 / 2, 0.15, 0.96 / 2 + 0.15]))
        placement = pin.SE3(
            np.eye(3), np.array([0.86 / 2, 0.15, 0.96 / 2 + 0.15 + 0.05])
        )

        # 2. Create the Collision Geometry Object
        box_collision = pin.GeometryObject(
            "cart", ee_frame.parentJoint, ee_frame_id, placement, box_shape
        )
        box_collision.meshColor = np.array([0.0, 0.5, 0.5, 0.3])  # some teal

        # 3. Add to the collision model
        box_id = self._collision_model.addGeometryObject(box_collision)
        box_id = self._visual_model.addGeometryObject(box_collision)


# TODO: make this work in an eventual refactor (for control modes to work with pinocchio algorithms without a hassle)
#  (then the jacobians would need to be rewritten and most likely some other stuff too)
#    @property
#    def model(self) -> pin.Model:
#        if self._mode == AbstractRobotManager.control_mode.whole_body:
#            return self._model
#        if self._mode == AbstractRobotManager.control_mode.base_only:
#            return self._base_only_model
#        if self._mode == AbstractRobotManager.control_mode.upper_body:
#            return self._upper_body_model
#        return self._model
# NOTE:  there's a shitton of stuff to re-write for this to work, and i'm not doing it now
#        self._base_only_model, _ = get_mobile_base_model(underactuated=False)
#        self._upper_body_model, _, _, _ = get_yumi_model()
#        print(self._upper_body_model)


class SimulatedSLuMiRobotManager(
    AbstractSimulatedRobotManager, AbstractSLuMiRobotManager
):
    def __init__(self, cfg: GlobalConfig):
        if cfg.debug_prints:
            print("SimulatedSLuMiRobotManager init")
        super().__init__(cfg)

    def setInitialPose(self):
        if self.cfg.start_from_current_pose:
            if find_spec("docker"):
                from smc.robots.implementations.slumi.docker_utils import (
                    startDockerTranslationLayer,
                )
            else:
                raise ImportError(
                    "you need to install docker (both actual docker and the python package) to run the translation container which communicates with the robot"
                )

            try:
                from smc.multiprocessing.networking.client import client_receiver
            except ImportError:
                print(
                    "you need to install protobuf to communicate with the translation layer which runs in docker. you might have to recompile messages with the appropriate protoc version also. you find old protoc on protobuf's github page. needs to be old enough to work with the messages in the translation layer. look into the dockerfile for details"
                )
                exit()
            # TODO: you should load the cfg here.
            # the design for how to do this is ass atm.
            if not self.cfg.slumi.manual_translation_start:
                docker_client, translation_container = startDockerTranslationLayer(
                    self.cfg.slumi,
                    "slumi_translator",
                )
            print(
                "SLUMI_SIM_MANAGER: waiting to receive initial pose. the visualizer is broken until i get it [upside down smile emoji]"
            )
            # NOTE: we always receive the full joint position vector
            # for communication simplicity.
            side_data = partial(
                client_receiver, self.cfg.slumi.host, self.cfg.slumi.port_data
            )
            client_receiver_data = NetworkClientProcess(
                self.cfg,
                side_data,
                {"q": np.zeros(self.model.nq)},
            )
            q_recv = np.zeros(self.model.nq)
            while np.linalg.norm(q_recv) == 0.0:
                q_recv = client_receiver_data.getData()["q"]
            self._q = q_recv
            client_receiver_data.terminateProcess()
            if not self.cfg.manual_translation_start:
                docker_client.containers.model.kill("slumi_translator")

        else:
            # self._q = pin.randomConfiguration(
            #    self.model, self.model.lowerPositionLimit, self.model.upperPositionLimit
            # )
            self._q[4:] = self._comfy_configuration[4:] * (1 + 0.1 * np.random.random())
            # pin.RandomConfiguration does not work well for planar joint,
            # or at least i remember something along those lines being the case
            self._q[0] = np.random.random() * 0.1 - 0.05
            self._q[1] = np.random.random() * 0.1 - 0.05
            theta = (np.random.random() * 2 * np.pi - np.pi) * 0.3
            self._q[2] = np.cos(theta)
            self._q[3] = np.sin(theta)


def get_slumi_model() -> (
    # TODO: base not implemented, mesh files are not where they should be
    # so no visualization is actually there (for yumi, in general)
    tuple[pin.Model, pin.GeometryModel, pin.GeometryModel, pin.Data]
):

    model_arms, collision_model_arms, visual_model_arms, _ = get_yumi_model()
    model_mobile_base, geom_model_mobile_base = get_slumi_mobile_base_model()

    # frame-index should be 1
    yumi_placement_on_sleipner_transform = pin.SE3.Identity()
    yumi_placement_on_sleipner_transform.rotation = pin.rpy.rpyToMatrix(0.0, 0.0, np.pi)
    yumi_placement_on_sleipner_transform.translation[0] = -1 * (0.87 - (1.04 / 2))
    yumi_placement_on_sleipner_transform.translation[1] = 0.0
    yumi_placement_on_sleipner_transform.translation[2] = 0.805
    model_mobile_base.addFrame(
        pin.Frame(
            "mobile_base",
            1,
            #            0,
            pin.SE3.Identity(),
            # yumi_placement_on_sleipner_transform.copy(),
            pin.FrameType.JOINT,
        )
    )
    model, visual_model = pin.appendModel(
        model_mobile_base,
        model_arms,
        geom_model_mobile_base,
        visual_model_arms,
        1,
        yumi_placement_on_sleipner_transform.copy(),
    )
    data = model.createData()

    return model, visual_model.copy(), visual_model, data


def get_slumi_mobile_base_model() -> tuple[pin.Model, pin.GeometryModel]:

    # mobile base as planar joint (there's probably a better
    # option but whatever right now)
    model_mobile_base = pin.Model()
    model_mobile_base.name = "mobile_base"
    geom_model_mobile_base = pin.GeometryModel()
    joint_name = "mobile_base_planar_joint"
    parent_id = 0
    MOBILE_BASE_JOINT_ID = model_mobile_base.addJoint(
        parent_id, pin.JointModelPlanar(), pin.SE3.Identity(), joint_name
    )
    # we should immediately set velocity limits.
    # there are no position limit by default and that is what we want.
    # TODO: put in slumi's values
    # TODO: make these parameters the same as in mpc_params in the planner
    # NOTE: in real life the limit is 6 (all dirs), but we don't actually want this!
    model_mobile_base.velocityLimit[0] = 3
    model_mobile_base.velocityLimit[1] = 3
    model_mobile_base.velocityLimit[2] = 3
    model_mobile_base.effortLimit[0] = 200
    model_mobile_base.effortLimit[1] = 200
    model_mobile_base.effortLimit[2] = 200

    body_inertia = pin.Inertia.FromBox(83 + 50, 1.04, 0.58, 0.805)
    # maybe change placement to sth else depending on where its grasped
    model_mobile_base.appendBodyToJoint(
        MOBILE_BASE_JOINT_ID, body_inertia, pin.SE3.Identity()
    )
    box_shape = coal.Box(1.04, 0.58, 0.805)
    body_placement = pin.SE3.Identity()
    body_placement.translation[2] += 0.4025
    geometry_mobile_base = pin.GeometryObject(
        "box_shape", MOBILE_BASE_JOINT_ID, box_shape, body_placement.copy()
    )

    geometry_mobile_base.meshColor = np.array([1.0, 0.1, 0.1, 1.0])
    geom_model_mobile_base.addGeometryObject(geometry_mobile_base)

    return model_mobile_base, geom_model_mobile_base
