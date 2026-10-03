from smc.bookkeeping.base_config import ConfigBase
from smc.robots.abstract_simulated_robotmanager import AbstractSimulatedRobotManager
from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.robots.interfaces.mobile_base_interface import (
    get_mobile_base_model,
)
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.robots.implementations.ur5e import get_model


import numpy as np
import pinocchio as pin


class AbstractHeronRobotManager(
    ForceTorqueOnSingleArmWrist, SingleArmWholeBodyInterface
):
    def __init__(self, cfg: ConfigBase, diff_drive_base: bool):
        if cfg.debug_prints:
            print("AbstractHeronRobotManager init")
        self._model, self._collision_model, self._visual_model, self._data = (
            heron_approximation(diff_drive_base)
        )
        self._ee_frame_id = self.model.getFrameId("tool0")
        self._base_frame_id = self.model.getFrameId("mobile_base")
        # TODO: CHANGE THIS TO REAL VALUES
        self._MAX_ACCELERATION = 1.7  # const
        self._MAX_QD = 3.14  # const
        self._comfy_configuration = np.array(
            [
                0.0,
                0.0,
                1.0,
                0.0,
                1.54027569e00,
                -1.95702042e00,
                1.46127540e00,
                -1.07315435e00,
                -1.61189968e00,
                -1.65158907e-03,
            ]
        )
        super().__init__(cfg)


class SimulatedHeronRobotManager(
    AbstractHeronRobotManager, AbstractSimulatedRobotManager
):
    def __init__(self, cfg: ConfigBase, diff_drive_base: bool):
        if cfg.debug_prints:
            print("SimulatedRobotManagerHeron init")
        super().__init__(cfg, diff_drive_base)
        self.setInitialPose()

    # NOTE: overriding wrench stuff here
    # there can be a debated whether there should be a simulated forcetorquesensorinterface,
    # but it's annoying as hell and there is no immediate benefit in solving this problem
    def _updateWrench(self) -> None:
        self._wrench_base = np.random.random(6)
        # NOTE: create a robot_math module, make this mapping a function called
        # mapse3ToDifferent frame or something like that
        mapping = np.zeros((6, 6))
        mapping[0:3, 0:3] = self._T_w_e.rotation
        mapping[3:6, 3:6] = self._T_w_e.rotation
        self._wrench = mapping.T @ self._wrench_base

    def zeroFtSensor(self) -> None:
        self._wrench_bias = np.zeros(6)

    def setInitialPose(self):
        if self.cfg.start_from_current_pose:
            # TODO: read position from localization topic,
            # put that into _q
            # HAS TO BE [x, y, cos(theta), sin(theta)] due to pinocchio's
            # representation of planar joint state
            self._q = np.zeros(self.nq)
            rtde_receive = RTDEReceiveInterface(self.cfg.robot_ip)
            self._q[4:] = np.array(rtde_receive.getActualQ())
            raise NotImplementedError("there is no localization here!")
        else:
            self._q = pin.randomConfiguration(
                self.model, self.model.lowerPositionLimit, self.model.upperPositionLimit
            )
            # pin.RandomConfiguration does not work well for planar joint,
            # or at least i remember something along those lines being the case
            self._q[0] = np.random.random()
            self._q[1] = np.random.random()
            theta = np.random.random() * 2 * np.pi - np.pi
            self._q[2] = np.cos(theta)
            self._q[3] = np.sin(theta)


def heron_approximation(
    diff_drive_base: bool,
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel, pin.Data]:
    # arm + gripper
    model_arm, collision_model_arm, visual_model_arm, _ = get_model(
        with_gripper_joints=True
    )

    # mobile base as planar joint (there's probably a better
    # option but whatever right now)
    model_mobile_base, geom_model_mobile_base = get_mobile_base_model(diff_drive_base)
    # frame-index should be 1
    model, visual_model = pin.appendModel(
        model_mobile_base,
        model_arm,
        geom_model_mobile_base,
        visual_model_arm,
        1,
        pin.SE3.Identity(),
    )
    model = pin.buildReducedModel(model, [8, 9], np.zeros(model.nq))
    data = model.createData()

    # fix gripper
    for geom in visual_model.geometryObjects:
        if "hand" in geom.name:
            s = geom.meshScale
            geom.meshcolor = np.array([1.0, 0.1, 0.1, 1.0])
            # this looks exactly correct lmao
            s *= 0.001
            geom.meshScale = s

    return model, visual_model.copy(), visual_model, data


# NOTE: this gives me a flying joint for the camera,
# and a million joints for wheels -> it's unusable
# TODO: look what's done in pink, see if this can be usable
# after you've removed camera joint and similar.
# NOTE: NOT USED, BUT STUFF WILL NEED TO BE EXTRACTED FROM THIS EVENTUALLY
def get_heron_model_from_full_urdf() -> (
    tuple[pin.Model, pin.GeometryModel, pin.GeometryModel, pin.Data]
):

    # urdf_path_relative = files('smc.robot_descriptions.urdf').joinpath('ur5e_with_robotiq_hande_FIXED_PATHS.urdf')
    urdf_path_absolute = "/home/gospodar/home2/gospodar/lund/praxis/software/ros/ros-containers/home/model.urdf"
    # mesh_dir = files('smc')
    # mesh_dir_absolute = os.path.abspath(mesh_dir)
    mesh_dir_absolute = "/home/gospodar/lund/praxis/software/ros/ros-containers/home/heron_description/MIR_robot"

    model = None
    collision_model = None
    visual_model = None
    # this command just calls the ones below it. both are kept here
    # in case pinocchio people decide to change their api.
    # model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path_absolute, mesh_dir_absolute)
    model = pin.buildModelFromUrdf(urdf_path_absolute)
    visual_model = pin.buildGeomFromUrdf(
        model, urdf_path_absolute, pin.GeometryType.VISUAL, None, mesh_dir_absolute
    )
    collision_model = pin.buildGeomFromUrdf(
        model, urdf_path_absolute, pin.GeometryType.COLLISION, None, mesh_dir_absolute
    )

    data = pin.Data(model)

    return model, collision_model, visual_model, data
