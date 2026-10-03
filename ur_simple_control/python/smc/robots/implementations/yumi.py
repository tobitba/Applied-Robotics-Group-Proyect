from smc.bookkeeping.base_config import ConfigBase
from smc.robots.abstract_robotmanager import AbstractRealRobotManager
from smc.robots.abstract_simulated_robotmanager import AbstractSimulatedRobotManager
from smc.robots.interfaces.dual_arm_interface import DualArmInterface

from importlib.resources import files
from os import path
import pinocchio as pin
import numpy as np
from copy import deepcopy


class AbstractYuMiRobotManager(DualArmInterface):
    def __init__(self, cfg: ConfigBase):
        if cfg.debug_prints:
            print("YuMiRobotManager init")
        self._model, self._collision_model, self._visual_model, self._data = (
            get_yumi_model()
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

        self._MAX_ACCELERATION = 1.7  # const
        self._MAX_QD = 3.14  # const
        # initial configuration
        self._comfy_configuration = np.array(
            [
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

        super().__init__(cfg)


class SimulatedYuMiRobotManager(
    AbstractSimulatedRobotManager, AbstractYuMiRobotManager
):
    def __init__(self, cfg: ConfigBase):
        if cfg.debug_prints:
            print("SimulatedYuMiRobotManager init")
        #        AbstractYuMiRobotManager.__init__(self, cfg)
        AbstractSimulatedRobotManager.__init__(self, cfg)


# TODO: write this if/when you'll need it
class RealYuMiRobotManager(AbstractRealRobotManager, AbstractYuMiRobotManager):
    pass


def get_yumi_model() -> (
    tuple[pin.Model, pin.GeometryModel, pin.GeometryModel, pin.Data]
):

    if path.isfile(
        "/opt/ros/humble/share/abb_irb14000_description/meshes/irb14000_05_50/visual/base_link.stl"
    ):
        urdf_path_relative = files("smc.robots.robot_descriptions").joinpath(
            "yumi.urdf"
        )
    else:
        package_dir_relative = files("smc.robots.robot_descriptions")
        urdf_path_relative = package_dir_relative.joinpath(
            "yumi_description/urdf/yumi.urdf"
        )
    urdf_path_absolute = path.abspath(urdf_path_relative)
    package_dir_absolute = path.abspath(package_dir_relative)

    # this command just calls the ones below it. both are kept here
    # in case pinocchio people decide to change their api.
    # model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path_absolute, package_dir_absolute)
    model = pin.buildModelFromUrdf(urdf_path_absolute)
    visual_model = pin.buildGeomFromUrdf(
        model, urdf_path_absolute, pin.GeometryType.VISUAL, None, package_dir_absolute
    )
    collision_model = pin.buildGeomFromUrdf(
        model,
        urdf_path_absolute,
        pin.GeometryType.COLLISION,
        None,
        package_dir_absolute,
    )

    if not path.isfile(
        "/opt/ros/humble/share/abb_irb14000_description/meshes/irb14000_05_50/visual/base_link.stl"
    ):
        # model = pin.buildReducedModel(model, [8, 9, 17, 18], np.zeros(model.nq))
        joint_names = [
            "gripper_l_joint",
            "gripper_l_joint_m",
            "gripper_r_joint",
            "gripper_r_joint_m",
        ]
        joint_ids = [model.getJointId(name) for name in joint_names]
        joint_vals = np.zeros(model.nq)
        modelcp = deepcopy(model)
        model, visual_model = pin.buildReducedModel(
            model, visual_model, joint_ids, joint_vals
        )
        modelcp, collision_model = pin.buildReducedModel(
            modelcp, collision_model, joint_ids, joint_vals
        )

    # urdf is lying
    model.velocityLimit /= 3.0
    model.velocityLimit[4:7] /= 2.0
    model.velocityLimit[11:14] /= 2.0
    data = pin.Data(model)
    return model, collision_model, visual_model, data
