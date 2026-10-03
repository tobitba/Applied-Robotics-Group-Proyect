from smc.multiprocessing.consumer_process import ConsumerProcess
from smc.robots.abstract_robotmanager import (
    AbstractRealRobotManager,
)
from smc.robots.implementations.slumi.slumi import AbstractSLuMiRobotManager
from smc.multiprocessing import NetworkClientProcess
from smc.multiprocessing.networking.client import client_receiver, client_sender
from smc.robots.implementations.slumi.docker_utils import startDockerTranslationLayer
from smc.bookkeeping.load_config import GlobalConfig

import numpy as np
from pinocchio import SE3
import time
from functools import partial
from sys import platform

# NOTE: do not load any ros stuff if it isn't installed
from importlib.util import find_spec

# TODO: import networking server and client according to need,
# preferably clients though (let translation layer start first and be the server,
# i don't want control to be a server literaly just for nicer design (even though it doesn't matter))


if find_spec("docker"):
    import docker
else:
    raise ImportError(
        "can't run translation container without the docker package (pip install docker)"
    )


class RealSLuMiRobotManager(AbstractSLuMiRobotManager, AbstractRealRobotManager):
    def __init__(
        self,
        cfg: GlobalConfig,
    ):
        # TODO: write ForceTorqueDualArmInterface
        # to be able to use the force-torque sensors
        # self._wrench_base: np.ndarray = np.zeros(6)
        # self._wrench: np.ndarray = np.zeros(6)
        ## NOTE: wrench bias will be defined in the frame your sensor's gives readings
        # self._wrench_bias: np.ndarray = np.zeros(6)
        self._T_w_e = SE3.Identity()

        # AbstractSLuMiRobotManager.__init__(self, cfg, True)
        AbstractSLuMiRobotManager.__init__(self, cfg)
        # NOTE: we store the computed control signal here
        # it is then read and published by ros topics.
        # this is the thinnest possible hack to bridge
        # SMC and ROS API.
        # but also objectively required due to differt timing of actuators????
        self._v_cmd = np.zeros(self.model.nv)

        self.cfg = cfg

        self._dt = 1 / self.cfg.ctrl_freq
        # TODO: either read from argument, or get Hz from something
        self._base_cmd_dt = 1 / 20

        self.current_iteration = 0

        self.odom_initialized = False
        self.init_odom = np.zeros(3)
        # TODO: figure out what to do with the grippers
        self.client_receiver_data: NetworkClientProcess
        self.client_sender_command: ConsumerProcess
        # TODO: idk which one this even is, should be abstractslumi
        # super().__init__(cfg, True)
        # super().__init__(cfg)
        # TODO: figure out how to use setInitial pose and put it here.
        # a hacky but stable solution is good.

    def _updateQ(self):
        # NOTE: here it is assumed all pre-processing like odometry framing has
        # been done in the translation layer (it has)
        self._q = self.client_receiver_data.getData()["q"]

    # TODO: have this piece together odom and arms
    def _updateV(self):
        pass

    def connectToRobot(self):
        # start translation docker container
        # command_port: 6666
        # data_port: 7777
        try:
            print("this is the port to send cmds:", self.cfg.slumi.port_command)
        except AttributeError:
            print(
                "you didn't load slumi connection cfg to your main. i'm putting them to default for you"
            )
            if platform != "darwin":
                self.cfg.slumi.host = "127.0.0.1"
            else:
                # NOTE: macos-docker specific. sharing is not via localhost
                self.cfg.slumi.host = "0.0.0.0"
            self.cfg.slumi.port_data = 6666
            self.cfg.slumi.port_command = 7777

        if not self.cfg.slumi.manual_translation_start:
            self.container_name = "slumi_translator"
            self.docker_client, self.translation_container = (
                startDockerTranslationLayer(self.cfg.slumi, self.container_name)
            )

        # TODO: write dual arm ft,
        # write connection to those sensors
        # prio 2-3 tbh
        # self.wrench_offset = self.calibrateFT(self._dt)

        # NOTE: this is how we ensure nothing will start before we get first q
        self._q = np.zeros(self.model.nq)

        cli_data = partial(
            client_receiver, self.cfg.slumi.host, self.cfg.slumi.port_data
        )
        # NOTE: we always receive the full joint position vector
        # for communication simplicity.
        self.client_receiver_data = NetworkClientProcess(
            self.cfg,
            cli_data,
            {"q": self._q},
        )
        # NOTE: we always send the full velocity
        # for communication simplicity.
        cli_command = partial(
            client_sender, self.cfg.slumi.host, self.cfg.slumi.port_command
        )
        self.client_sender_command = ConsumerProcess(
            self.cfg,
            cli_command,
            {"v_cmd": np.zeros(self._model.nv)},
        )
        while np.linalg.norm(self._q) == 0.0:
            self._updateQ()

    # NOTE: handled by ros in the translation layer
    def connectToGripper(self):
        pass

    def setInitialPose(self):
        # NOTE: if you're running on real you're starting from what the robot gives you
        pass

    # TODO :(
    def _updateWrench(self):
        pass

    # TODO :(
    def zeroFtSensor(self):
        pass

    # NOTE: look at sendVelocityCommand in AbstractRobotManager to
    # understand relationship between sendVelocityCommand and sendVelocityCommandToReal
    def sendVelocityCommandToReal(self, v_cmd):
        self.client_sender_command.sendCommand({"v_cmd": v_cmd})

    def stopRobot(self):
        for _ in range(300):
            self.client_sender_command.sendCommand({"v_cmd": np.zeros(self.model.nv)})
            time.sleep(self._dt)
        print("successfully sent 300 zero velocity commands")
        self.client_sender_command.sendCommand("befree")

    # NOTE: called leadthrough on yumi
    def setFreedrive(self):
        raise NotImplementedError

    # NOTE: called leadthrough on yumi
    def unSetFreedrive(self):
        raise NotImplementedError
