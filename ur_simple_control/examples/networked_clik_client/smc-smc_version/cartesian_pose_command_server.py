from dataclasses import dataclass, field
from smc import load_config
from smc.bookkeeping.registry import ConfigRegistry
from smc.multiprocessing import ConsumerProcess, NetworkClientProcess
from smc.multiprocessing.networking.server import server_sender
from smc.multiprocessing.networking.client import client_receiver

from pinocchio import SE3
import numpy as np
import time
from functools import partial

ConfigRegistry.register("netex")


@dataclass
class ConfigNetEx:
    host: str = field(default="127.0.0.1", metadata={"help": "host ip address"})
    port_wrench: int = field(default=6666, metadata={"help": "host's port"})
    port_command: int = field(default=7777, metadata={"help": "host's port"})


def sendCircleGoalReceiveWrench(freq, i):
    # create command to go in a circle
    radius = 0.6
    t = (i / freq) / 6
    pose = SE3.Identity()
    pose.translation = radius * np.array([np.cos(t), np.sin(t), 0.0])

    transform = SE3.Identity()
    # transform = SE3(
    #    rpy.rpyToMatrix(0.0, np.pi / 2, 0.0), np.array([0.3, 0.0, 0.3])
    # )

    pose = pose.act(transform)
    # if i % freq == 0:
    #    print(pose.translation)
    # print("translation:", pose.translation)
    # print("rotation:", Quaternion(pose.rotation))

    return {"T_goal": pose, "v": np.zeros(6)}


if __name__ == "__main__":
    cfg = load_config()

    # command_sender: 7777

    side_cmd = partial(server_sender, cfg.netex.host, cfg.netex.port_command)
    sender = ConsumerProcess(
        cfg, side_cmd, {"T_goal": SE3.Identity(), "v": np.zeros(6)}
    )
    # wrench_receiver: 6666
    side_wrench = partial(client_receiver, cfg.netex.host, cfg.netex.port_wrench)
    receiver = NetworkClientProcess(
        cfg,
        side_wrench,
        {"wrench": np.zeros(6)},
    )
    freq = 200
    i = 0
    while True:
        i += 1
        sender.sendCommand(sendCircleGoalReceiveWrench(freq, i))
        if i % freq == 0:
            print(i, receiver.getData())
        time.sleep(1 / 200)
