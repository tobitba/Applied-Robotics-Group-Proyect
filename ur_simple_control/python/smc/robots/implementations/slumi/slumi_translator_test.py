import argparse
from threading import Thread
from smc_to_translator_utils import (
    connectToClient,
    receiveForTest,
    sendRandomForTest,
)


def getArgs():
    parser = argparse.ArgumentParser()
    parser.description = "send and receive random protobuf messages to make sure SMC-translation_layer works"
    parser.add_argument("--host", type=str, help="host ip address", default="0.0.0.0")
    parser.add_argument(
        "--port-data",
        type=int,
        help="host's port for receiving joint positions",
        default=6666,
    )
    parser.add_argument(
        "--port-command",
        type=int,
        help="host's port for sending joint velocity commands",
        default=7777,
    )

    parser.add_argument(
        "--sending-frequency",
        type=int,
        help="recv/send freq for testing",
        default=500,
    )
    cfg = parser.parse_args()
    return cfg


if __name__ == "__main__":
    args = getArgs()
    return_t1 = {}
    t1 = Thread(
        target=connectToClient,
        args=("SERVER_COMMAND_RECEIVER", args.host, args.port_command, return_t1),
    )
    return_t2 = {}
    t2 = Thread(
        target=connectToClient,
        args=("SERVER_JOINT_POSITIONS_SENDER", args.host, args.port_data, return_t2),
    )

    t1.start()
    t2.start()
    t1.join()
    command_socket = return_t1["comm_socket"]
    t2.join()
    joint_position_socket = return_t2["comm_socket"]

    t = Thread(
        target=sendRandomForTest,
        args=(
            args,
            joint_position_socket,
        ),
    )
    t.start()
    receiveForTest(command_socket)
