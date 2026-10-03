from smc.bookkeeping.load_config import GlobalConfig
from smc.multiprocessing.networking.util import DictPb2EncoderDecoder

import socket
from google.protobuf.internal.encoder import _VarintBytes


from multiprocessing import Queue
from typing import Any


def server_sender(
    host: str, port: int, cfg: GlobalConfig, init_command: dict[str, Any], queue: Queue
):
    """
    server
    -------
    listens for a connection, then sends messages to the singular accepted client

    ex. host = "127.0.0.1"
    ex. host_port = 7777

    use comm_direction = 0 in processmanager for this
    """
    encoder_decoder = DictPb2EncoderDecoder()
    host_addr = (host, port)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(host_addr)
    except OSError as e:
        print(e)
        print(
            f"i'm assuming this is [Errno 48] Address already in use, on port {port} \n \
    this happens because the os did not clean the previous socket. please manually select a different port through an argument. email the smc author if you really need this fixed automatically"
        )
    if cfg.debug_prints:
        print("[SMC NETWORKING] SERVER_SENDER: server listening on", host_addr)
    try:
        s.listen()
    except KeyboardInterrupt:
        s.close()
        if cfg.debug_prints:
            print("[SMC NETWORKING] SERVER_SENDER: caught KeyboardInterrupt, i'm out")
        return
    comm_socket, comm_addr = s.accept()
    # we're only accepting a single connection
    s.close()
    if cfg.debug_prints:
        print("[SMC NETWORKING] SERVER_SENDER: accepted a client", comm_addr)
    try:
        while True:
            # the construction of the message should happen in processmanager
            msg_dict = queue.get()
            if msg_dict == "befree":
                if cfg.debug_prints:
                    print(
                        "[SMC NETWORKING] SERVER_SENDER: got befree, networking server out"
                    )
                break
            comm_socket.send(encoder_decoder.dictToSerializedPb2Msg(msg_dict))
    except KeyboardInterrupt:
        if cfg.debug_prints:
            print(
                "[SMC NETWORKING] SERVER_SENDER: caugth KeyboardInterrupt, networking server out"
            )
    comm_socket.close()


# TODO: server receiver
