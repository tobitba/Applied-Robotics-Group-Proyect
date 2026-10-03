from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.types import Command
from smc.multiprocessing.networking.util import DictPb2EncoderDecoder

from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf.internal.decoder import _DecodeVarint32
from multiprocessing import shared_memory, Queue
from multiprocessing.synchronize import Lock
import socket
import pickle
import time
from sys import platform


def client_receiver(
    host: str,
    port: int,
    cfg: GlobalConfig,
    init_command: Command,
    shm_name: str,
    lock: Lock,
):
    """
    client
    -------
    connects to a server, then receives messages from it.
    offers latest message via shared memory

    the message is not deserialized here because it needs to
    be serialized to be put into shared memory anyway.
    so better this than to deserialize and the serialize
    again differently.

    to use this, you comm_direction = 4 in processmanager

    ex. host = "127.0.0.1"
    ex. host_port = 7777
    """

    def parse_message(buffer: bytes):
        """
        parse_message
        -------------
        here the message is what we got from recv(),
        and parsing it refers to finding serialized pb2 messages in it
        NOTE: we only keep the latest message because
              we're not going to do anything with the missed message.
              the assumption is that we only get the same kind of message from
              a sensor or something similar, not files or whatever else needs to be whole
        """
        pos, next_pos = 0, 0
        buffer_len = len(buffer)
        msg_in_bytes = b""
        len_size_offset = 0
        while True:
            try:
                next_pos, pos = _DecodeVarint32(buffer, pos)
            except IndexError:
                if cfg.debug_prints:
                    print(
                        "[SMC NETWORKING] CLIENT_RECEIVER: got IndexError while decoding message length. this only happens if the sender died. i'll exit as well"
                    )
                return None, -1
            # TODO: either save the message chunk, or save how many initial bytes to ignore in the next message
            if pos + next_pos > buffer_len:
                # print("NETWORKING CLIENT: BUFFER OVERFLOW, DROPPING MSG!")
                return msg_in_bytes, pos - len_size_offset
            msg_in_bytes = _VarintBytes(pos + next_pos) + buffer[pos : pos + next_pos]
            len_size_offset = len(_VarintBytes(pos + next_pos))
            pos += next_pos
            if pos >= buffer_len:
                return msg_in_bytes, pos

    buffer = b""
    encoder_decoder = DictPb2EncoderDecoder()
    msg_code = encoder_decoder.dictToMsgCode(init_command)

    shm = shared_memory.SharedMemory(name=shm_name)
    host_addr = (str(host), int(port))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while True:
        try:
            s.connect(host_addr)
            break
        except ConnectionRefusedError:
            # NOTE: today i learned i fun fact i never wanted to know about.
            # on mac and bsd, if you try to connect with a socket, and it doesn't work,
            # you can just throw it away. why? i have absolutely no idea.
            # this is not done in all cases because i want to be reminded
            # i overcame this problem and am still ok.
            if platform == "darwin":
                s.close()
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            time.sleep(0.005)
        except KeyboardInterrupt:
            s.close()
            if cfg.debug_prints:
                print(
                    "[SMC NETWORKING] CLIENT_RECEIVER: caught KeyboardInterrupt, i'm out"
                )
            return
    if cfg.debug_prints:
        print("[SMC NETWORKING] CLIENT_RECEIVER: connected to server")

    try:
        while True:
            msg_raw = s.recv(1024)
            buffer += msg_raw
            msg_in_bytes, pos = parse_message(buffer)
            # -1 means an error happened
            if pos == -1:
                break
            buffer = buffer[pos:]
            dict_msg = encoder_decoder.serializedPb2MsgToDict(msg_in_bytes, msg_code)

            # print(
            #    "NETWORKING CLIENT: putting new message in:",
            #    encoder_decoder.serializedPb2MsgToDict(msg_in_bytes, 1),
            # )
            # TODO: I CAN'T READ THIS WITHOUT pre-CROPPING THE MESSAGE
            # WHEN SENDING TO serializedPb2MsgToDict!!!!!!!!!!!
            # THE FUNCTION CAN CORRECTLY DECODE THE LENGTH OF THE MESSAGE,
            # AND I CROP IT THERE, BUT FOR SOME REASON IT REFUSES TO COORPERATE
            # so fck it i'm reserializing with pickling to avoid wasting more time
            # shm.buf[: len(msg_in_bytes)] = msg_in_bytes[:]
            dict_msg_pickle = pickle.dumps(dict_msg)
            lock.acquire()
            shm.buf[: len(dict_msg_pickle)] = dict_msg_pickle
            # print("NETWORKING CLIENT: message in bytes length", len(msg_in_bytes))
            # mem = shm.buf[: len(msg_in_bytes)]
            ## print(mem)
            # print("NETWORKING CLIENT: i can read back from shm:")
            # print("NETWORKING_CLIENT", encoder_decoder.serializedPb2MsgToDict(mem, 1))
            # NOTE: this works just fine, but not, but you have to crop here with len(msg_in_bytes),
            # even though the message begins with the int which is the same number.
            # but cropping in serializedPb2MsgToDict refuses to work
            # print(
            #   "NETWORKING_CLIENT",
            # encoder_decoder.serializedPb2MsgToDict(shm.buf[: len(msg_in_bytes)], 1),
            # )
            lock.release()
    except KeyboardInterrupt:
        s.close()
        if cfg.debug_prints:
            print("[SMC NETWORKING] CLIENT_RECEIVER: caught KeyboardInterrupt, i'm out")


def client_sender(
    host: str, port: int, cfg: GlobalConfig, init_command: Command, queue: Queue
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
    while True:
        try:
            s.connect(host_addr)
            break
        except ConnectionRefusedError:
            # NOTE: today i learned i fun fact i never wanted to know about.
            # on mac and bsd, if you try to connect with a socket, and it doesn't work,
            # you can just throw it away. why? i have absolutely no idea.
            # this is not done in all cases because i want to be reminded
            # i overcame this problem and am still ok.
            if platform == "darwin":
                s.close()
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            time.sleep(0.005)
        except KeyboardInterrupt:
            s.close()
            if cfg.debug_prints:
                print(
                    "[SMC NETWORKING] CLIENT_SENDER: caught KeyboardInterrupt, i'm exiting"
                )
            return
    if cfg.debug_prints:
        print("[SMC NETWORKING] CLIENT_SENDER: connected to server")

    try:
        while True:
            # the construction of the message should happen in processmanager
            msg_dict = queue.get()
            if msg_dict == "befree":
                if cfg.debug_prints:
                    print("[SMC NETWORKING] CLIENT_SENDER: got befree, i'm exiting")
                break
            s.send(encoder_decoder.dictToSerializedPb2Msg(msg_dict))
    except KeyboardInterrupt:
        if cfg.debug_prints:
            print(
                "[SMC NETWORKING] CLIENT_SENDER: caught KeyboardInterrupt, i'm exiting"
            )
    s.close()
