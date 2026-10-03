import socket
from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf.internal.decoder import _DecodeVarint32
import argparse
import message_specs_pb2
import random
import time
from multiprocessing import Queue

from importlib.util import find_spec

if find_spec("rospy"):
    from geometry_msgs.msg import Pose, PoseStamped, Wrench, Twist
else:
    raise ImportError("you are supposed to run this is computer with ros1! exiting")


# NOTE: return_dict is python's evil pass by reference
# to get the comm_socket back
def connectToClient(service_name: str, host: str, port: int, return_dict: dict):
    host_addr = (host, port)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(host_addr)
    print(
        f"[TRANSLATION LAYER] {service_name}: listening on port ",
        port,
    )
    s.listen()
    comm_socket, comm_addr = s.accept()
    # we're only accepting a single connection
    s.close()
    print(f"[TRANSLATION LAYER] {service_name}: accepted a client", comm_addr)
    return_dict["comm_socket"] = comm_socket


def parseProtobufPacket(buffer: bytes):
    """
    parse_protobuf_message
    ----------------------
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
        next_pos, pos = _DecodeVarint32(buffer, pos)
        if pos + next_pos > buffer_len:
            return msg_in_bytes, pos - len_size_offset
        msg_in_bytes = _VarintBytes(pos + next_pos) + buffer[pos : pos + next_pos]
        len_size_offset = len(_VarintBytes(pos + next_pos))
        pos += next_pos
        if pos >= buffer_len:
            return msg_in_bytes, pos


def receiveProtobuf(
    # pb2_msg_contructor, buffer: dict[str, bytes], receiving_socket: socket.socket
    pb2_msg_contructor,
    buffer,
    receiving_socket: socket.socket,
):
    msg_raw = receiving_socket.recv(1024)
    buffer["b"] += msg_raw
    if len(msg_raw) < 1:
        return None
    msg_in_bytes, pos = parseProtobufPacket(buffer["b"])
    buffer["b"] = buffer["b"][pos:]

    next_pos, pos = 0, 0
    next_pos, pos = _DecodeVarint32(msg_in_bytes, pos)
    pb2_msg_in_bytes_cut = msg_in_bytes[pos : pos + next_pos]
    pb2_msg = pb2_msg_contructor()
    pb2_msg.ParseFromString(pb2_msg_in_bytes_cut)
    return pb2_msg


def receivingAndPuttingToQueue(
    pb2_msg_constructor, receiving_socket: socket.socket, recv_queue: Queue
):
    # NOTE: have to make it a hack to pass by reference
    buffer = {"b": b""}
    while True:
        pb2_msg = receiveProtobuf(pb2_msg_constructor, buffer, receiving_socket)
        if pb2_msg is None:
            print(
                "PROTO_RECV_THREAD: dropped an empty package. this almost certainly means client exited. i'm sending a zero velocity command, then exiting too."
            )
            recv_queue.put(None)
            #            exit()
            # return
            break
        if not recv_queue.full():
            recv_queue.put_nowait(pb2_msg)


def receiveForTest(receiving_socket: socket.socket):
    buffer = b""
    iteration = 0
    while True:
        # receive wrench as protobuf message
        msg_raw = receiving_socket.recv(1024)
        buffer += msg_raw
        if len(msg_raw) < 1:
            continue
        msg_in_bytes, pos = parseProtobufPacket(buffer)
        buffer = buffer[pos:]

        next_pos, pos = 0, 0
        next_pos, pos = _DecodeVarint32(msg_in_bytes, pos)
        pb2_msg_in_bytes_cut = msg_in_bytes[pos : pos + next_pos]
        pb2_msg = message_specs_pb2.v_cmd()
        pb2_msg.ParseFromString(pb2_msg_in_bytes_cut)
        iteration += 1


# return True if successful, False otherwise
def sendProtobuf(pb2_msg, comm_socket: socket.socket) -> bool:
    msg_length = pb2_msg.ByteSize()
    msg_serialized = pb2_msg.SerializeToString()
    msg = _VarintBytes(msg_length) + msg_serialized
    try:
        rez = comm_socket.send(msg)
    except OSError as e:
        print(e)
        print(
            "SENDING_PROTO: i'm assuming this is '[Errno 32] Broken pipe'. but in any case seems like the client died"
        )
        return False
    return True


def sendRandomForTest(cfg: argparse.Namespace, comm_socket: socket.socket):
    success = True
    while True:
        pb2_msg = message_specs_pb2.joint_positions()
        faux_q = [random.random()] * (2 * 7 + 4)
        pb2_msg.q.extend(faux_q)
        success = sendProtobuf(pb2_msg, comm_socket)
        if not success:
            return
        time.sleep(1 / cfg.sending_frequency)


def TwistMsgToList(twist: Twist):
    vx = twist.linear.x
    vy = twist.linear.y
    vz = twist.linear.z
    wx = twist.angular.x
    wy = twist.angular.y
    wz = twist.angular.z
    return [vx, vy, vz, wx, wy, wz]


def WrenchMsgToList(wrench: Wrench):
    fx = wrench.force.x
    fy = wrench.force.y
    fz = wrench.force.z
    mx = wrench.torque.x
    my = wrench.torque.y
    mz = wrench.torque.z
    return [fx, fy, fz, mx, my, mz]
