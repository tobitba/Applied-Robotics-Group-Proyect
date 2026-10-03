from smc import load_config
import socket
from google.protobuf.internal.encoder import _VarintBytes
from google.protobuf.internal.decoder import _DecodeVarint32
import argparse
import message_specs_pb2
from functools import partial
import importlib.util
import random
import time
from threading import Thread

if importlib.util.find_spec("rospy"):
    from agents_test.msg import Command
    import rospy
    from geometry_msgs.msg import WrenchStamped


def getArgs():
    parser = argparse.ArgumentParser()
    parser.description = (
        "get ros1 message, make it a protobuf message, send it via tcp socket"
    )
    parser.add_argument("--host", type=str, help="host ip address", default="127.0.0.1")
    parser.add_argument(
        "--port-wrench", type=int, help="host's port for receiving wrench", default=6666
    )
    parser.add_argument("--port-command", type=int, help="host's port", default=7777)

    cfg = parser.parse_cfg()
    return cfg


def command_callback(command_socket, c):
    position = [c.goal.position.x, c.goal.position.y, c.goal.position.z]
    orientation = [
        c.goal.orientation.w,
        c.goal.orientation.x,
        c.goal.orientation.y,
        c.goal.orientation.z,
    ]
    position_vel = [c.goal_dot.position.x, c.goal_dot.position.y, c.goal_dot.position.z]
    orientation_vel = [
        c.goal_dot.orientation.w,
        c.goal_dot.orientation.x,
        c.goal_dot.orientation.y,
    ]
    pb2_msg = message_specs_pb2.T_goal()
    pb2_msg.position.extend(position)
    pb2_msg.rotation.extend(orientation)
    pb2_msg.velocity.extend(position_vel + orientation_vel)
    msg_length = pb2_msg.ByteSize()
    msg_serialized = pb2_msg.SerializeToString()
    msg = _VarintBytes(msg_length) + msg_serialized
    command_socket.send(msg)


def sendRandomForTest(comm_socket):
    while True:
        pb2_msg = message_specs_pb2.T_goal()
        pb2_msg.position.extend([random.random()] * 3)
        pb2_msg.rotation.extend([random.random()] * 4)
        pb2_msg.velocity.extend([random.random()] * 6)
        msg_length = pb2_msg.ByteSize()
        msg_serialized = pb2_msg.SerializeToString()
        msg = _VarintBytes(msg_length) + msg_serialized
        comm_socket.send(msg)
        time.sleep(0.002)


def parse_message(buffer):
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
        next_pos, pos = _DecodeVarint32(buffer, pos)
        if pos + next_pos > buffer_len:
            return msg_in_bytes, pos - len_size_offset
        msg_in_bytes = _VarintBytes(pos + next_pos) + buffer[pos : pos + next_pos]
        len_size_offset = len(_VarintBytes(pos + next_pos))
        pos += next_pos
        if pos >= buffer_len:
            return msg_in_bytes, pos


def getCommandClient(cfg):
    cfg = getArgs()
    host_addr = (cfg.host, cfg.port_command)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(host_addr)
    print("NETWORKING_SERVER_COMMAND_SENDER listening on port ", cfg.port_command)
    s.listen()
    command_socket, comm_addr = s.accept()
    # we're only accepting a single connection
    s.close()
    print("NETWORKING_SERVER_COMMAND_SENDER: accepted a client", comm_addr)
    return command_socket


def getWrenchClient(cfg):
    cfg = getArgs()
    host_addr = (cfg.host, cfg.port_wrench)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(host_addr)
    print("NETWORKING_SERVER_COMMAND_SENDER listening on port ", cfg.port_wrench)
    s.listen()
    wrench_socket, comm_addr = s.accept()
    # we're only accepting a single connection
    s.close()
    print("NETWORKING_SERVER_COMMAND_SENDER: accepted a client", comm_addr)
    return wrench_socket


if __name__ == "__main__":
    if importlib.util.find_spec("rospy"):
        rospy.init_node("ros1_to_ur5e_mapper", anonymous=True)
        ros1_exists = True
    else:
        ros1_exists = False
    cfg = getArgs()
    command_socket = getCommandClient(cfg)
    wrench_socket = getWrenchClient(cfg)

    # ros subscriber
    if ros1_exists:
        callback_part = partial(command_callback, command_socket)
        rospy.Subscriber("/robot_command", Command, callback_part)
        pub = rospy.Publisher("wrench_msg_mapper", WrenchStamped, queue_size=10)
    else:
        t = Thread(target=sendRandomForTest, cfg=(command_socket,))
        t.run()

    buffer = b""
    while True:
        # receive wrench as protobuf message
        msg_raw = wrench_socket.recv(1024)
        buffer += msg_raw
        if len(msg_raw) < 1:
            continue
        msg_in_bytes, pos = parse_message(buffer)
        buffer = buffer[pos:]

        next_pos, pos = 0, 0
        next_pos, pos = _DecodeVarint32(msg_in_bytes, pos)
        pb2_msg_in_bytes_cut = msg_in_bytes[pos : pos + next_pos]
        pb2_msg = message_specs_pb2.wrench()
        pb2_msg.ParseFromString(pb2_msg_in_bytes_cut)
        print(pb2_msg)

        # send wrench as ros1 message
        if ros1_exists:
            wrench_message = WrenchStamped()
            wrench_message.wrench.force.x = pb2_msg.wrench[0]
            wrench_message.wrench.force.y = pb2_msg.wrench[1]
            wrench_message.wrench.force.z = pb2_msg.wrench[2]
            wrench_message.wrench.torque.x = pb2_msg.wrench[3]
            wrench_message.wrench.torque.y = pb2_msg.wrench[4]
            wrench_message.wrench.torque.z = pb2_msg.wrench[5]
            wrench_message.header.stamp = rospy.Time.now()
            wrench_message.header.frame_id = "0"
            pub.publish(wrench_message)
