from smc import load_config
import socket
from google.protobuf.internal.encoder import _VarintBytes
import argparse
import message_specs_pb2
import random
import importlib.util
import time
from functools import partial

if importlib.util.find_spec("rospy"):
    from agents_test.msg import Command
    import rospy


def getArgs():
    parser = argparse.ArgumentParser()
    parser.description = (
        "get ros1 message, make it a protobuf message, send it via tcp socket"
    )
    parser.add_argument("--host", type=str, help="host ip address", default="127.0.0.1")
    parser.add_argument("--port", type=int, help="host's port", default=7777)

    cfg = parser.parse_cfg()
    return cfg


def callback(comm_socket, c):
    # print("received callback")
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
    # print(pb2_msg)
    msg_length = pb2_msg.ByteSize()
    msg_serialized = pb2_msg.SerializeToString()
    msg = _VarintBytes(msg_length) + msg_serialized
    comm_socket.send(msg)


def sendRandomForTest(comm_socket):
    pb2_msg = message_specs_pb2.T_goal()
    pb2_msg.position.extend([random.random()] * 3)
    pb2_msg.rotation.extend([random.random()] * 4)
    pb2_msg.velocity.extend([random.random()] * 6)
    print(pb2_msg)
    msg_length = pb2_msg.ByteSize()
    msg_serialized = pb2_msg.SerializeToString()
    msg = _VarintBytes(msg_length) + msg_serialized
    comm_socket.send(msg)
    # time.sleep(0.002)
    time.sleep(0.02)


if __name__ == "__main__":
    if importlib.util.find_spec("rospy"):
        rospy.init_node("ros1_to_ur5e_mapper", anonymous=True)
        ros1_exists = True
    else:
        ros1_exists = False
    cfg = getArgs()
    host_addr = (cfg.host, cfg.port)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(host_addr)

    print("listening on port ", cfg.port)
    s.listen()
    comm_socket, comm_addr = s.accept()
    # we're only accepting a single connection
    s.close()
    print("NETWORKING_SERVER: accepted a client", comm_addr)

    # ros subscriber
    if ros1_exists:
        callback_part = partial(callback, comm_socket)
        rospy.Subscriber("/robot_command", Command, callback_part)
        rospy.spin()
    else:
        while True:
            sendRandomForTest(comm_socket)

    comm_socket.close()


# try:
# 	while True:
# 		# listen to a pre-determined ros1 message here
# except KeyboardInterrupt:
# 	if cfg.debug_prints:
# 		print("NETWORKING_SERVER: caugth KeyboardInterrupt, networking server out")
#
