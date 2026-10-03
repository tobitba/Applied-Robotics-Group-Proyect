import argparse
import message_specs_pb2
from smc_to_translator_utils import (
    connectToClient,
    WrenchMsgToList,
    TwistMsgToList,
    sendProtobuf,
    receivingAndPuttingToQueue,
)
from multiprocessing import Queue
from threading import Thread
import numpy as np

from importlib.util import find_spec

if find_spec("rospy"):
    # TODO: import messages we're reading
    import rospy
    from geometry_msgs.msg import Pose, PoseStamped, Wrench, WrenchStamped, Twist
    from sensor_msgs.msg import JointState
    from abb_egm_msgs.msg import EGMState
    from std_msgs.msg import Float64MultiArray
    from nav_msgs.msg import Odometry
else:
    raise ImportError("you are supposed to run this is computer with ros1! exiting")


def getArgs():
    parser = argparse.ArgumentParser()
    parser.description = "be a translator between SMC and ROS1. intended to run in a docker container on computer running SMC. network topology is SMC-translation_layer-Robot.\n \
    receive ros1 message, make it a protobuf message, send it via tcp socket. get protobuf message, make it a ros1 message, send it via topic"
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
    # TODO: unusued at the moment
    parser.add_argument(
        "--use-wrenches",
        # NOTE: BooleanOptionalAction added in python 3.9,
        # and we have to run this in 3.8
        # action=argparse.BooleanOptionalAction,
        help="read and send wrench commands, yes or no",
        default=False,
    )
    parser.add_argument(
        "--send-commands",
        # NOTE: BooleanOptionalAction added in python 3.9,
        # and we have to run this in 3.8
        # action=argparse.BooleanOptionalAction,
        help="do plan to send commands, or just read?",
        default=False,
    )
    parser.add_argument(
        "--sending-frequency",
        type=int,
        help="recv/send freq for testing",
        default=250,
    )

    cfg = parser.parse_args()
    return cfg


class SLuMiTranslator:
    def __init__(
        self,
        cfg: argparse.Namespace,
        topic_egm_state: str,
        topic_joint_state: str,
        topic_sensor_l: str,
        topic_sensor_r: str,
        topic_odometry: str,
        topic_arms_velocity_command: str,
        topic_base_velocity_command: str,
    ):
        self.urdf_joint_names = [
            "yumi_joint_1_l",
            "yumi_joint_2_l",
            "yumi_joint_7_l",
            "yumi_joint_3_l",
            "yumi_joint_4_l",
            "yumi_joint_5_l",
            "yumi_joint_6_l",
            "yumi_joint_1_r",
            "yumi_joint_2_r",
            "yumi_joint_7_r",
            "yumi_joint_3_r",
            "yumi_joint_4_r",
            "yumi_joint_5_r",
            "yumi_joint_6_r",
        ]
        urdf_name_to_id_map = {key: i for i, key in enumerate(self.urdf_joint_names)}
        # ATTENTION: this naming is inconsistent with the naming found in ABB driver!
        #            joint_7 is between joint_2 and joint_3, but still refers
        #            to the third joint in the chain. Also, left and right are
        #            switched, and the names have different prefixes in general...
        self.egm_joint_names = [
            "yumi_robr_joint_1",
            "yumi_robr_joint_2",
            "yumi_robr_joint_3",
            "yumi_robr_joint_4",
            "yumi_robr_joint_5",
            "yumi_robr_joint_6",
            "yumi_robr_joint_7",
            "yumi_robl_joint_1",
            "yumi_robl_joint_2",
            "yumi_robl_joint_3",
            "yumi_robl_joint_4",
            "yumi_robl_joint_5",
            "yumi_robl_joint_6",
            "yumi_robl_joint_7",
        ]
        egm_name_to_id_map = {key: i for i, key in enumerate(self.egm_joint_names)}
        self._urdf_to_egm_joint_names_map = {
            urdf_name: egm_name
            for urdf_name, egm_name in zip(
                self.urdf_joint_names[7:14], self.egm_joint_names[0:7]
            )
        }
        self._urdf_to_egm_joint_names_map.update(
            {
                urdf_name: egm_name
                for urdf_name, egm_name in zip(
                    self.urdf_joint_names[0:7], self.egm_joint_names[7:14]
                )
            }
        )
        self.urdf_id_to_egm_id_map = {
            urdf_name_to_id_map[key]: egm_name_to_id_map[
                self._urdf_to_egm_joint_names_map[key]
            ]
            for key in self.urdf_joint_names
        }
        # print(self.urdf_id_to_egm_id_map)
        self._urdf_joint_ids_map = {
            urdf_name: i for i, urdf_name in enumerate(self.urdf_joint_names)
        }
        # TODO: expand this translator to work for
        # both yumi and slumi.
        # there's no point in having 2?
        # or is there? idk, figure it out boss, make a decision,
        # and follow through
        self._q: list[float] = [0.0] * 18
        self._q[2] = 1.0
        self._dq: list[float] = [0.0] * 17
        self._v_cmd: list[float] = [0.0] * 17
        self._wrench_l: list[float] = [0.0] * 6
        self._wrench_r: list[float] = [0.0] * 6
        self._egm_active: bool = False
        self.init_odom = np.zeros(3)
        self.odom_initialized = False
        self.sendProtobufSuccess = True
        self.recvProtobufSuccess = True
        self.n_zeros_to_send = 100

        self.cfg = cfg

        # connect to SMC computer
        return_t1 = {}
        t1 = Thread(
            target=connectToClient,
            cfg=("SERVER_COMMAND_RECEIVER", cfg.host, cfg.port_command, return_t1),
        )
        return_t2 = {}
        t2 = Thread(
            target=connectToClient,
            cfg=(
                "SERVER_JOINT_POSITIONS_SENDER",
                cfg.host,
                cfg.port_data,
                return_t2,
            ),
        )

        if self.cfg.send_commands:
            t1.start()
        t2.start()
        t2.join()
        if len(return_t2) == 0:
            print(
                "connecting to client failed. exiting. \n cause is almost certaintly that the address is already in use. kill the script, or select a different port"
            )
        self.data_sending_socket = return_t2["comm_socket"]

        if self.cfg.send_commands:
            t1.join()
            self.command_receiving_socket = return_t1["comm_socket"]
            if len(return_t1) == 0:
                print(
                    "connecting to client failed. exiting. \n cause is almost certaintly that the address is already in use. kill the script, or select a different port"
                )
            self.recv_queue = Queue(2)
            self.t_v_cmd_receiver = Thread(
                target=receivingAndPuttingToQueue,
                cfg=(
                    message_specs_pb2.v_cmd,
                    self.command_receiving_socket,
                    self.recv_queue,
                ),
            )
            self.t_v_cmd_receiver.start()
        else:
            self.n_zeros_to_send = 0

        # subscribe to joint_states
        self.subscriber_joint_state = rospy.Subscriber(
            topic_joint_state,
            JointState,
            self.callback_joint_state,
            queue_size=1,
            tcp_nodelay=False,
        )

        # subscribe to odometry
        self.sub_base_odom = rospy.Subscriber(
            topic_odometry,
            Odometry,
            self.callback_odometry,
            queue_size=1,
            tcp_nodelay=False,
        )

        # create arms velocity command publisher
        self.publisher_v_cmd_arms = rospy.Publisher(
            topic_arms_velocity_command,
            Float64MultiArray,
            queue_size=1,
            tcp_nodelay=True,
        )

        # create base velocity command publisher
        self.publisher_v_cmd_base = rospy.Publisher(
            topic_base_velocity_command,
            Twist,
            queue_size=1,
            tcp_nodelay=True,
        )

    # NOTE: not used atm. we just assume egm is started and works
    def callback_egm_state(self, msg: EGMState):
        self._egm_active = msg.egm_channels[0].active and msg.egm_channels[1].active

    def callback_ext_force(self, data: WrenchStamped, arm: str):
        if arm == "right":
            self._wrench_l = WrenchMsgToList(data.wrench)
        elif arm == "left":
            self._wrenche_r = WrenchMsgToList(data.wrench)

    def callback_joint_state(self, msg: JointState):
        # `msg.name` contains `egm_joint_names`
        # Pinocchio contains `urdf_joint_names`
        egm_values = {
            egm_name: (pos, vel)
            for egm_name, pos, vel in zip(msg.name, msg.position, msg.velocity)
        }
        for urdf_name in self.urdf_joint_names:
            egm_name = self._urdf_to_egm_joint_names_map.get(urdf_name, None)
            i = self._urdf_joint_ids_map.get(urdf_name, None)
            self._q[i + 4] = egm_values[egm_name][0]
            self._dq[i + 3] = egm_values[egm_name][1]

        # NOTE: for now, let's just send point positions.
        # and make sure things work as expected.
        # then we can add the velocities as well.
        # and then we can add the wrenches.
        # it should all work without hickups, but let's
        # do it one step at a time.
        pb2_msg = message_specs_pb2.joint_positions()
        pb2_msg.q.extend(self._q)
        # NOTE: main checks the flag and exits if appropriate
        if self.sendProtobufSuccess:
            self.sendProtobufSuccess = sendProtobuf(pb2_msg, self.data_sending_socket)

    def callback_velocity_commands(self):
        if not self.cfg.send_commands:
            return
        if not self.odom_initialized:
            print("odom not initialized, i won't publish anythin until it is")
            return
        self.until_pub_base = 0
        # blocking call is good. don't send if you don't have anything to send
        # completely pointless for loop lol
        if self.recvProtobufSuccess:
            pb2_msg = self.recv_queue.get()
            if pb2_msg is None:
                print(
                    "MAIN: ros1 node got None message. sending zero v_cmd then exiting"
                )
                self.recvProtobufSuccess = False
            else:
                for i, val in enumerate(pb2_msg.v_cmd):
                    self._v_cmd[i] = val
        else:
            for i in range(len(self._v_cmd)):
                self._v_cmd[i] = 0.0
            self.n_zeros_to_send -= 1

        ros1msg = Float64MultiArray()
        ros1msg.data = self._v_cmd[3:]
        self.publisher_v_cmd_arms.publish(ros1msg)

        # TODO: set self-limiting on sending base velocity commands if required
        # (i'm assuming the max possible rate is smaller than for the arms)
        if self.until_pub_base == 0:
            self.until_pub_base = 0
            twistmsg = Twist()
            twistmsg.linear.x = self._v_cmd[0]
            twistmsg.linear.y = self._v_cmd[1]
            twistmsg.angular.z = self._v_cmd[2]
            self.publisher_v_cmd_base.publish(twistmsg)
        self.until_pub_base += 1
        if self.n_zeros_to_send == 0:
            exit()

    # TODO publish base commands at  teleop topic
    def callback_odometry(self, msg: Odometry):
        # NOTE: the Odometry message uses a 3D rotation quarternion,
        # and this does not directly match pinocchio's 2D quarternion.
        # so here we compute the mapping
        q = msg.pose.pose.orientation
        th = -2 * np.arctan2(q.z, q.w)
        ###########################################3333

        # NOTE: after the robot has started, it runs the
        # same odometry topic. this means that when you run
        # your control any time except the first,
        # you'll get non-zero odometry readings.
        # this can be really bad as we normally assume (0,0)
        # in our controllers when starting and using odometry.
        # so here we zero out odometry readings based on the first one.
        # current_iteration < 50 is a hack to make sure
        # everything else started and initialized.
        if not self.odom_initialized:
            self.init_odom[0] = msg.pose.pose.position.x
            self.init_odom[1] = msg.pose.pose.position.y
            self.init_odom[2] = th
            self.odom_initialized = True
            print(self.init_odom)
        else:
            T_odom = np.zeros((3, 3))
            T_odom[0, 0] = np.cos(self.init_odom[2])
            T_odom[0, 1] = -1 * np.sin(self.init_odom[2])
            T_odom[0, 2] = self.init_odom[0]
            T_odom[1, 0] = np.sin(self.init_odom[2])
            T_odom[1, 1] = np.cos(self.init_odom[2])
            T_odom[1, 2] = self.init_odom[1]
            T_odom[2, 2] = 1.0
            p_odom = np.array(
                [
                    msg.pose.pose.position.x - self.init_odom[0],
                    msg.pose.pose.position.y - self.init_odom[1],
                ]
            )
            p_ctrl = T_odom[:2, :2] @ p_odom
            self._q[0] = p_ctrl[0]
            self._q[1] = p_ctrl[1]
            self._q[2] = np.cos(self.init_odom[2] - th)
            self._q[3] = np.sin(self.init_odom[2] - th)


if __name__ == "__main__":

    # starting ROS node
    rospy.init_node("slumi_translator", anonymous=False)

    topic_egm_state = "/yumi/egm/egm_states"
    topic_joint_state = "/yumi/egm/joint_states"
    topic_sensor_r = "/ftsensor_r/world"
    topic_sensor_l = "/ftsensor_l/world"
    topic_odometry = "/base/odometry_controller/odometry"
    topic_arms_velocity_command = "/yumi/egm/joint_group_velocity_controller/command"
    topic_base_velocity_command = "/base/twist_mux/command_teleop_keyboard"

    cfg = getArgs()
    translator = SLuMiTranslator(
        cfg,
        topic_egm_state,
        topic_joint_state,
        topic_sensor_r,
        topic_sensor_l,
        topic_odometry,
        topic_arms_velocity_command,
        topic_base_velocity_command,
    )

    # TODO publish base commands at  teleop topic
    rate = rospy.Rate(cfg.sending_frequency)
    while not rospy.is_shutdown():
        # init = rospy.Time.now()
        # NOTE: also sends base commands because in ROS1 we set only one timer
        translator.callback_velocity_commands()
        if (not translator.sendProtobufSuccess) and (translator.n_zeros_to_send <= 0):
            print("MAIN: exiting")
            exit()
        # print(1/(rospy.Time.now() - init).to_sec())
        rate.sleep()
