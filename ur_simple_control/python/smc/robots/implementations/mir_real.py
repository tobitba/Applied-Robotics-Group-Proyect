from smc.bookkeeping.base_config import ConfigBase
from smc.robots.abstract_robotmanager import (
    AbstractRobotManager,
    AbstractRealRobotManager,
)
from smc.robots.implementations.mir import AbstractMirRobotManager
from smc.control.control_loop_manager import ControlLoopManager

from importlib.util import find_spec

if find_spec("rclpy"):
    import rclpy
    from rclpy.time import Time
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from abb_python_utilities.names import get_rosified_name
    from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import argparse


from smc.control.cartesian_space import getClikArgs
from smc.control.optimal_control.util import get_OCP_cfg
from smc.path_generation.planner import getPlanningArgs


# TODO: load cfg into a config from yaml instead.
# there is no way to use arguments with ros.
# this is massive refactor though so let's not do it now
def get_cfg():
    from smc import load_config

    parser = load_config()
    parser.description = "spinning a node to interact with heron's ros2 topics"
    parser = getClikArgs(parser)
    parser = get_OCP_cfg(parser)
    parser = getPlanningArgs(parser)
    # NOTE: not sure if this is needed on heron
    parser.add_argument(
        "--publish_commands",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="publish or not. useful when you want to 1) verify the connection to the robot, and 2) getting the initial pose to run the preparoty simulation",
    )
    parser.add_argument(
        "--sim",
        action=argparse.BooleanOptionalAction,
        help="if in sim you need to set sim time, otherwise not",
        default=True,
    )
    parser.add_argument("--ros-cfg", action="append", ncfg="*")
    parser.add_argument("-r", action="append", ncfg="*")
    parser.add_argument("-p", action="append", ncfg="*")
    cfg = parser.parse_cfg()
    cfg.robot = "mir"
    cfg.plotter = False
    # cfg.plotter = True
    cfg.real = True
    cfg.ctrl_freq = 20
    # NOTE: does not work due to ctrl-c being overriden
    cfg.save_log = True
    cfg.run_name = "testing"
    cfg.index_runs = True
    return cfg


class RealMirRobotManagerNode(Node, AbstractMirRobotManager, AbstractRealRobotManager):
    def __init__(
        self,
        cfg: ConfigBase,
    ):

        AbstractMirRobotManager.__init__(self, cfg, True)
        super().__init__("RealMirRobotManagerNode")
        self._v_cmd = np.zeros(self.model.nv)
        # NOTE: we're not calling realrobotmanager's super
        # which just calls connectToRobot, connectToGripper, setInitialPose
        # as we don't have a solution for setInitialPose yet
        if cfg.sim:
            self.set_parameters(
                [
                    rclpy.parameter.Parameter(
                        "use_sim_time", rclpy.Parameter.Type.BOOL, True
                    )
                ]
            )
            self.wait_for_sim_time()
        self.cfg = cfg
        self.loop_manager: ControlLoopManager

        # idk what this is
        self._cb = ReentrantCallbackGroup()

        self.get_logger().info("### Starting smc mir node")

        # TODO: read this from node or whatever, the assert is here just in case
        assert self.cfg.ctrl_freq < 50
        self._dt = 1 / self.cfg.ctrl_freq

        self.current_iteration = 0

        self.odom_initialized = False
        self.init_odom = np.zeros(3)
        self.connectToRobot()
        # TODO: figure out how to use setInitial pose and put it here.
        # a hacky but stable solution is good.

        self.get_logger().info(
            "### Connection complete! (created pubs, subs and the rest)"
        )

    # NOTE: required for gazebo simulation to work!
    def wait_for_sim_time(self):
        """Wait for the /clock topic to start publishing."""
        self.get_logger().info("Waiting for simulated time to be active...")
        while not self.get_clock().now().nanoseconds > 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Simulated time is now active!")

    # NOTE: modes here is pointless, but let's have nice uniform code
    def setModesAndLoops(
        self,
        modes_and_loops: list[
            tuple[AbstractRobotManager.control_mode, ControlLoopManager]
        ],
    ):
        """
        setModesAndLoops
        ---------------
        due the fact that the end of the program needs to spin the ros node,
        you have to provide all controllers before the spin call.
        use this function to do so.
        """
        mode, self.loop_manager = modes_and_loops.pop(0)
        self.mode = mode
        self.modes_and_loops = modes_and_loops
        self.get_logger().info(
            "about to run: " + str(self.loop_manager.controlLoop.func.__name__)
        )

    def publishBaseVelocityCommand(self):
        """
        the base takes in velocity commands via the 6D twist message.
        we map to it from _v_cmd as follows:
        0 -> twist.linear.x
        1 -> twist.linear.y
        2 -> twist.angular.z
        """

        if self.cfg.publish_commands:
            msg = Twist()
            # msg.header.stamp = Time().to_msg()
            # msg.twist.twist.linear.x = self._v_cmd[0]
            # msg.twist.twist.linear.y = self._v_cmd[1]
            # msg.twist.twist.angular.z = self._v_cmd[2]
            msg.linear.x = self._v_cmd[0]
            msg.linear.y = self._v_cmd[1]
            msg.angular.z = self._v_cmd[2]

            self._base_vel_cmd_pub.publish(msg)

    # NOTE: while it seems reasonable to put odometry reading here,
    # it's called by _step(), and that there it does not have the message as argument!
    def _updateQ(self):
        pass

    def _updateBasePosition(self, msg: Odometry):
        # NOTE: the Odometry message uses a 3D rotation quarternion,
        # and this does not directly match pinocchio's 2D quarternion.
        # so here we compute the mapping
        ###########################################3333
        # TODO: test: either this is correct
        # costh2 = msg.pose.pose.orientation.w
        # sinth2 = np.linalg.norm(
        #    [
        #        msg.pose.pose.orientation.x,
        #        msg.pose.pose.orientation.y,
        #        msg.pose.pose.orientation.z,
        #    ]
        # )
        # th = 2 * np.arctan2(sinth2, costh2)
        # TODO: test: or this is correct
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
            self.get_logger().info(str(self.init_odom))
        if (
            self.cfg.publish_commands and self.odom_initialized
        ) or self.current_iteration < 50:
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

    # TODO: ideally get this from IMU or something
    def _updateV(self):
        pass

    def connectToRobot(self):
        self._control_timer = self.create_timer(self._dt, self.computeControl)

        # NOTE: use this if you're using amcl localization
        # self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_prof)
        # NOTE: but keep in mind that this is likely useless
        # as it's updating frequency is 0.5Hz!
        # either use the ekf filtering, or odometry, or write your
        # own filter!

        # only to send velocity commands to the base
        # NOTE: I have no idea what this topic is, this is from yuyao's code
        # if cfg.real:
        #    self._base_vel_cmd_pub = self.create_publisher(Twist, "/mir/vel_cmd", 1)
        # else:
        # this topic is in sim
        self._base_vel_cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # NOTE: a the moment, odometry is the only supported localization
        # NOTE: I have no idea what this topic is, this is from yuyao's code
        # if cfg.real:
        #    self.sub_base_odom = self.create_subscription(
        #        Odometry, "/mir/odom", self.callback_base_odom, 1
        #    )
        # else:
        # this topic is in sim
        self.sub_base_odom = self.create_subscription(
            Odometry, "/odom", self._updateBasePosition, 1
        )

    # pointless here
    def connectToGripper(self):
        pass

    def computeControl(self):
        self.current_iteration += 1
        breakFlag = self.loop_manager.run_one_iter(self.loop_manager.current_iteration)
        if breakFlag:
            if len(self.modes_and_loops) > 0:
                mode, self.loop_manager = self.modes_and_loops.pop(0)
                self.mode = mode
                self.get_logger().info(
                    "about to run: " + str(self.loop_manager.controlLoop.func.__name__)
                )
            else:
                self._v_cmd[:] = 0.0
                self.stopRobot()

    # TODO: figure out how to do this with the topics
    # probably need some flags to see whether we got the first message
    # from a topic.
    def setInitialPose(self):
        pass

    # NOTE: look at sendVelocityCommand in AbstractRobotManager to
    # understand relationship between sendVelocityCommand and sendVelocityCommandToReal
    def sendVelocityCommandToReal(self, v):
        # NOTE:
        # 1) the /vel_cmd topic callback will read from _v_cmd
        #    and send the command to the base.
        # 2) since the arm is of the highest control frequency,
        #    we can send that here to separate it from the computeControl logic
        self._v_cmd = v
        if self.cfg.publish_commands:
            if not self.odom_initialized:
                self.get_logger().info(
                    "odom not initialized, hence not publishing anything!"
                )
            else:
                self.publishBaseVelocityCommand()
                self.get_logger().info("command is" + str(self._v_cmd))

    # MASSIVE TODO right here
    def stopRobot(self):
        pass

    # TODO: probably useful to have teleop for this?
    def setFreedrive(self):
        pass

    def unSetFreedrive(self):
        pass
