from smc.robots.abstract_robotmanager import (
    AbstractRealRobotManager,
    AbstractRobotManager,
)
from smc.robots.implementations.heron import AbstractHeronRobotManager
from smc.robots.grippers.robotiq.robotiq_gripper import RobotiqGripper
from smc.robots.grippers.rs485_robotiq.rs485_robotiq import RobotiqHand


from smc.control.control_loop_manager import ControlLoopManager

import numpy as np
import argparse
import pinocchio as pin
import time

# NOTE: do not load any ros stuff if it isn't installed
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

    if not find_spec("rtde_control"):
        raise ImportError(
            "despite using ros to control the base, the arm is still controlled with ur_rtde instead!"
        )
    else:
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
        from rtde_io import RTDEIOInterface


# NOTE: ros eats all arguments, so your nice arguments = config
# desing does not work. this function is a temporary workaround.
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
    parser.add_argument(
        "--base-ctrl-freq",
        action=argparse.BooleanOptionalAction,
        help="the control frequency of the base",
        default=30,
    )
    parser.add_argument("--ros-cfg", action="append", ncfg="*")
    parser.add_argument("-r", action="append", ncfg="*")
    parser.add_argument("-p", action="append", ncfg="*")
    cfg = parser.parse_cfg()
    cfg.robot = "heron"
    cfg.plotter = False
    # cfg.plotter = True
    cfg.real = True
    cfg.ctrl_freq = 500
    # NOTE: does not work due to ctrl-c being overriden
    cfg.save_log = True
    cfg.run_name = "testing"
    cfg.index_runs = True
    return cfg


class RealHeronRobotManagerNode(
    Node, AbstractHeronRobotManager, AbstractRealRobotManager
):
    def __init__(
        self,
        cfg: MasterConfig,
    ):
        self._wrench_base: np.ndarray = np.zeros(6)
        self._wrench: np.ndarray = np.zeros(6)
        # NOTE: wrench bias will be defined in the frame your sensor's gives readings
        self._wrench_bias: np.ndarray = np.zeros(6)
        self._T_w_e = pin.SE3.Identity()
        self.speed_slider = 1.0  # don't touch this, it messes up speedJ

        AbstractHeronRobotManager.__init__(self, cfg, True)
        # NOTE: we store the computed control signal here
        # it is then read and published by ros topics.
        # this is the thinnest possible hack to bridge
        # SMC and ROS API.
        self._v_cmd = np.zeros(self.model.nv)
        # NOTE: we're not calling realrobotmanager's super
        # which just calls connectToRobot, connectToGripper, setInitialPose
        # as we don't have a solution for setInitialPose yet

        super().__init__("RealHeronRobotManagerNode")

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

        # NOTE: QoSProfile might be required for some topics
        # the settings are to give the latest thing even if there wasn't an update:
        # qos_prof = rclpy.qos.QoSProfile(
        #    reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        #    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        #    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        #    depth=1,
        # )

        # NOTE: i have absolutely no idea what this is
        self._cb = ReentrantCallbackGroup()

        self.get_logger().info(f"### Starting smc heron node")
        # NOTE: use ur_rtde instead of ROS to control the arm
        # (already works with SMC, why fix it if it isn't broken)
        self._rtde_control: RTDEControlInterface
        self._rtde_receive: RTDEReceiveInterface
        self._rtde_io: RTDEIOInterface

        self._dt = 1 / self.cfg.ctrl_freq
        # TODO: either read from argument, or get Hz from ROS
        self._base_cmd_dt = 1 / 20

        self.current_iteration = 0

        self.odom_initialized = False
        self.init_odom = np.zeros(3)
        self.connectToGripper()
        self.connectToRobot()
        # TODO: figure out how to use setInitial pose and put it here.
        # a hacky but stable solution is good.

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

    def publishBaseVelocityCommand(self):
        """
        the base takes in velocity commands via the 6D twist message.
        we map to it from _v_cmd as follows:
        0 -> twist.linear.x
        1 -> twist.linear.y
        2 -> twist.angular.z
        """

        if self.cfg.publish_commands:
            if not self.odom_initialized:
                self.get_logger().info(
                    "odom not initialized, hence not publishing anything!"
                )
            else:
                msg = Twist()
                # NOTE: whatever ros we're on on heron
                # does not have headers!
                # msg.header.stamp = Time().to_msg()
                if self._mode != AbstractRobotManager.control_mode.upper_body:
                    # msg.twist.twist.linear.x = self._v_cmd[0]
                    # msg.twist.twist.linear.y = self._v_cmd[1]
                    # msg.twist.twist.angular.z = self._v_cmd[2]
                    msg.linear.x = self._v_cmd[0]
                    msg.linear.y = self._v_cmd[1]
                    msg.angular.z = self._v_cmd[2]
                else:
                    # NOTE: this way we ensure that it's not following
                    # last velocity command which might be different than zero!
                    # msg.twist.twist.linear.x = 0.0
                    # msg.twist.twist.linear.y = 0.0
                    # msg.twist.twist.angular.z = 0.0
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.angular.z = 0.0

                self._base_vel_cmd_pub.publish(msg)

    def callback_base_odom(self, msg: Odometry):
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

    def setSpeedSlider(self, value):
        """
        setSpeedSlider
        ---------------
        update in all places
        NOTE: THIS IS EVIL AND NOTHING WORKS UNLESS IT'S SET TO 1.0!!!
                USE AT YOUR PERIL IF YOU DON'T KNOW WHAT IT DOES (i don't)
        """
        assert value <= 1.0 and value > 0.0
        # NOTE: can't use rtde_io on heron apparetnly.
        # you have to manually check the value of speed slider.
        #        if self.cfg.real:
        #            self._rtde_io.setSpeedSlider(value)
        self.speed_slider = value

    def _updateQ(self):
        q = self._rtde_receive.getActualQ()
        self._q[4:] = np.array(q)

    def _updateV(self):
        v = self._rtde_receive.getActualQd()
        self._qd[3:] = np.array(v)

    # NOTE: required for gazebo simulation to work!
    def wait_for_sim_time(self):
        """Wait for the /clock topic to start publishing."""
        self.get_logger().info("Waiting for simulated time to be active...")
        while not self.get_clock().now().nanoseconds > 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Simulated time is now active!")

    def connectToRobot(self):
        # NOTE: use ur_rtde instead of ROS to control the arm
        # (already works with SMC, why fix it if it isn't broken)
        # NOTE: you can't connect twice, so you can't have more than one
        # RobotManager per robot.  if this produces errors like "already in
        # use", and it's not already in use, try just running your new program
        # again. it could be that the socket wasn't given back to the os even
        # though you've shut off the previous program.
        self._rtde_control = RTDEControlInterface(self.cfg.robot_ip)
        self._rtde_receive = RTDEReceiveInterface(self.cfg.robot_ip)
        # NOTE: refuses to work on heron for some reason.
        # probably some default was overriden.
        # thankfully we don't need it
        # self._rtde_io = RTDEIOInterface(self.cfg.robot_ip)
        assert self.speed_slider == 1.0
        # self._rtde_io.setSpeedSlider(self.cfg.speed_slider)

        # NOTE: the force/torque sensor just has large offsets for no reason,
        # and you need to minus them to have usable readings.
        # we provide this with calibrateFT
        self.wrench_offset = self.calibrateFT(self._dt)

        # NOTE: computeControl is to be run at the HIGHEST control frequency
        # out of the available things to control.
        # computeControl() will save the control command,
        # and publish to what has the highest frequency (arm in this case).
        # other publishers will read the stored command
        # and publish it  when they can (base command in this case)
        self._control_timer = self.create_timer(self._dt, self.computeControl)

        # only to send velocity commands to the base
        # NOTE: I have no idea what this topic is, this is from yuyao's code
        # if cfg.real:
        #    self._base_vel_cmd_pub = self.create_publisher(Twist, "/mir/vel_cmd", 1)
        # else:
        # this topic is in sim
        self._base_vel_cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self._base_velocity_command_timer = self.create_timer(
            self._base_cmd_dt, self.publishBaseVelocityCommand
        )

        # NOTE: use this if you're using amcl localization
        # self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_prof)
        # NOTE: but keep in mind that this is likely useless
        # as it's updating frequency is 0.5Hz!
        # either use the ekf filtering, or odometry, or write your
        # own filter!

        # NOTE: a the moment, odometry is the only supported localization
        # NOTE: I have no idea what this topic is, this is from yuyao's code
        # if cfg.real:
        #    self.sub_base_odom = self.create_subscription(
        #        Odometry, "/mir/odom", self.callback_base_odom, 1
        #    )
        # else:
        # this topic is in sim
        self.sub_base_odom = self.create_subscription(
            Odometry, "/odom", self.callback_base_odom, 1
        )

    def connectToGripper(self):
        # NOTE: in this case, the ip should be the arm's IP!
        if (self.cfg.gripper == "none") or not self.cfg.real:
            self.gripper = None
            return
        if self.cfg.gripper == "robotiq":
            self.gripper = RobotiqGripper()
            self.gripper.connect(self.cfg.robot_ip, 63352)
            self.gripper.activate()
        if self.cfg.gripper == "rs485":
            self.gripper = RobotiqHand()
            self.gripper.connect(self.cfg.robot_ip, 54321)
            self.gripper.reset()
            self.gripper.activate()
            result = self.gripper.wait_activate_complete()
            if result != 0x31:
                print("ERROR: can't activate gripper!! - exiting")
                self.gripper.disconnect()
                exit()

    # TODO: figure out how to do this with the topics
    # probably need some flags to see whether we got the first message
    # from a topic.
    def setInitialPose(self):
        pass

    def _updateWrench(self):
        if not self.cfg.real:
            self._wrench_base = np.random.random(6)
        else:
            # NOTE: UR5e's ft-sensors gives readings in robot's base frame
            self._wrench_base = (
                np.array(self._rtde_receive.getActualTCPForce()) - self._wrench_bias
            )
        # NOTE: we define the default wrench to be given in the end-effector frame
        mapping = np.zeros((6, 6))
        mapping[0:3, 0:3] = self._T_w_e.rotation
        mapping[3:6, 3:6] = self._T_w_e.rotation
        self._wrench = mapping.T @ self._wrench_base

    def zeroFtSensor(self):
        self._rtde_control.zeroFtSensor()

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
            if self._mode == AbstractRobotManager.control_mode.whole_body:
                self._rtde_control.speedJ(self._v_cmd[3:], self._acceleration, self._dt)
            if self._mode == AbstractRobotManager.control_mode.upper_body:
                self._rtde_control.speedJ(self._v_cmd, self._acceleration, self._dt)
            # NOTE: this is done by the base velocity command publisher,
            # kept here just as a reminder that this mode exists
            if self._mode == AbstractRobotManager.control_mode.base_only:
                pass
            # NOTE: in addition, we should update the base position
            # in-between odometry readings because more correct
            # than not updating (smoother localization, smoother control)
            # NOTE: we're not actually measuring the velocity of
            # the base tho! so this doesn't exactly make sense
            # self._q = pin.integrate(self.model, self.q, v * self._dt)

    def stopRobot(self):
        self._rtde_control.speedStop(1)
        print("sending a stopj as well")
        self._rtde_control.stopJ(1)
        print("putting it to freedrive for good measure too")
        print("stopping via freedrive lel")
        self._rtde_control.freedriveMode()
        time.sleep(0.5)
        self._rtde_control.endFreedriveMode()
        # TODO: we need to stop be the base as well.
        # option 1) send zero velocity commands.
        # but then make sure that it doesn't keep going forward
        # with a near-zero velocity (this happens on UR5e, that's why
        # the freedrive is started because it actually stops the arm)
        # option 2) programaticaly activate the emergency button
        # option 3) stop just the arm, do nothing for the base, and
        # clearly document that the robot has to be stopped by manually
        # pressing the emergency button

    def setFreedrive(self):
        self._rtde_control.freedriveMode()
        # TODO: if it's possible to manually push the base, great,
        # put that option here. if not, remove the above error throw,
        # document that there's no freedrive for the base here
        # and just put the arm to freedrive mode

    def unSetFreedrive(self):
        self._rtde_control.endFreedriveMode()
        # TODO: if it's possible to manually push the base, great,
        # put that option here. if not, remove the above error throw,
        # document that there's no freedrive for the base here
        # and just put the arm to freedrive mode
