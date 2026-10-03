from smc import load_config
#!/usr/bin/env python3
#
# Copyright (c) 2024, ABB Schweiz AG
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from abb_python_utilities.names import get_rosified_name
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import argparse
from smc.robots.implementations.mobile_yumi import RealMobileYumiRobotManager
from smc import load_config
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.cartesian_space import getClikArgs, getIKSolver
from smc.control.joint_space.joint_space_point_to_point import moveJP
from smc.control.cartesian_space.cartesian_space_point_to_point import (
    controlLoopClik,
    moveL,
    moveLDualArm,
    moveL,
)
from smc.control.optimal_control.croco_point_to_point.mpc.base_reference_mpc import (
    CrocoBaseP2PMPC,
)
from smc.control.optimal_control.util import get_OCP_cfg
from smc.robots.utils import getRobotFromConfig
import pinocchio as pin

import numpy as np


# def main(cfg=None):
def main(cfg=None):
    # evil and makes cfg unusable but what can you do
    cfg_smc = get_cfg()
    assert cfg_smc.robot == "myumi"
    robot = RealMobileYumiRobotManager(cfg_smc)
    # you can't do terminal input with ros
    # goal = robot.defineGoalPointCLI()
    T_w_absgoal = pin.SE3.Identity()
    T_w_absgoal.translation = np.array([0.5, 0.0, 0.5])
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation = np.array([0.0, 0.2, 0.0])
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation = np.array([0.0, -0.2, 0.0])
    # loop_manager = CrocoIKMPC(cfg, robot, goal, run=False)
    # TODO: put all loops and corresponding modes in this list
    # then in cmd_pub callback go to the next loop
    # once you get a breakFlag == True
    loops_n_modes = []
    mode = robot.control_mode.upper_body
    loop_manager = moveJP(robot._comfy_configuration, cfg_smc, robot, run=False)
    loops_n_modes.append(mode, loop_manager)
    loop_manager = moveLDualArm(
        cfg_smc, robot, T_w_absgoal, T_absgoal_l, T_absgoal_r, run=False
    )
    loops_n_modes.append(mode, loop_manager)
    # robot._mode = robot.control_mode.whole_body
    # loop_manager = CrocoBaseP2PMPC(cfg_smc, robot, np.zeros(3), run=False)

    rclpy.init(cfg=cfg)

    executor = MultiThreadedExecutor()
    node = DummyNode(cfg_smc, robot, loop_manager)
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
