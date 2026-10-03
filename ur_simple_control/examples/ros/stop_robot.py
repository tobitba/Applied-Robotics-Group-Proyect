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
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from abb_python_utilities.names import get_rosified_name
from rclpy.callback_groups import ReentrantCallbackGroup


class DummyNode(Node):
    def __init__(self):
        super().__init__("dummy_cmds_pub_node")
        # give me the latest thing even if there wasn't an update
        qos_prof = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._cb = ReentrantCallbackGroup()

        self._ns = "myumi_001"

        self.get_logger().info(f"### Starting dummy example under namespace {self._ns}")

        self._cmd_pub = self.create_publisher(
            JointState, f"/{self._ns}/platform/joints_cmd", 1
        )
        self.empty_msg = JointState()
        for i in range(29):
            self.empty_msg.velocity.append(0.0)

        self._dt = 1 / 50

        self._pub_timer = self.create_timer(self._dt, self.send_cmd)

    def send_cmd(self):
        msg = self.empty_msg
        self._cmd_pub.publish(msg)


# def main(cfg=None):
def main(cfg=None):
    rclpy.init(cfg=cfg)

    executor = MultiThreadedExecutor()
    node = DummyNode()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
