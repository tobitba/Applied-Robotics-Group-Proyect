from smc import load_config
from smc import load_config, getRobotFromConfig
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_point_to_point import (
    moveL,
)

import argparse
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt


def get_cfg() -> argparse.Namespace:
    parser = load_config()
    parser.description = "Run closed loop inverse kinematics \
    of various kinds. Make sure you know what the goal is before you run!"
    parser = getClikArgs(parser)
    parser.add_argument(
        "--randomly-generate-goal",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="if true, the target pose is randomly generated, if false you type it target translation in via text input",
    )
    cfg = parser.parse_cfg()
    return cfg


if __name__ == "__main__":
    cfg = get_cfg()
    cfg.plotter = False
    if not cfg.visualizer:
        cfg.ctrl_freq = -1.0
    robot = getRobotFromConfig(cfg)
    # NOTE: default setting when simulating is dt = 1e-3
    # this is too high if we don't want this to last forever
    # in this case thankfully, we can override whatever we want in python -
    # private variables aren't real :)
    robot._dt = 1e-2

    # NOTE: max number of iterations should be set to something low.
    # alternatively, since it's just moveLs, you can kill the function
    # as soon as the commanded velocity and/or change in joint angles
    # is too low.
    # doing this is almost certainly necessary as we want some solid
    # 100 x 100 grid, and thats 1e4 movels! we can't have 0.5 seconds for
    # empty iterations in place...
    cfg.max_iterations = 2000

    # point cloud definition
    height = 0.3
    rotation = pin.rpy.rpyToMatrix(np.array([np.pi, 0.0, 0.0]))
    length = 1.0
    width = 1.0
    N = 10
    xs = np.linspace(-0.5 * length, 0.5 * length, N)
    ys = np.linspace(0.0, width, N)
    workspace_mask = np.zeros((N, N), dtype=np.bool)

    for i in range(len(xs)):
        for j in range(len(xs)):
            robot._q = robot.comfy_configuration
            robot._step()
            print("checking", i * N + j, "out of", N**2)
            translation = np.array([xs[i], ys[j], height])
            T_w_goal = pin.SE3(rotation, translation)
            loop_manager = moveL(cfg, robot, T_w_goal, run=False)
            loop_manager.run()
            print(loop_manager.current_iteration)
            print(cfg.max_iterations)
            print(loop_manager.current_iteration < cfg.max_iterations)
            workspace_mask[i][j] = loop_manager.current_iteration < cfg.max_iterations

    XS, YS = np.meshgrid(xs, ys)
    plt.scatter(XS[workspace_mask], YS[workspace_mask], color="b")
    plt.scatter(
        XS[np.bitwise_not(workspace_mask)],
        YS[np.bitwise_not(workspace_mask)],
        color="r",
    )
    plt.show()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
