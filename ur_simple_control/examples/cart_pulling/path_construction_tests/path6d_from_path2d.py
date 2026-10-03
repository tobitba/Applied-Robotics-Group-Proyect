from smc import load_config
from smc import getRobotFromConfig
from smc import load_config
from smc.control.cartesian_space import getClikArgs
from smc.path_generation.planner import getPlanningArgs
from smc.control.optimal_control.croco_mpc_path_following import initializePastData
from smc.path_generation.path_math.path2d_to_6d import path2D_to_SE3
from smc.path_generation.path_math.path_to_trajectory import path2D_to_trajectory2D
from smc.control.optimal_control.util import get_OCP_cfg

from smc.path_generation.path_math.cart_pulling_path_math import (
    time_base_path,
    construct_EE_path,
)

import yaml
import numpy as np
from functools import partial
import numpy as np
import time


def get_cfg():
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getPlanningArgs(parser)
    parser.add_argument(
        "--handlebar-height",
        type=float,
        default=0.5,
        help="heigh of handlebar of the cart to be pulled",
    )
    parser.add_argument(
        "--base-to-handlebar-preferred-distance",
        type=float,
        default=0.5,
        help="prefered path arclength from mobile base position to handlebar",
    )
    cfg = parser.parse_cfg()
    return cfg


cfg = get_cfg()
cfg.past_window_size = 100
cfg.robot = "heron"
robot = getRobotFromConfig(cfg)
robot._step()
robot.visualizer_manager.sendCommand({"q": robot.q})
input("press Enter to continue")
print(robot.q)

max_base_v = np.linalg.norm(robot._max_v[:2])
dt = 1 / cfg.ctrl_freq

path2D_handlebar = initializePastData(
    cfg, robot.T_w_e, robot.base_position2D, max_base_v
)
# path2D_handlebar = np.linspace(0, 1, cfg.past_window_size).reshape((-1, 1))
# path2D_handlebar = np.hstack((path2D_handlebar, path2D_handlebar))
print(path2D_handlebar.shape)
path3D_handlebar = np.hstack(
    (path2D_handlebar, np.ones((len(path2D_handlebar), 1)) * robot.T_w_e.translation[2])
)
# print(path3D_handlebar.shape)
# robot.visualizer_manager.sendCommand({"path": path2D_handlebar})
robot.visualizer_manager.sendCommand({"path": path3D_handlebar})
print(
    "we initialize path ee poses with this 2D path (z axis adjusted just for visual clarity"
)
print(path3D_handlebar)
# print(path2D_handlebar)
input("press Enter to continue")

# path you already traversed
# time_past = np.linspace(0.0, cfg.past_window_size * dt, cfg.past_window_size)
x = np.linspace(0.0, cfg.past_window_size * dt, cfg.past_window_size)
# x = np.linspace(0.0, 2.0, 200)
x = x.reshape((-1, 1))
y = np.sin(x)
past_data = {}
past_data["path2D_untimed"] = np.hstack((x, y))

# path you get from path planner
x = np.linspace(0.0, cfg.past_window_size * dt, cfg.past_window_size)
# x = np.linspace(2.0, 4.0, 200)
x = x.reshape((-1, 1))
y = np.sin(x)
path2D_untimed_base = np.hstack((x, y))

p = path2D_untimed_base[-1]
path2D_untimed_base = np.array(path2D_untimed_base).reshape((-1, 2))

path_base = time_base_path(cfg, path2D_untimed_base, max_base_v)
print(path_base)
pathSE3_handlebar = construct_EE_path(cfg, p, past_data["path2D_untimed"])

for i in range(100):
    visualizer_manager.sendCommand({"frame_path": pathSE3_handlebar})
    # visualizer_manager.sendCommand({"frame_path": some_path})
    visualizer_manager.sendCommand({"path": path_base})
    time.sleep(1)
print("send em")

time.sleep(10)
visualizer_manager.terminateProcess()
