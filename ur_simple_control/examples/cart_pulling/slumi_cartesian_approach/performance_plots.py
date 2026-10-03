from smc import load_config
from smc.logging.logger import Logger
from smc.visualization.manipulator_comparison_visualizer import (
    getLogComparisonArgs,
)

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update(
    {
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Computer Modern Roman"],
        "font.size": 10,
    }
)

cfg = getLogComparisonArgs()
log_manager = Logger(None)
log_manager.loadLog(cfg.log_file1)

loop_log = log_manager.loop_logs[
    "3_CartPullingDisjointEEAndBaseCartesianCtrlControlLoop"
]
log_manager.cfg.visualizer = False
log_manager.cfg.plotter = False
log_manager.cfg.draw_new = False


# instantiate figure
# fig = plt.figure()
plt.figure(figsize=(16, 9))
ax_curvature = plt.subplot(511)
ax_curvature.set_xlabel("t/s")
# ax_tracking_error_base = plt.subplot(512)
# ax_tracking_error_base.set_xlabel("t/s")
ax_path_distance = plt.subplot(513)
ax_path_distance.set_xlabel("t/s")
ax_phis = plt.subplot(514)
ax_phis.set_xlabel("t/s")
ax_thetas = plt.subplot(515)
ax_thetas.set_xlabel("t/s")
# ax_joint_limits = plt.subplot(516)
ax_joint_limits = plt.subplot(512)
ax_joint_limits.set_xlabel("t/s")

N = len(loop_log["qs"])
T_final = N / log_manager.cfg.ctrl_freq
t_array = np.linspace(0.0, T_final, N)

# easy stuff first
# ax_manip.plot(t_array, loop_log["manipulability"], label="manipulability")
# ebp = loop_log["ebd_dist"]
# ebp[:, 0] = -log_manager.cfg.base_to_cart_arclength - ebp[:, 0]
## ref was 0,0 for y and theta
# ebp[:, 1:3] = -1 * ebp[:, 1:3]
# ax_path_distance.plot(
#    t_array, ebp, label=["B-E error x [m]", "B-E error y[m]", "B-E error theta [rad]"]
# )
# ee_norm_left = np.linalg.norm(loop_log["ee_pose_error"][:, :6], axis=1)
# ee_norm_right = np.linalg.norm(loop_log["ee_pose_error"][:, 6:], axis=1)
# ax_phis.plot(
#   t_array,
#   np.hstack((ee_norm_left.reshape((-1, 1)), ee_norm_right.reshape((-1, 1)))),
#   label=["E_l pose error L_2 norm", "E_r pose error L_2 norm"],
# )
ax_phis.plot(
    t_array,
    loop_log["phis"][:, 0],
    label=r"$\phi$ achieved",
)
ax_phis.plot(
    t_array,
    loop_log["phis"][:, 1],
    label=r"$\phi$ requested",
)


ax_curvature.plot(t_array, loop_log["path_curvature"], label="path curvature")

# ax_tracking_error_base.plot(
#    t_array, loop_log["base_position_error"], label="base position error [m]"
# )

ax_joint_limits.plot(
    t_array, loop_log["closes_joint_limit"], label="closest joint limit [rad]"
)

ax_thetas.plot(t_array, loop_log["headings"][:, 0], label=r"$\theta$ achieved")
ax_thetas.plot(t_array, loop_log["headings"][:, 1], label=r"$\theta$ requested")

ax_path_distance.plot(
    t_array, loop_log["cart_path_distance"], label="cart-path distance"
)

ax_curvature.legend()
ax_curvature.grid()
# ax_tracking_error_base.legend()
# ax_tracking_error_base.grid()
ax_phis.legend()
ax_phis.grid()
ax_path_distance.legend()
ax_path_distance.grid()
ax_thetas.legend()
ax_thetas.grid()
ax_joint_limits.legend()
ax_thetas.grid()

plt.savefig("./plots/curvature_vs_errors.pdf", dpi=300, bbox_inches="tight")
plt.show()
