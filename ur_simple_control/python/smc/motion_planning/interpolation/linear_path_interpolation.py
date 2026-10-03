from smc.bookkeeping.load_config import GlobalConfig
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
from smc.bookkeeping.types import (
    JointReference,
    CartesianReference,
    ReferenceT,
    PathPointT,
    PathPointsT,
)

import numpy as np
from pinocchio import SE3, log6


class LinearInterpolator(AbstractInterpolator):
    """
    LinearInterpolator
    ------------------
    linearly interpolate through whatever set of points (2d, SE3, joint values)
    """

    def __init__(self, points: PathPointsT):
        super().__init__(points)

    def computeInterpolation(self):
        raise NotImplementedError

    def getPathPoint(self, s: float) -> ReferenceT:
        raise NotImplementedError


# TODO: what is this evil???????????
def SLERPPointToPointInterpolation(
    T_w_start: SE3, T_w_goal: SE3, s: float
) -> tuple[SE3, np.ndarray]:
    T_w_s = SE3.Interpolate(T_w_start, T_w_goal, s)
    # NOTE: since we're solving for constant twist,
    # 1. i'm 90% sure we can do this from T_w_start because we're solving for a contant twist
    # 2. we don't even need to compute this every time
    V_s = log6(T_w_s.actInv(T_w_goal)).vector
    V_s = V_s / np.linalg.norm(V_s)

    return T_w_s, V_s


def computePath2DLength(path2D):
    x_i = path2D[:, 0][:-1]  # no last element
    y_i = path2D[:, 1][:-1]  # no last element
    x_i_plus_1 = path2D[:, 0][1:]  # no first element
    y_i_plus_1 = path2D[:, 1][1:]  # no first element
    x_diff = x_i_plus_1 - x_i
    x_diff = x_diff.reshape((-1, 1))
    y_diff = y_i_plus_1 - y_i
    y_diff = y_diff.reshape((-1, 1))
    path_length = np.sum(np.linalg.norm(np.hstack((x_diff, y_diff)), axis=1))
    return path_length


def path2D_to_trajectory2D(
    cfg: GlobalConfig, path2D: np.ndarray, velocity: float
) -> np.ndarray:
    """
    path2D_to_trajectory2D
    ---------------------
    read the technical report for details
    """
    path_length = computePath2DLength(path2D)
    # NOTE: sometimes the path planner gives me the same god damn points
    # and that's it. can't do much about, expect set those point then.
    # and i need to return now so that the math doesn't break down the line
    if path_length < 1e-3:
        return np.ones((cfg.n_knots + 1, 2)) * path2D[0]
    total_time = path_length / velocity
    # NOTE: assuming the path is uniformly sampled
    t_path = np.linspace(0.0, total_time, len(path2D))
    t_ocp = np.linspace(0.0, cfg.ocp_dt * (cfg.n_knots + 1), cfg.n_knots + 1)

    trajectory2D = np.array(
        [
            np.interp(t_ocp, t_path, path2D[:, 0]),
            np.interp(t_ocp, t_path, path2D[:, 1]),
        ]
    ).T
    return trajectory2D


# TODO: what is this evil???????????
def pathSE3_to_trajectorySE3(
    cfg: GlobalConfig, pathSE3: list[SE3], velocity: float
) -> list[SE3]:
    path2D = np.zeros((len(pathSE3), 2))
    for i, pose in enumerate(pathSE3):
        path2D[i] = pose.translation[:2]
    path_length = computePath2DLength(path2D)
    # NOTE: sometimes the path planner gives me the same god damn points
    # and that's it. can't do much about, expect set those point then.
    # and i need to return now so that the math doesn't break down the line
    if path_length < 1e-3:
        return [pathSE3[0]] * (cfg.n_knots + 1)
    total_time = path_length / velocity
    # NOTE: assuming the path is uniformly sampled
    t_path = np.linspace(0.0, total_time, len(path2D))
    t_ocp = np.linspace(0.0, cfg.ocp_dt * (cfg.n_knots + 1), cfg.n_knots + 1)

    trajectorySE3 = []
    path_index = 0
    for t_traj in t_ocp:
        if t_traj > t_path[path_index + 1]:
            if path_index < len(pathSE3) - 2:
                path_index += 1

        if path_index < len(pathSE3) - 2:
            pose_traj = SE3.Interpolate(
                pathSE3[path_index],
                pathSE3[path_index + 1],
                t_traj - t_path[path_index],
            )
            trajectorySE3.append(pose_traj)
        else:
            trajectorySE3.append(pathSE3[-1])
    return trajectorySE3
