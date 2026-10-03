from smc import load_config
from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.registry import ConfigRegistry
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
from smc.motion_planning.path_planning.test_planners.test_planner_config import (
    ConfigTestPathGenerator,
)
from smc.robots.interfaces.single_arm_interface import SingleArmInterface

from dataclasses import dataclass, field
import argparse
from pinocchio import SE3
import pinocchio as pin
import numpy as np
from typing import Callable


@ConfigRegistry.register("cart_pulling")
@dataclass
class ConfigCartPulling:
    base_to_cart_arclength: float = field(
        default=0.9,
        metadata={
            "help": "prefered path arclength from mobile base position to cart. this way you indirectly select which point on the cart you are controlling"
        },
    )
    safety_radius: float = field(
        default=0.5,
        metadata={"help": "safety radius around each path point"},
    )
    base_reference: bool = field(
        default=True,
        metadata={
            "help": "is the path defined just for end_effectors, or for the base also (base in front to make it pulling)"
        },
    )
    ebd_vs_ee_path_tradeoff: float = field(
        default=0.8,
        metadata={
            "help": "how much do you want the end_effectors to follow the path vs desired relative base_ee pose. only used (and only makes sense) if you use base reference. NOTE: must be in 0_1 range (directly used to slerp between the two)"
        },
    )


def getPathPoints():
    from smc.motion_planning.path_planning.test_planners.draw_2d_path import (
        drawOrLoad2DPath,
    )
    from smc.motion_planning.interpolation.preprocessing import reduce2DPathPoints

    cfg = load_config()
    path2D = drawOrLoad2DPath(cfg.test_path)
    path2D[:, 1] -= cfg.test_path.map_height / 2
    path2D = reduce2DPathPoints(0.5, path2D)
    path2D = np.array(path2D)
    path2D = path2D[:, :2]
    # radius = max(base_half_diagonal, cfg.safety_radius)
    radii = np.ones(len(path2D)) * cfg.cart_pulling.safety_radius
    return path2D, radii


def getCartPoints(T_w_c: SE3, cart_shape) -> list[SE3]:
    # NOTE: divided by 4 because we start from cart center!!!
    T_c_c1 = SE3(np.eye(3), np.array([0.0, cart_shape[1] / 4, 0.0]))
    T_c_c2 = SE3(np.eye(3), np.array([0.0, -cart_shape[1] / 4, 0.0]))
    T_w_c1 = T_w_c.act(T_c_c1)
    T_w_c2 = T_w_c.act(T_c_c2)
    # NOTE: height is assumed to be in T_w_c already
    # T_c_c3 = SE3(np.eye(3), np.array([-cart_shape[0] / 2, cart_shape[1] / 2, 0.0]))
    # T_c_c4 = SE3(np.eye(3), np.array([-cart_shape[0] / 2, -cart_shape[1] / 2, 0.0]))
    # T_w_c3 = T_w_c.act(T_c_c3)
    # T_w_c4 = T_w_c.act(T_c_c4)
    # return [T_w_c1, T_w_c2, T_w_c3, T_w_c4]
    return [T_w_c1, T_w_c2]


def checkCartObstacleDistance(
    s_c: float, smooth_path, cartPoints: SE3
) -> tuple[bool, float]:
    circles = smooth_path.getClosestCircles(s_c)
    collision_flag = False
    min_distance = 1000
    for center, radius in circles:
        for cartPoint in cartPoints:
            distance = radius - np.linalg.norm(center - cartPoint.translation[:2])
            min_distance = min(min_distance, distance)
            if distance < 0.0:
                collision_flag = True
                break
        if collision_flag:
            break

    return collision_flag, min_distance


def transfEERefToBRef(cfg: ConfigTestPathGenerator, T_s: SE3) -> SE3:
    rot_mat_x = pin.rpy.rpyToMatrix(cfg.path_rotx, 0.0, 0.0)
    T_s_b = T_s.copy()
    T_s_b.rotation = T_s_b.rotation @ rot_mat_x.T
    T_s_b.translation[2] = 0.0
    return T_s_b


def bringPathToRobot(path2D, T_w_s1):
    first_pt = path2D[0]
    movedPath = path2D.copy()
    for i, pt in enumerate(path2D):
        T_pt = pin.SE3(np.eye(3), np.array([pt[0], pt[1], 0.0]))
        translation = T_w_s1.actInv(T_pt).translation
        movedPath[i][0] = translation[0]
        movedPath[i][1] = translation[1]
    return movedPath


def computeInitialBasePoseForCtrlWithBaseReference(
    cfg: GlobalConfig,
    smooth_path: AbstractInterpolator,
) -> tuple[float, SE3]:
    # find arclength corresponding to distance to base
    arclength = 0.0
    s_step = 0.001
    s = 0.0
    T_s = smooth_path.getPoint(0.0)
    p_prev = T_s.translation[:2]
    while arclength < cfg.cart_pulling.base_to_cart_arclength:
        s += s_step
        T_s = smooth_path.getPoint(s)
        p_base_current = T_s.translation[:2]
        path_point_relative_distances = np.linalg.norm(p_base_current - p_prev)
        p_prev = p_base_current
        arclength += path_point_relative_distances

    if cfg.debug_prints:
        print(
            "the preferred distance of ",
            arclength,
            "is achieved at s value of",
            s,
        )
    s_base_offset = s
    return s_base_offset, T_s


def constructInitialT_w_abs(
    cfg: argparse.Namespace, path_base: np.ndarray, rotation: np.ndarray
) -> SE3:
    direction = path_base[1] - path_base[0]
    handlebar_direction = -1 * direction
    handlebar_direction = handlebar_direction / np.linalg.norm(handlebar_direction)
    offset = cfg.base_to_cart_arclength * handlebar_direction
    translation = path_base[0] + offset
    translation[2] = cfg.path_height
    return SE3(rotation, translation)


def initializePastData(
    cfg: argparse.Namespace, T_w_e: SE3, p_base: np.ndarray, max_base_v: float
) -> np.ndarray:
    # prepopulate past data to make base and cart be on the same path in the past
    # (which didn't actually happen because this just started)
    p_ee = T_w_e.translation[:2]
    straight_line_path = np.linspace(p_ee, p_base, cfg.past_window_size)
    # straight_line_path_timed = path2D_timed(cfg, straight_line_path, max_base_v)
    # return straight_line_path_timed # this one is shortened to cfg.n_knots! and we want the whole buffer
    return straight_line_path


def constructedTradedOffEEReference(
    cfg: GlobalConfig, robot: SingleArmInterface, T_w_s: pin.SE3, V_s: np.ndarray
) -> tuple[pin.SE3, np.ndarray]:
    """
    constructedTradedOffEEReference
    -------------------------------
    NOTE: experiments clearly show that following the path exactly
    leads to super bad sideways arms in tight corners.
    and both the base and the ees are on the path so the control objective is
    achieved. it's just that it's a bad objective to attain.
    likewise, if we focus only on having end-effectors on the path and the relative base-ee pose,
    the base goes too much off the path (the robot has a "fat ass" and clearly
    can not manage tight corners).
    so we in fact want something in between these two options.
    enter slerping between the "neutral" base-ee pose and
    keeping the ee on the path.
    we keep the base on the path as that goes first (we're cart-pulling)
    so it makes sense for that to respect the path more.
    that also, conveniently, makes it so that we have exhausted all dofs
    expect the nullspace of the arms,
    which you can use for your favourite secondary ik objective (manip, posture, joint limits, w/e)
    """
    # EBP = end-effector base relative pose goal (neutral cart-pulling pose).
    # this is only [x,y, theta_z] = [-cfg.prefered_base_handlebar_distance,0.0,0.0]
    # fortunately [z, theta_x, theta_y] are defined by the grasp constraint
    # (we're grasping the handlebar rigidly).
    # [z, theta_x, theta_y] = [cfg.path_height, 0.0,0.0] - the last two
    # are just how we decided to do the grasp
    # let's defined the whole thing as T_b_e, i.e. absolute ee frame in base frame.
    T_b_e = pin.SE3(
        pin.rpy.rpyToMatrix(-np.pi, 0.0, 0.0),
        np.array(
            [-cfg.cart_pulling.base_to_cart_arclength, 0.0, cfg.test_path.path_height]
        ),
    )
    # now we need to map this to the world frame.
    # call that T_w_EBP
    # since it's defined in the base frame, it's easy:
    T_w_EBP = robot.T_w_b.act(T_b_e)
    # and now we can do our slerp
    T_w_armsref = pin.SE3.Interpolate(
        T_w_EBP, T_w_s, cfg.cart_pulling.ebd_vs_ee_path_tradeoff
    )

    # TODO: think whether and how you could/should map V_s to new reference

    return T_w_armsref, V_s
