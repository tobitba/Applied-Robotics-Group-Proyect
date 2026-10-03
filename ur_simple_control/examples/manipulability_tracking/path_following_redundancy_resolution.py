from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.registry import ConfigRegistry
from smc.bookkeeping.types import PastData
from smc.control.control_typing import ControlLoopReturn
from smc.control.cartesian_space import moveL
from smc import load_config, getRobotFromConfig
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
from smc.motion_planning.path_planning.test_planners import test_path_generator
from smc.motion_planning.trajectory_generation.end_effector_velocity_limit_scaling import (
    #    computeTimeScalingEEVelConstraint,
    UniformTimeLawWithEEContraints,
)
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.control.joint_space.joint_space_point_to_point import moveJPWTraj
from manipulability_tracking import (
    velocity_ellipsoid,
    computeEllipsoidToEllipsoidTangentVector,
    computeEllipsoidFollowingCost,
)
from smc.control.control_loop_manager import ControlLoopManager
from smc.motion_planning.path_planning.test_planners.draw_2d_path import (
    drawOrLoad2DPath,
)
from smc.motion_planning.interpolation.preprocessing import reduce2DPathPoints


from dataclasses import dataclass, field
import numpy as np
import pinocchio as pin
from functools import partial
from qpsolvers import solve_qp

COLOR_TARGET = 0x0055FF
COLOR_CURRENT = 0xFF5500


@ConfigRegistry.register("grr")
@dataclass
class ConfigGeometricRedundancyResolution:
    w_track: float = field(
        metadata={
            "help": "tracking (end-effector desired twist) task weight cost in the QP"
        },
        default=1.0,
    )
    w_manip: float = field(
        metadata={"help": "manipulability task weight cost in the QP"},
        default=1.0,
    )
    K_ellipsoid: float = field(
        metadata={"help": "weight for feedback in path following"},
        default=1.0,
    )
    K_fb: float = field(
        metadata={"help": "tracking controller position error gain"},
        default=1.0,
    )


def targetSphere():
    return np.eye(3)


# TODO: refactor smooth path into motion plan
# def curvatureBasedManipulability(smooth_path: AbstractInterpolator, s: float):
#    # rotation dictated by curvature.
#    # --> probably just acceleration?
#    # - or nothing?
#
#    # shape dictated by curvature
#    kappa = np.abs(smooth_path.curvature(s))
#    # let's put some restraints
#    allowable_ratio = 2
#    ratio = np.clip(kappa, 1 / allowable_ratio, allowable_ratio)
#    len_x = 1 / ratio
#    len_y = ratio
#    ellipsoid = np.eye(3)
#    ellipsoid[0, 0] = len_x
#    ellipsoid[1, 1] = len_y
#    # R2, p2 = smooth_2Dpath.getSE2FrenetInduced(s)
#    # T_s = smooth_path.liftSE2ToSE3(R2, p2, smooth_path.rotx, smooth_path.height)
#    # ellipsoid = T_s.rotation.copy()
#    # ellipsoid[:, 0] *= len_x
#    # ellipsoid[:, 1] *= len_y
#    return ellipsoid


# TODO: plot the K vector, i have no idea what it is actually.
# as in i don't know what the induced framed is.
# --> the orientation we have on the path has nothing to do with the frenet frame
# TODO: so the end-effector frame and the frenet frame do not correspond
def curvatureBasedManipulability(smooth_path: AbstractInterpolator, s: float):
    K_s = smooth_path.get3DCurvatureVector(s)
    ellipsoid = np.eye(3)

    # let's pretend that x direction is always the tangent vector
    V_s = smooth_path.getVelocity(s)
    T_s = V_s / np.linalg.norm(V_s)
    ellipsoid[:, 0] = T_s
    # now have to get the other 2 two axes from K_s somehow
    # if i have the unit axis i can just do dot product of K_s with them
    # TODO: i pulled this out of a hat
    ellipsoid[:, 1] = K_s @ ellipsoid[:, 1]
    ellipsoid[:, 2] = K_s @ ellipsoid[:, 2]
    return ellipsoid


# TODO: refactor smooth path into motion plan
def pathFollowingWithNaturalRedundancyResolutionControlLoop(
    smooth_path,
    time_law,
    cfg: GlobalConfig,
    robot: SingleArmInterface,
    i: int,  # will be float eventually
    past_data: PastData,
) -> ControlLoopReturn:
    """
    EEP2PCtrlLoopTemplate
    ---------------
    generic control loop for point to point motion with for the end-effector
    (handling error to final point etc).
    """
    breakFlag = False
    log_item = {}
    save_past_item = {}

    t = i * robot.dt
    s, s_dot, s_ddot = time_law.getCurrentPathParameter(t)
    Psi_s, V_s, _ = smooth_path.getPathPoint(s)
    V_s = V_s * s_dot

    position_error = pin.log6(robot.T_w_e.actInv(Psi_s)).vector
    V_cmd = cfg.cs.K_fb * position_error + V_s

    # i only care about linear velocities
    # TODO: verify whether linear velocities are actually the first 3 numbers
    # (just command to move in a particular axis direciton, all else 0, and it will be obvious)
    d = 3
    J = robot.getJacobian()[:d, :]
    # for math i need the SPD matrix representation of the ellipsoid.
    # not inversed as per calinon in paper
    M_current = J @ J.T

    # now compute the tangent vector, rooted at M_current,
    # which points toward Sigma_w_goal.
    # L = computeEllipsoidToEllipsoidTangentVector(M_current, Sigma_w_goal)
    # ellipsoid_target = cfg.K_ellipsoid * targetSphere()
    ellipsoid_target = cfg.grr.K_ellipsoid * curvatureBasedManipulability(
        smooth_path, s + 0.05
    )
    L = computeEllipsoidToEllipsoidTangentVector(M_current, ellipsoid_target)
    P_manip, q_manip = computeEllipsoidFollowingCost(robot, d, L, cfg.grr.w_manip)

    J = robot.getJacobian()
    P_vel = (J.T @ J) + (cfg.cs.tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    P_vel = cfg.grr.w_track * P_vel
    q_vel = -cfg.grr.w_track * 2 * V_cmd.T @ J
    P = P_manip + P_vel
    q = q_manip + q_vel

    # solve IK as QP
    # TODO: make the QP framework in SMC extensible.
    # i want to easily test and play around with different costs.
    # it's kind of the whole point that i can have 5 of them if i want
    # (but i also really need comparisons here)
    A = None
    b = None
    G = None
    h = None

    # NOTE: if implementing a new QP and you're getting something weird,
    # turning on verbose might be a good idea
    v_cmd = solve_qp(
        P,
        q,
        G,
        h,
        A,
        b,
        -1 * robot.max_v,
        robot.max_v,
        solver=cfg.cs.qp_solver,
        verbose=False,
    )

    robot.sendVelocityCommand(v_cmd)

    # update viz
    # for that i need the radii + rotation representation of the ellipsoid
    # meshcat ellipsoid
    if cfg.visualizer:
        J = robot.getJacobian()[:d, :]
        radii, rotation = velocity_ellipsoid(J, robot.max_v)
        robot.visualizer_manager.sendCommand(
            {"ellipsoid_current": (radii, robot.T_w_e, COLOR_CURRENT)}
        )
        rotation_target, radii_target, _ = np.linalg.svd(ellipsoid_target)
        robot.visualizer_manager.sendCommand(
            {"ellipsoid_target": (radii_target, Psi_s, COLOR_TARGET)}
        )

        # TODO: target ellipsoid

    err_manip_vector_norm = np.linalg.norm(L)
    if s >= 1.0:
        breakFlag = True
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["vs_cmd"] = v_cmd
    log_item["err_norm_task"] = np.linalg.norm(position_error).reshape((1,))
    log_item["err_norm_manip"] = err_manip_vector_norm.reshape((1,))
    return breakFlag, save_past_item, log_item


def pathFollowingWithNaturalRedundancyResolution(
    cfg: GlobalConfig,
    robot: SingleArmInterface,
    smooth_path,
    time_law,
    run=True,
) -> None | ControlLoopManager:
    controlLoop = partial(
        pathFollowingWithNaturalRedundancyResolutionControlLoop,
        smooth_path,
        time_law,
        cfg,
        robot,
    )
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "err_norm_task": np.zeros(1),
        "err_norm_manip": np.zeros(1),
    }
    save_past_dict = {"L_prev": np.zeros(1)}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


def get2DPathPoints():

    cfg = load_config()
    path2D = drawOrLoad2DPath(cfg.test_path)
    path2D[:, 1] -= cfg.test_path.map_height / 2
    path2D = reduce2DPathPoints(0.5, path2D)
    path2D = np.array(path2D)
    path2D = path2D[:, :2]
    # radius = max(base_half_diagonal, cfg.safety_radius)
    return path2D


if __name__ == "__main__":
    from smc.motion_planning.path_planning.test_planners.test_path_generator import (
        TestSE3PathGenerator,
    )
    from smc.motion_planning.interpolation.bsplinesSE3 import CubicBezierSplineSE3

    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    robot._step()
    robot._step()

    # path2D = getPathPoints()
    test_path_generator = TestSE3PathGenerator(cfg)
    test_path_generator.computePath(robot.T_w_e)
    pathSE3 = test_path_generator.getPathPoints()

    init_vel = np.zeros(6)
    final_vel = np.zeros(6)
    smooth_path = CubicBezierSplineSE3(pathSE3)

    # from smc.motion_planning.interpolation.bsplines2D_unedited_vibecode import CubicSpline2D
    # init_vel = np.zeros(2)
    # final_vel = np.zeros(2)
    # smooth_2Dpath = CubicSpline2D(path2D, 0.0, init_vel, final_vel)
    # smooth_2Dpath.setSE3LiftingParams(
    #    cfg.test_path.path_rotx, cfg.test_path.path_height
    # )
    #    T_1 = smooth_2Dpath.getPathPoint(0.001)[0]
    #
    #    path2D = bringPathToRobot(path2D, T_1)
    #    smooth_2Dpath = CubicSpline2D(
    #        path2D,
    #        0.0,
    #        init_vel,
    #        final_vel,
    #    )
    #    smooth_2Dpath.setSE3LiftingParams(cfg.path_rotx, cfg.path_height)

    if cfg.visualizer:
        # robot.visualizer_manager.sendCommand({"Mgoal": T_list[0]})
        # viz_path = np.hstack((path2D, np.zeros((len(path2D), 1))))
        # print(viz_path)
        viz_Ts = []
        ss = np.linspace(0, 1, 500)
        for s in ss:
            viz_Ts.append(smooth_path.getPoint(s))
        robot.visualizer_manager.sendCommand({"framepath": viz_Ts})

    robot.mode = robot.control_mode.upper_body
    moveJPWTraj(robot.comfy_configuration, cfg, robot)
    T_1 = smooth_path.getPoint(0.001)
    robot.mode = robot.control_mode.whole_body
    moveL(cfg, robot, T_1)
    time_law = UniformTimeLawWithEEContraints(smooth_path)
    time_law.T_total = 40
    pathFollowingWithNaturalRedundancyResolution(cfg, robot, smooth_path, time_law)

    robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
