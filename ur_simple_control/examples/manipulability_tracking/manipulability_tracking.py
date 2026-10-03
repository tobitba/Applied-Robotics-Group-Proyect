from smc import load_config, getRobotFromConfig
from smc.bookkeeping.load_config import GlobalConfig
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.control.control_loop_manager import ControlLoopManager
from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
from pinocchio import SE3

import numpy as np
import pinocchio as pin
from collections import deque
from functools import partial
from scipy import linalg
from qpsolvers import solve_qp

# viz
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
import meshcat.geometry as g


def velocity_ellipsoid(J: np.ndarray, dq_max: np.ndarray):
    """
    note: taken from pycapacity (i mean it's 2 lines of math, but still)
    """
    # limits scaling (idk how, didnt' check)
    W = np.diagflat(dq_max)
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(J.dot(W))

    # create the ellipsoid from the singular values and the unit vector angle
    # raddi, rotation
    return S, U


def computeEllipsoidToEllipsoidTangentVector(
    Sigma: np.ndarray, Lambda: np.ndarray
) -> np.ndarray:
    # now compute the tangent vector, rooted at M_current,
    # which points toward Sigma_w_goal.
    # since M \in 3 x 3, the dimension of SPD space is
    # D = D * (D + 1) / 2 = 6.
    # important because this is the dimension of the vector i'm looking for
    # distance (Sigma, Lambda) = frobenius_norm(log(Sigma^-1/2 * Lambda * Sigma^-1/2 )),
    # which is
    # distance (Sigma, Lambda) = Log_Sigma(Lambda)
    #      = Sigma^1/2 * log(Sigma^-1/2 * Lambda * Sigma^-1/2 ) * Sigma^1/2
    # we use this quantity to formulate the controller
    Sigma_minus_pow = linalg.fractional_matrix_power(Sigma, -1 / 2)
    Sigma_plus_pow = linalg.fractional_matrix_power(Sigma, 1 / 2)
    # from  M to Sigma, just the log operation
    # on SPD manifold, trust the math people it makes sense
    L = (
        Sigma_plus_pow
        @ linalg.logm(Sigma_minus_pow @ Lambda @ Sigma_minus_pow)
        @ Sigma_plus_pow
    )
    return L


def computeEllipsoidFollowingCost(
    robot: SingleArmInterface,
    d: int,
    L: np.ndarray,
    cost: float,
    tikhonov_damp=1e-3,
):
    J = robot.getJacobian()[:d, :]
    # we need calJ_3.
    # part_M/part_q = dJ/dq * J^T + J * (dJ/dq)^T

    # issue 1: dJ/dq is a d x d x n tensor,
    # and i want to work with matrices in the end,
    # so i should create the matricized objects directly.
    # this is fine, just needs to be beared in mind.

    # issue 2: pinocchio does not give me dJ/dq.
    # instead, it gives me dotJ/dq_i.
    # this is also fine if for joint i,
    # i select velocity to be [0,...,1,...0] on ith entry

    # matricized result
    calJ_3 = np.zeros((d**2, robot.model.nv))
    for i in range(robot.model.nv):
        # partial derivative w.r.t. ith joint has this effect on velocity
        # as joint velocities are mutually independent
        qdot = np.zeros(robot.model.nv)
        qdot[i] = 1.0

        # required every time because it depends on velocity (obviously)
        pin.computeJointJacobiansTimeVariation(robot.model, robot.data, robot.q, qdot)

        # required as per getFrameJacobianTimeVariation doc
        pin.updateFramePlacements(robot.model, robot.data)
        dJ_dqi = pin.getFrameJacobianTimeVariation(
            robot.model, robot.data, robot.ee_frame_id, pin.LOCAL
        )

        # again, cut off the angular velocities
        dJ_dqi = dJ_dqi[:d, :]
        dM_dqi = dJ_dqi @ J.T + J @ dJ_dqi.T

        calJ_3[:, i] = dM_dqi.reshape(-1)

    P = (calJ_3.T @ calJ_3) + (tikhonov_damp * np.eye(calJ_3.shape[1], dtype="double"))
    q = -2 * L.reshape(-1).T @ calJ_3

    return cost * P, cost * q


def moveTowardEllipsoidControlLoop(
    Sigma_w_goal: SE3,
    viz: MeshcatVisualizer,
    cfg: GlobalConfig,
    robot: SingleArmInterface,
    t: int,  # will be float eventually
    past_data: dict[str, deque[np.ndarray]],
) -> tuple[bool, dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    EEP2PCtrlLoopTemplate
    ---------------
    generic control loop for point to point motion with for the end-effector
    (handling error to final point etc).
    """
    breakFlag = False
    log_item = {}
    save_past_item = {}

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
    L = computeEllipsoidToEllipsoidTangentVector(M_current, Sigma_w_goal)
    P, q = computeEllipsoidFollowingCost(robot, d, L, 1.0)

    # solve IK as QP
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
    radii, rotation = velocity_ellipsoid(J, robot.max_v)
    # meshcat ellipsoid
    ellipsoid = g.Ellipsoid(radii=radii / 5)

    viz.viewer["ellipse"].set_object(
        ellipsoid,
        g.MeshBasicMaterial(color=0xFF5500, transparent=True, opacity=0.4),
    )
    viz.viewer["ellipse"].set_transform(
        pin.SE3(rotation, robot.T_w_e.translation).homogeneous
    )
    viz.display(robot.q)

    err_vector_norm = np.linalg.norm(L)
    if np.linalg.norm(v_cmd) < cfg.goal_error:
        breakFlag = True
    log_item["qs"] = robot.q
    log_item["vs"] = robot.v
    log_item["vs_cmd"] = v_cmd
    log_item["err_norm"] = err_vector_norm.reshape((1,))
    return breakFlag, save_past_item, log_item


def moveTowardEllipsoid(
    cfg: GlobalConfig,
    robot: SingleArmInterface,
    Sigma_w_goal: np.ndarray,
    viz: MeshcatVisualizer,
    run=True,
) -> None | ControlLoopManager:
    controlLoop = partial(moveTowardEllipsoidControlLoop, Sigma_w_goal, viz, cfg, robot)
    log_item = {
        "qs": np.zeros(robot.nq),
        "vs": np.zeros(robot.nv),
        "vs_cmd": np.zeros(robot.nv),
        "err_norm": np.zeros(1),
    }
    save_past_dict = {"L_prev": np.zeros(1)}
    loop_manager = ControlLoopManager(robot, controlLoop, cfg, save_past_dict, log_item)
    if run:
        loop_manager.run()
    else:
        return loop_manager


if __name__ == "__main__":
    cfg = load_config()
    cfg.visualizer = False
    robot = getRobotFromConfig(cfg)
    robot._step()
    if (not cfg.real) and cfg.randomly_generate_goal:
        T_w_goal = getRandomlyGeneratedGoal(cfg, robot)
        if cfg.visualizer:
            robot.visualizer_manager.sendCommand({"Mgoal": T_w_goal})
    else:
        if cfg.real and cfg.randomly_generate_goal:
            print("Ain't no way you're going to a random goal on the real robot!")
            print("Look at the current pose, define something appropriate manually")
        T_w_goal = defineGoalPointCLI(robot)

    # start meshcat here
    viz = MeshcatVisualizer(
        model=robot.model,
        collision_model=robot.collision_model,
        visual_model=robot.visual_model,
    )
    viz.display(robot.q)

    # generate random goal ellipsoid
    q_old = robot.q
    q_rand = 2 * np.pi * np.random.random(robot.model.nv) - np.pi
    robot._q = q_rand
    robot._step()
    J = robot.getJacobian()[:3, :]
    Sigma_w_goal = J @ J.T
    robot._q = q_old
    robot._step()
    radii, rotation = velocity_ellipsoid(J, robot.max_v)
    print(radii)

    # meshcat ellipsoid
    ellipsoid = g.Ellipsoid(radii=radii / 5)

    viz.viewer["goal_ellipse"].set_object(
        ellipsoid,
        g.MeshBasicMaterial(color=0x0055FF, transparent=True, opacity=0.4),
    )
    # NOTE: it doesn't matter where it is, the only important thing is that we see it
    viz.viewer["goal_ellipse"].set_transform(pin.SE3(rotation, np.ones(3)).homogeneous)

    viz.display(robot.q)
    moveTowardEllipsoid(cfg, robot, Sigma_w_goal, viz)

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.plotAllControlLoops()
        robot._log_manager.saveLog()
