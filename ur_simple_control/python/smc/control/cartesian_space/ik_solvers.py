from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
from typing import Callable
import numpy as np
from qpsolvers import solve_qp
import pinocchio as pin

from importlib.util import find_spec

if find_spec("proxsuite"):
    from proxsuite import proxqp

from functools import partial
from argparse import Namespace

from smc.robots.interfaces.mobile_base_interface import MobileBaseInterface
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)


def getIKSolver(
    cfg: ConfigCartesianSpace, robot: SingleArmInterface | DualArmInterface
) -> Callable[[Namespace, SingleArmInterface, np.ndarray], np.ndarray]:
    """
    getIKSolver
    -----------------
    A string argument is used to select one of these.
    It's a bit ugly, bit totally functional and OK solution.
    we want all of theme to accept the same arguments, i.e. the jacobian and the error vector.
    if they have extra stuff, just map it in the beginning with partial
    NOTE: this could be changed to something else if it proves inappropriate later
    TODO: write out other algorithms
    """
    if cfg.ik_solver == "dampedPseudoinverse":
        return dampedPseudoinverse
    if cfg.ik_solver == "jacobianTranspose":
        return jacobianTranspose
    if cfg.ik_solver == "adaptiveDampedPseudoinverse":
        return adaptiveDampedPseudoinverse
    # the tunable one
    if cfg.ik_solver == "QP":
        return partial(
            IKQP,
            cfg.vel_track_cost,
            cfg.posture_cost,
            cfg.manipulability_cost,
            cfg.ebd_cost,
        )
    if cfg.ik_solver == "QPVanilla":
        return partial(IKQP, -1.0, -1.0, -1.0, -1.0)
    if cfg.ik_solver == "QPWithPosture":
        return partial(IKQP, 50, 1e-2, -1.0, -1.0)
    if cfg.ik_solver == "QPManipMax":
        return partial(IKQP, 50, -1.0, 1e-1, -1.0)
    if cfg.ik_solver == "QPWithPostureAndEBDistance":
        assert issubclass(robot.__class__, SingleArmWholeBodyInterface) or issubclass(
            robot.__class__, DualArmWholeBodyInterface
        )
        return partial(IKQP, 50, 1e-3, -1.0, 1e-1)

    # TODO: apply rewrite
    if cfg.ik_solver == "QPproxsuite":
        H = np.eye(robot.nv)
        g = np.zeros(robot.nv)
        G = np.eye(robot.nv)
        J = robot.getJacobian()
        A = np.eye(J.shape[0], robot.nv)
        b = np.ones(J.shape[0]) * 0.1
        qp = proxqp.dense.QP(robot.nv, J.shape[0], robot.nv)
        # proxqp does lb <= Cx <= ub
        C = np.eye(robot.nv)
        lb = -1 * robot.max_v
        ub = robot.max_v
        qp.init(H, g, A, b, G, lb, ub)
        qp.solve()
        return partial(QPproxsuite, qp)

    # TODO: implement and add in the rest
    # if controller_name == "invKinmQPSingAvoidE_kI":
    #    return invKinmQPSingAvoidE_kI
    # if controller_name == "invKinmQPSingAvoidE_kM":
    #    return invKinmQPSingAvoidE_kM
    # if controller_name == "invKinmQPSingAvoidManipMax":
    #    return invKinmQPSingAvoidManipMax

    # default
    return dampedPseudoinverse


def dampedPseudoinverse(
    cfg: ConfigCartesianSpace, robot: SingleArmInterface, err_vector: np.ndarray
) -> np.ndarray:
    # NOTE: for fine tuning you could throw in:
    # - a weighting matrix W, having J @ W @ J.T
    # - a weighting matrix W on the err_vector, W @ err_vector
    # since this is robot and application specific, we go for the second option,
    # and let the user do it within their task-specific control loop
    J = robot.getJacobian()
    qd = (
        J.T
        @ np.linalg.inv(J @ J.T + np.eye(J.shape[0], J.shape[0]) * cfg.tikhonov_damp)
        @ err_vector
    )
    return qd


def jacobianTranspose(
    cfg: ConfigCartesianSpace, robot: SingleArmInterface, err_vector: np.ndarray
) -> np.ndarray:
    J = robot.getJacobian()
    qd = J.T @ err_vector
    return qd


# TODO: test this
# TODO: put in documentation
# TODO: find reference for this
def adaptiveDampedPseudoinverse(
    cfg: ConfigCartesianSpace, robot: SingleArmInterface, err_vector: np.ndarray
) -> np.ndarray:
    """
    Improved version of the damped pseudo-inverse computation with manipulability-based adaptive damping.

    Parameters:
    tikhonov_damp (float): The base damping factor used for regularization.
    J (numpy.ndarray): The Jacobian matrix of the system.
    err_vector (numpy.ndarray): The error vector representing the desired task-space velocity.

    Returns:
    numpy.ndarray: The computed joint velocity vector.
    """
    J = robot.getJacobian()
    # Compute the manipulability measure of the Jacobian
    m = robot.computeManipulabilityIndex()

    # Increase damping when manipulability is low to prevent numerical instability
    lambda_adaptive = cfg.tikhonov_damp / (m + 1e-6)

    # Perform Singular Value Decomposition (SVD) on the Jacobian
    U, S, Vt = np.linalg.svd(
        J, full_matrices=False
    )  # full_matrices=False avoids dimension mismatch

    # Construct the filtered singular values for the damped pseudo-inverse
    S_filtered = np.diag([s / (s**2 + lambda_adaptive) if s > 1e-4 else 0 for s in S])

    # Compute the Moore-Penrose pseudo-inverse
    J_pseudo = Vt.T @ S_filtered @ U.T

    # Compute the joint velocity
    qd = J_pseudo @ err_vector

    return qd


def computeEndEffectorGoalCost(
    J: np.ndarray, V_e_e: np.ndarray, tikhonov_damp=1e-3
) -> tuple[np.ndarray, np.ndarray]:
    """
    computeEndEffectorGoalCost
    -------------------------
    returns P quadratic cost matrix and q linear cost vector for
    end effector goal
    """
    P = (J.T @ J) + (tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    q = -2 * V_e_e.T @ J

    return P, q


def computePostureGoalCost(
    robot: SingleArmInterface, q_ref: np.ndarray, cost: float, tikhonov_damp=1e-3
) -> tuple[np.ndarray, np.ndarray]:
    """
    computeEndEffectorGoalCost
    -------------------------
    returns P quadratic cost matrix and q linear cost vector for
    end effector goal
    """
    P_posture = cost * np.eye(robot.nv)
    error_posture = q_ref - robot.q
    # TODO: need separate models for separate modes,
    # otherwise we're doing half-broken patching bullshit
    if (
        issubclass(robot.__class__, MobileBaseInterface)
        and robot.mode == robot.control_mode.whole_body
    ):
        error_posture = error_posture[1:]
        error_posture[:3] = 0.0
    q_posture = -cost * 2 * error_posture
    return P_posture, q_posture


# NOTE: we're just inserting this into the q.
# is this the most correct way to do it?
# no.
# but i have no idea what the jacobian of this is supposed to be
def computeManipulabilityGoalCost(robot: SingleArmWholeBodyInterface, cost: float):
    P_manip = np.zeros(robot.nv)
    q_manip = -1 * cost * robot.computeManipulabilityIndexQDerivative()
    # q_manip = cost * robot.computeManipulabilityIndexQDerivative()
    return P_manip, q_manip


def computeEBDistanceGoalCost(
    robot: SingleArmWholeBodyInterface,
    ebd_ref: np.ndarray,
    cost: float,
    tikhonov_damp=1e-3,
):
    J = robot.getJacobian()
    # x,y only
    # diff = (
    #    robot.T_w_b.rotation.T @ (robot.T_w_e.translation - robot.T_w_b.translation)
    # )[0:2]
    T_b_e = robot.T_w_b.actInv(robot.T_w_e)

    # x,y only
    # e_ebd = ebd_ref[0:2] - np.abs(diff)
    # x,y,theta_z
    ebd = T_b_e.translation.copy()
    # TODO: make this without rpy pliz
    # we should know better
    ebd[2] = pin.rpy.matrixToRpy(T_b_e.rotation)[2]
    # ebd[2] = pin.log3(T_b_e.rotation)[2]
    e_ebd = ebd_ref - ebd

    # e_ebd[2] = (ebd_ref[2] - pin.rpy.matrixToRpy(robot.T_w_e.rotation.T @ robot.T_w_b.rotation)[2]) % np.pi
    # ---------
    # explanation:
    # the absolute jacobian is (6, nv) for the absolute frame.
    # since we assume a symmetric task, it is literally just 1/2 jacobian_left_ee + 1/2 jacobian_right_ee.
    # 1. extract contibution to v_a, v_y from the absolute jacobian.
    #    - the default whole body DualArm jacobian in SMC is (12,nv), i.e. the left and the right jac are simply stacked (J_left_ee, J_right_ee).T
    # J_abs = 0.5 * (J[[0, 1], :] + J[[6, 7], :])  # (xL,yL,xR,yR) -> (xA,yA)
    # now with omega_z also (angular rot around z)
    J_abs = 0.5 * (J[[0, 1, 5]] + J[[6, 7, 11]])  # (xL,yL,xR,yR) -> (xA,yA)
    # np.sign(diff)[:,None] is literally just [sign(diff[0]), sign(diff[1])] here
    # J_ebd = -np.sign(diff)[:, None] * (J_abs - np.eye(2, robot.nv))  # 2x17
    J_ebd = J_abs - np.eye(3, robot.nv)  # 3x17
    P_ebd = (J_ebd.T @ J_ebd) + (tikhonov_damp * np.eye(J_ebd.shape[1], dtype="double"))
    P_ebd = cost * P_ebd
    cost_mat = -cost * np.eye(3)
    cost_mat[2] *= 10
    q_ebd = 2 * cost_mat @ e_ebd.T @ J_ebd
    return P_ebd, q_ebd


# NOTE: again, inspiration from pink
# also, for simplicity, i'm assuming all joints have limits.
# this is not correct in general.
# but i really can't be bothered at the moment of writing.
# however i do have to take robot mode into account,
# otherwise i have big problems.
# TODO:
# this is now again tied into not having multiple
# or reduced models available. this really need to be implemented
# before an official release.
# NOTE: joint limit gain of 0.5 means you only get to move
# half the gap to the limit.
# this seems pretty reasonable, i don't have a need to change it.
def constructJointLimitConstraint(robot: SingleArmInterface, joint_limit_gain=0.5):
    Delta_q_max = pin.difference(robot.model, robot._q, robot.model.upperPositionLimit)
    Delta_q_min = pin.difference(robot.model, robot._q, robot.model.lowerPositionLimit)
    q_max = joint_limit_gain * Delta_q_max
    q_min = joint_limit_gain * Delta_q_min

    # NOTE: q_max, q_min are of dimension nv here.
    # it's from what pin.difference spits out.
    # it make sense, at it gives you what we want
    q_max = q_max[robot.getJointVelocityIndexMask()]
    q_min = q_min[robot.getJointVelocityIndexMask()]
    projection_matrix = np.eye(robot.nv)
    G = np.vstack([projection_matrix, -projection_matrix])
    h = np.hstack([q_max / robot.dt, -q_min / robot.dt])
    return G, h


def constructJointAccelerationConstraint(
    robot: SingleArmInterface, joint_limit_gain=0.5
):
    a_max = np.ones(robot.nv) * robot._acceleration
    a_min = -1 * a_max
    lb_a = robot.dt * a_min + robot.v
    ub_a = robot.dt * a_max + robot.v

    # NOTE: q_max, q_min are of dimension nv here.
    # it's from what pin.difference spits out.
    # it make sense, at it gives you what we want
    # a_max = a_max[robot.getJointVelocityIndexMask()]
    # a_min = a_min[robot.getJointVelocityIndexMask()]
    projection_matrix = np.eye(robot.nv)
    G = np.vstack([projection_matrix, -projection_matrix])
    h = np.hstack([ub_a, -lb_a])
    print("="*50)
    print("G @ robot.v")
    print(G @ robot.v)
    print("h")
    print(h)
    return G, h


def IKQP(
    vel_track_cost: float,
    posture_cost: float,
    manipulability_cost: float,
    ebd_cost: float,
    cfg: ConfigCartesianSpace,
    robot: SingleArmInterface,
    V_e_e: np.ndarray,
    past_v=None,  # TODO: utilize
    tikhonov_damp=1e-3,
) -> np.ndarray | None:
    """
    QPquadprog
    ---------
    generic QP:
        minimize 1/2 x^T P x + q^T x
        subject to
                 G x \\leq h
                 A x = b
                 lb <= x <= ub
    inverse kinematics QP:
        for error e, with error jacobian J_e
        minimize ||J_e delta q - K*e||^2
        which becomes
        minimize 1/2 delta q^T (J_e^TJ + lambda I) delta q + 2 * e^T J delta q
        subject to
                 G qd \\leq h (optional)
                 lb <= delta q^t <= ub (optional)
        and we of course equate delta q^t with dt * v (joint velocities)
    """
    J = robot.getJacobian()

    # NOTE: negative means "not explicitly selected" - there is no one catch all case,
    # so putting sane defautls for all would be all are used, and that makes no sense
    if vel_track_cost < 0.0:
        # this pops out when computing P and q from the cost function
        # (which is a norm)
        vel_track_cost = 1

    Ps_n_qs: list[tuple[np.ndarray, np.ndarray]] = []
    # you always want the target velocity cost
    P_vel = (J.T @ J) + (tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    P_vel = vel_track_cost * P_vel
    q_vel = -vel_track_cost * 2 * V_e_e.T @ J
    Ps_n_qs.append((P_vel, q_vel))

    if posture_cost > 0.0:
        Ps_n_qs.append(
            computePostureGoalCost(robot, robot.comfy_configuration, posture_cost)
        )
    if manipulability_cost > 0.0:
        Ps_n_qs.append(computeManipulabilityGoalCost(robot, manipulability_cost))
    if ebd_cost > 0.0:
        assert issubclass(robot.__class__, SingleArmWholeBodyInterface) or issubclass(
            robot.__class__, DualArmWholeBodyInterface
        )
        try:
            # NOTE: the minus is only here because of slumi!
            # TODO: make it general by making use of the relative position of the arm's base
            # with respect to the mobile frame base!
            # --> although to be fair this will be robot config and urdf dependent,
            # and lord knows what goes on over there in general
            ebd_ref = np.array([-cfg.base_to_handlebar_preferred_distance, 0.0, 0.0])
        except AttributeError:
            print(
                "using sane default for distance, you don't have it defined via argument"
            )
            ebd_ref = np.array([0.9, 0.0, 0.0])

        Ps_n_qs.append(computeEBDistanceGoalCost(robot, ebd_ref, ebd_cost))

    P = np.zeros((robot.nv, robot.nv))
    q = np.zeros(robot.nv)
    for P_i, q_i in Ps_n_qs:
        P += P_i
        q += q_i
    # TODO: this is broken they way it's written now.
    # because i need v*dt
    if cfg.respect_joint_limits:
        G, h = constructJointLimitConstraint(robot)
    if cfg.respect_joint_accelerations:
        G, h = constructJointAccelerationConstraint(robot)
    if cfg.respect_joint_limits and cfg.respect_joint_accelerations:
        raise NotImplementedError(
            "need to stack both limit matrices for both, to have one G and one h, didn't do it, sorry :/"
        )
    if (not cfg.respect_joint_limits) and (not cfg.respect_joint_accelerations):
        G = None
        h = None

    # TODO:  make warmstarting the QP an argument, or make it a separate controller
    # options:
    # - past_v
    # -J.T @ err_vector
    # - pseudoinverse

    A = None
    b = None
    # NOTE: we work in the end-effector frame.
    # so it's the body jacobian.
    # so this is exactly what we want, no transformations needed
    # NOTE: this is stupid and does exactly what you don't want - it does not correct the errors, and moves you off the plane...
    #    if cfg.is_task_2D:
    #        # z axis is index 2
    #        # roll 3, pitch 3
    #        A = J[[2, 3, 4], :]
    #        b = np.zeros(3)

    # NOTE: if implementing a new QP and you're getting something weird,
    # turning on verbose might be a good idea
    qd = solve_qp(
        P,
        q,
        G,
        h,
        A,
        b,
        -1 * robot.max_v,
        robot.max_v,
        solver=cfg.qp_solver,
        verbose=False,
    )
    return qd


# TODO: apply rewrite fix here
def QPproxsuite(
    qp: proxqp.dense.QP,
    J: np.ndarray,
    V_e_e: np.ndarray,
) -> np.ndarray:
    # proxqp does lb <= Cx <= ub
    qp.settings.initial_guess = proxqp.InitialGuess.WARM_START_WITH_PREVIOUS_RESULT
    # qp.update(g=q, A=A, b=h, l=lb, u=ub)
    # NOTE: if err_vector is too big, J * qd = err_vector is infeasible
    # for the given ub, lb! (which makes perfect sense as it makes the constraints
    # incompatible)
    # thus we have to scale it to the maximum followable value under the given
    # inequality constraints on the velocities.
    # TODO:
    # unfortunatelly we need to do some non-trivial math to figure out what the fastest
    # possible end-effector velocity in the V_e_e direction is possible
    # given our current bounds. this is because it depends on the configuration,
    # i.e. on J of course - if we are in a singularity, then maximum velocity is 0!
    # and otherwise it's proportional to the manipulability ellipsoid.
    # NOTE: for now we're just eyeballing for sport
    # NOTE: this fails in low manipulability regions (which makes perfect sense also)
    V_e_e_norm = np.linalg.norm(V_e_e)
    max_V_e_e_norm = 0.3
    if V_e_e_norm < max_V_e_e_norm:
        b = V_e_e
    else:
        b = (V_e_e / V_e_e_norm) * max_V_e_e_norm
    # qp.update(A=J, b=err_vector)
    qp.update(A=J, b=b)
    qp.solve()
    qd = qp.results.x

    if qp.results.info.status == proxqp.PROXQP_PRIMAL_INFEASIBLE:
        # if np.abs(qp.results.info.duality_gap) > 0.1:
        print("didn't solve shit")
        qd = None
    return qd

    # def QPWithPosture(
    #    robot: SingleArmInterface,  # ideally i don't pass this, but it is what it is
    #    q_ref: np.ndarray,
    #    J: np.ndarray,
    #    V_e_e: np.ndarray,
    #    lb=None,
    #    ub=None,
    #    past_qd=None,
    #    tikhonov_damp=0.01,
    #    vel_cost=50,
    #    posture_cost=1e-3,
    # ) -> np.ndarray | None:
    #    """
    #    QPquadprogWithPosture
    #    ---------
    #    generic QP:
    #        minimize 1/2 x^T P x + q^T x
    #        subject to
    #                 G x \\leq h
    #                 A x = b
    #                 lb <= x <= ub
    #    inverse kinematics QP:
    #        minimize 1/2 delta q^T (J^TJ + lambda I) delta q
    #                    + q^T qd
    #        subject to
    #                 G qd \\leq h (optional)
    #                 lb <= qd <= ub (optional)
    #    P = J^J
    #    q = K V^TJ
    #    """
    #    P_vel = (J.T @ J) + (tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    #    P_vel = vel_cost * P_vel
    #    q_vel = -vel_cost * V_e_e.T @ J
    #    A = None
    #    b = None
    #    G = None
    #    h = None
    #
    #    P_posture = np.eye(robot.nv)
    #    P_posture = posture_cost * P_posture
    #    # TODO: need separate models for separate modes,
    #    # otherwise we're doing half-broken patching bullshit
    #    error_posture = q_ref - robot.q
    #    if (
    #        issubclass(robot.__class__, MobileBaseInterface)
    #        and robot.mode == robot.control_mode.whole_body
    #    ):
    #        error_posture = error_posture[1:]
    #        error_posture[:3] = 0.0
    #    q_posture = -posture_cost * error_posture
    #
    #    P = P_vel + P_posture
    #    q = q_vel + q_posture
    #    qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="quadprog", verbose=True)
    #    # qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="proxqp", verbose=False)
    #    return qd
    #
    # def QPWithPostureAndEBDistance(
    #    robot: SingleArmWholeBodyInterface,  # ideally i don't pass this, but it is what it is
    #    q_ref: np.ndarray,
    #    ebd_ref : np.ndarray,
    #    J: np.ndarray,
    #    V_e_e: np.ndarray,
    #    lb=None,
    #    ub=None,
    #    past_qd=None,
    #    tikhonov_damp=1e-3,
    #    vel_cost=50,
    #    posture_cost=1e-2,
    #    ebd_cost=1e-3,
    # ) -> np.ndarray | None:
    #    """
    #    QPquadprogWithPosture
    #    ---------
    #    generic QP:
    #        minimize 1/2 x^T P x + q^T x
    #        subject to
    #                 G x \\leq h
    #                 A x = b
    #                 lb <= x <= ub
    #    inverse kinematics QP:
    #        minimize 1/2 delta q^T (J^TJ + lambda I) delta q
    #                    + q^T qd
    #        subject to
    #                 G qd \\leq h (optional)
    #                 lb <= qd <= ub (optional)
    #    P = J^J
    #    q = K V^TJ
    #    """
    #    assert issubclass(robot.__class__, SingleArmWholeBodyInterface) or issubclass(robot.__class__, DualArmWholeBodyInterface)
    #    P_vel = (J.T @ J) + (tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    #    P_vel = vel_cost * P_vel
    #    q_vel = -vel_cost * V_e_e.T @ J
    #    A = None
    #    b = None
    #    G = None
    #    h = None
    #
    #    P_posture = np.eye(robot.nv)
    #    P_posture = posture_cost * P_posture
    #    # TODO: need separate models for separate modes,
    #    # otherwise we're doing half-broken patching bullshit
    #    error_posture = q_ref - robot.q
    #    if (
    #        issubclass(robot.__class__, MobileBaseInterface)
    #        and robot.mode == robot.control_mode.whole_body
    #    ):
    #        error_posture = error_posture[1:]
    #        error_posture[:3] = 0.0
    #    q_posture = -posture_cost * error_posture
    #
    #    # ebd := end-effector to base distance
    #
    #    diff = (robot.T_w_e.translation - robot.T_w_b.translation)[0:2]
    #    e_ebd = ebd_ref[0:2] - np.abs(diff)
    #    #e_ebd[2] = (ebd_ref[2] - pin.rpy.matrixToRpy(robot.T_w_e.rotation.T @ robot.T_w_b.rotation)[2]) % np.pi
    #    J_abs = 0.5 * (J[[0,1],:] + J[[6,7],:])  # (xL,yL,xR,yR) -> (xA,yA)
    #    J_ebd = - np.sign(diff)[:,None] * (J_abs - np.eye(2, robot.nv))  # 2x17
    #    P_ebd = (J_ebd.T @ J_ebd) + (tikhonov_damp * np.eye(J_ebd.shape[1], dtype="double"))
    #    P_ebd = ebd_cost * P_ebd
    #    # NOTE: THERE MIGHT HAVE TO BE A MINUS HERE IDK REAlLY
    #    q_ebd = ebd_cost * e_ebd.T @ J_ebd
    #
    #    P = P_vel + P_posture + P_ebd
    #    q = q_vel + q_posture + q_ebd
    #    qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="quadprog", verbose=True)
    #    # qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="proxqp", verbose=False)
    #    return qd
    #
    #
    ## NOTE: what is really the difference between this and solving
    ## separately for the base and the arms?
    ## - the arms don't effect the base
    ## - the base does affect the arms
    ## - so if the arms can't meet their target,
    ##   the base and adjust from it's target.
    ## but you would only use this if you have a reasonable constructed target for the base.
    ## so then what is the point?
    ## also #TODO: this is not implemented correctly
    # def QPWithEEAndBaseRef(
    #    robot: SingleArmWholeBodyInterface,  # ideally i don't pass this, but it is what it is
    #    J: np.ndarray,
    #    V_e_e: np.ndarray,
    #    V_b_b: np.ndarray, # 3 dimensional!
    #    lb=None,
    #    ub=None,
    #    past_qd=None,
    #    tikhonov_damp=0.01,
    #    vel_cost=50,
    # ) -> np.ndarray | None:
    #    """
    #    QPquadprogWithPosture
    #    ---------
    #    generic QP:
    #        minimize 1/2 x^T P x + q^T x
    #        subject to
    #                 G x \\leq h
    #                 A x = b
    #                 lb <= x <= ub
    #    inverse kinematics QP:
    #        minimize 1/2 delta q^T (J^TJ + lambda I) delta q
    #                    + q^T qd
    #        subject to
    #                 G qd \\leq h (optional)
    #                 lb <= qd <= ub (optional)
    #    P = J^J
    #    q = K V^TJ
    #    """
    #    assert issubclass(robot.__class__, SingleArmWholeBodyInterface) or issubclass(robot.__class__, DualArmWholeBodyInterface)
    #    P_vel = (J.T @ J) + (tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    #    P_vel = vel_cost * P_vel
    #    q_vel = -vel_cost * V_e_e.T @ J
    #    A = None
    #    b = None
    #    G = None
    #    h = None
    #
    #    J_b = np.zeros((12, robot.nv))
    #    J_b[[0,1,5],:3] = J[[0,1,5],:3]
    #    P_b_vel = (J_b.T @ J_b) + (tikhonov_damp * np.eye(J_b.shape[1], dtype="double"))
    #    P_b_vel = vel_cost * P_b_vel
    #    V_b_b= np.concatenate((V_b_b, V_b_b))
    #    q_b_vel = -vel_cost * V_b_b.T @ J_b
    #
    #
    #    P = P_vel + P_b_vel
    #    q = q_vel + q_b_vel
    #    qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="quadprog", verbose=True)
    #    # qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="proxqp", verbose=False)
    #    return qd

    # def QPManipMax(
    #    robot,
    #    J: np.ndarray,
    #    V_e_e: np.ndarray,
    #    lb=None,
    #    ub=None,
    #    tikhonov_damp=1e-3,
    #    vel_cost=50,
    #    manip_cost=1e0,
    # ) -> np.ndarray:
    #    """
    #    QPManipMAx
    #    ---------
    #    generic QP:
    #        minimize 1/2 x^T P x + q^T x
    #        subject to
    #                 G x \\leq h
    #                 A x = b
    #                 lb <= x <= ub
    #    inverse kinematics QP:
    #        minimize 1/2 qd^T P qd
    #                    + q^T qd (where q is the partial deriviative of the manipulability index w.r.t. q)
    #        subject to
    #                 G qd \\leq h (optional)
    #                 J qd = b    (mandatory)
    #                 lb <= qd <= ub (optional)
    #    """
    #    P_vel = (J.T @ J) + (tikhonov_damp * np.eye(J.shape[1], dtype="double"))
    #    P_vel = vel_cost * P_vel
    #    q_vel = -vel_cost * V_e_e.T @ J
    #    q_manip = -1 * manip_cost * robot.computeManipulabilityIndexQDerivative()
    #    P = P_vel
    #    q = q_vel + q_manip
    #    A = None
    #    b = None
    #    G = None
    #    h = None
    #    qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="quadprog", verbose=False)
    #    return qd


# NOTE: incorrect version, put here in case someone wants to go down this path
# def QPquadprog(
#    J: np.ndarray, V_e_e: np.ndarray, lb=None, ub=None, past_qd=None
# ) -> np.ndarray:
#    """
#    QPquadprog
#    ---------
#    generic QP:
#        minimize 1/2 x^T P x + q^T x
#        subject to
#                 G x \\leq h
#                 A x = b
#                 lb <= x <= ub
#    inverse kinematics QP:
#        minimize 1/2 qd^T P qd
#                    + q^T qd (optional secondary objective)
#        subject to
#                 G qd \\leq h (optional)
#                 J qd = b    (mandatory)
#                 lb <= qd <= ub (optional)
#    """
#    P = np.eye(J.shape[1], dtype="double")
#    # secondary objective is given via q
#    # we set it to 0 here, but we should give a sane default here
#    q = np.array([0] * J.shape[1], dtype="double")
#    G = None
#    # NOTE: if err_vector is too big, J * qd = err_vector is infeasible
#    # for the given ub, lb! (which makes perfect sense as it makes the constraints
#    # incompatible)
#    # thus we have to scale it to the maximum followable value under the given
#    # inequality constraints on the velocities.
#    # TODO:
#    # unfortunatelly we need to do some non-trivial math to figure out what the fastest
#    # possible end-effector velocity in the V_e_e direction is possible
#    # given our current bounds. this is because it depends on the configuration,
#    # i.e. on J of course - if we are in a singularity, then maximum velocity is 0!
#    # and otherwise it's proportional to the manipulability ellipsoid.
#    # NOTE: for now we're just eyeballing for sport
#    # NOTE: this fails in low manipulability regions (which makes perfect sense also)
#    V_e_e_norm = np.linalg.norm(V_e_e)
#    max_V_e_e_norm = 0.3
#    if V_e_e_norm < max_V_e_e_norm:
#        b = V_e_e
#    else:
#        b = (V_e_e / V_e_e_norm) * max_V_e_e_norm
#    A = J
#    # TODO: you probably want limits here
#    lb = None
#    ub = None
#    # lb *= 20
#    # ub *= 20
#    h = None
#    # (n_vars, n_eq_constraints, n_ineq_constraints)
#    # qp.init(H, g, A, b, C, l, u)
#    # print(J.shape)
#    # print(q.shape)
#    # print(A.shape)
#    # print(b.shape)
#    # NOTE: you want to pass the previous solver, not recreate it every time
#    ######################
#    # solve it
#    qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="quadprog", verbose=False)
#    # qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="proxqp")
#    return qd


# mobile manipulator full-body control option: minimum end-effector and base distance
# def dPi_Weighted_nullspace(tikhonov_damp, q, J, err_vector, mode, robot):
#    """
#    Computes the weighted damped pseudo-inverse with null space motion optimization.
#    Ensures that the end-effector (EE) maintains a minimum base distance (>= 0.5) in "moveu" mode.
#
#    Parameters:
#    ----------
#    tikhonov_damp : float
#        Regularization damping factor for numerical stability.
#    q : numpy.ndarray
#        Current joint positions.
#    J : numpy.ndarray
#        Jacobian matrix of the robot.
#    err_vector : numpy.ndarray
#        Error vector representing the desired task-space velocity.
#    mode : str
#        Operation mode, either:
#        - "moveL": Moves towards a target position.
#        - "moveu": Moves with a given velocity while maintaining a minimum base distance.
#    robot : RobotManager
#        Robot instance, used to compute the end-effector to base distance.
#
#    Returns:
#    -------
#    numpy.ndarray
#        Computed joint velocity vector.
#    """
#
#    # Rest posture of the robot
#    q_rest = np.array([-1.40905799, -2.29935204,  0.96482552,  0.26289107, -0.12981707, -1.2151303,
#                        1.62831697, -0.41679404,  1.70733658,  1.57549959, -0.2871302,  -0.45769692])
#
#    # Remove y and z degrees of freedom
#    q = np.delete(q, [1, 2])
#    q_rest = np.delete(q_rest, [1, 2])
#
#    # Define a mask to optimize only specific joints
#    # mask = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
#    mask = np.array([1, 1, 1, 0, 1, 0, 0, 0, 0, 0])
#    # Assign different joint weights based on the mode
#    if mode == "moveL":
#        joint_weights = np.array([1, 0.1, 1, 1, 1, 1, 1, 0.1, 1, 1])
#    elif mode == "moveu":
#        joint_weights = np.array([10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1])
#    else:
#        raise ValueError("Invalid mode. Use 'moveL' or 'moveu'.")
#
#    W = np.diag(joint_weights)
#
#    # Compute adaptive damping factor
#    manipulability = compute_manipulability(J[:, 2:])
#    lambda_adaptive = tikhonov_damp / manipulability
#
#    # Compute the weighted damped pseudo-inverse
#    JWJ = J @ np.linalg.inv(W) @ J.T + lambda_adaptive * np.eye(J.shape[0])
#    J_pseudo = np.linalg.inv(W) @ J.T @ np.linalg.inv(JWJ)
#
#    # Compute primary task velocity
#    qd_task = J_pseudo @ err_vector
#
#    d_min = 0.7  # Minimum allowed EE-base distance
#
#    if mode == "moveu":
#        d = compute_ee2basedistance(robot)
#
#        # If the EE-base distance is sufficient, return the computed velocity directly
#        if d >= d_min:
#            return qd_task
#
#        # # Compute the retreating direction for base movement
#        # base_pos = robot.getT_w_e().translation[:2]  # X, Y position of base
#        # ee_pos = q[:2]  # X, Y position of end-effector
#        # direction_vector = ee_pos - base_pos
#        # direction_vector /= np.linalg.norm(direction_vector)  # Normalize
#        # move_direction = np.sign(direction_vector[1])  # Determine whether to move forward or backward
#
#        # Compute null space projection
#        I = np.eye(J.shape[1])
#        N = I - J_pseudo @ J
#        z = np.zeros_like(qd_task)
#        z[0] = -0.8 * (d - d_min) # Control the base to retreat
#        qd_null = N @ z
#
#        # Combine primary task velocity and null space velocity
#        return qd_task + qd_null
#
#    # Compute the null space projection matrix
#    I = np.eye(J.shape[1])
#    N = I - J_pseudo @ J
#
#    # Compute the null space direction
#    z = -(q - q_rest)
#    z *= mask
#    qd_null = N @ z
#    # Combine primary task velocity and null space velocity
#    return qd_task + qd_null

## TODO: calculate nice q (in QP) as the secondary objective
## this requires getting the forward kinematics hessian,
## a.k.a jacobian derivative w.r.t. joint positions  dJ/dq .
## the ways to do it are as follows:
##   1) shitty and sad way to do it by computing dJ/dq \cdot \dot{q}
##      with unit velocities (qd) and then stacking that (very easy tho)
##   2) there is a function in c++ pinocchio for it getKinematicsHessian and you could write the pybind
##   3) you can write it yourself following peter corke's quide (he has a tutorial on fwd kinm derivatives)
##   4) figure out what pin.computeForwardKinematicDerivatives and pin.getJointAccelerationDerivatives
##      actually do and use that
## HA! found it in a winter school
## use this
## and test it with finite differencing!
# class CostManipulability:
#    def __init__(self, jointIndex=None, frameIndex=None):
#        if frameIndex is not None:
#            jointIndex = robot.model.frames[frameIndex].parent
#        self.jointIndex = (
#            jointIndex if jointIndex is not None else robot.model.njoints - 1
#        )
#
#    def calc(self, q):
#        J = self.J = pin.computeJointJacobian(
#            robot.model, robot.data, q, self.jointIndex
#        )
#        return np.sqrt(det(J @ J.T))
#
#    def calcDiff(self, q):
#        Jp = pinv(pin.computeJointJacobian(robot.model, robot.data, q, self.jointIndex))
#        res = np.zeros(robot.model.nv)
#        v0 = np.zeros(robot.model.nv)
#        for k in range(6):
#            pin.computeForwardKinematicsDerivatives(
#                robot.model, robot.data, q, Jp[:, k], v0
#            )
#            JqJpk = pin.getJointVelocityDerivatives(
#                robot.model, robot.data, self.jointIndex, pin.LOCAL
#            )[0]
#            res += JqJpk[k, :]
#        res *= self.calc(q)
#        return res
