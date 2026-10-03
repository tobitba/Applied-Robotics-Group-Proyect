from pinocchio import SE3
import numpy as np

# TODO: detangle this mess

# TODO:
# replace scipy rotation and slerp with pinocchio to reduce number of dependecies.
# (as pinocchio has the exact same functionality)
from scipy.spatial.transform import Rotation, Slerp


def extract_positions_and_rots(
    poses: list[SE3],
) -> tuple[np.ndarray, list[Rotation]]:
    Ps = np.stack([p.translation for p in poses], axis=0)  # shape (N,3)
    rots = [Rotation.from_matrix(p.rotation) for p in poses]
    return Ps, rots


def compute_times_from_positions(
    positions: np.ndarray, total_time: float
) -> np.ndarray:
    diffs = np.linalg.norm(np.diff(positions, axis=0), axis=1)  # positions: (N,3)
    s = np.concatenate(([0.0], np.cumsum(diffs)))
    times = s / s[-1] * total_time
    return times


def compute_quintic_coeffs(
    p0: float,
    pT: float,
    v0: float,
    vT: float,
    a0: float,
    aT: float,
    traj_duration: float,
) -> np.ndarray:
    """
    p0, pT: initial and final positions
    v0, vT: initial and final velocities
    a0, aT: initial and final accelerations
    """

    T = traj_duration
    T2 = traj_duration**2
    T3 = traj_duration**3
    T4 = traj_duration**4
    T5 = traj_duration**5

    A = np.zeros((6, 6))
    b = np.zeros(6)
    # pos(0) = a0
    A[0, :] = [1, 0, 0, 0, 0, 0]
    b[0] = p0
    # pos(T)
    A[1, :] = [1, T, T2, T3, T4, T5]
    b[1] = pT
    # vel(0) = a1
    A[2, :] = [0, 1, 0, 0, 0, 0]
    b[2] = v0
    # vel(T)
    A[3, :] = [0, 1, 2 * T, 3 * T2, 4 * T3, 5 * T4]
    b[3] = vT
    # acc(0) = 2*a2
    A[4, :] = [0, 0, 2, 0, 0, 0]
    b[4] = a0
    # acc(T)
    A[5, :] = [0, 0, 2, 6 * T, 12 * T2, 20 * T3]
    b[5] = aT
    coeffs = np.linalg.solve(A, b)
    return coeffs  # length 6


def eval_quintic_and_derivative(coeffs, t):
    a0, a1, a2, a3, a4, a5 = coeffs
    t2 = t**2
    t3 = t**3
    t4 = t**4
    t5 = t**5
    pos = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5
    vel = a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4
    return pos, vel


def compute_desired_twist_from_quintic(
    Ps: np.ndarray, Rots: list[Rotation], times: np.ndarray, at_time_t: float
) -> SE3:
    total_time = times[-1]

    # velocities from finite difference: v0 ~ (p1 - p0)/(t1 - t0)
    assert (
        len(Ps) >= 2
    ), f"Finite difference calculation requires at least two points"  # do not think this is necessary if path_planner is doing its job
    vel_initial = (Ps[1] - Ps[0]) / max(1e-8, (times[1] - times[0]))
    vel_final = (Ps[-1] - Ps[-2]) / max(1e-8, (total_time - times[-2]))

    acc_initial = np.zeros(3)
    acc_final = np.zeros(3)

    # (I'd like to vectorize computing coefficients currently being done per axis)
    coeffs_axes = []
    for axis in range(3):  # XYZ
        coeffs = compute_quintic_coeffs(
            p0=Ps[0, axis],
            pT=Ps[-1, axis],
            v0=vel_initial[axis],
            vT=vel_final[axis],
            a0=acc_initial[axis],
            aT=acc_final[axis],
            traj_duration=total_time,
        )
        coeffs_axes.append(coeffs)
    coeffs_axes = np.array(coeffs_axes)  # shape (3,6)

    pos_eval = np.array(
        [eval_quintic_and_derivative(coeffs_axes[i], at_time_t)[0] for i in range(3)]
    )

    slerp = Slerp(times, Rotation.concatenate(Rots))
    rotation_at_time_t = slerp(at_time_t)

    desired_SE = SE3(rotation_at_time_t.as_matrix(), pos_eval)

    return desired_SE
