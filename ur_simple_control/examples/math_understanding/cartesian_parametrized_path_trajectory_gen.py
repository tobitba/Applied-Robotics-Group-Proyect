from smc import load_config
import numpy as np
import pinocchio as pin
from numpy.linalg import solve
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
import time
import meshcat_shapes
import matplotlib.pyplot as plt
from smc.path_generation.interpolation.bsplinesSE3 import se3_bezier_casteljau



def compute_node_velocities_SE3_with_endpoints(T_list, t_list, V0, VN):
    """
    Compute node velocities V_i (6D twists) for a C^2 cubic spline in SE(3),
    with *user-specified* boundary velocities:

        V_0 = V0  (given)
        V_N = VN  (given)

    Parameters
    ----------
    T_list : list of pin.SE3
        Poses T_0 .. T_N.
    t_list : list or array of float
        Times t_0 .. t_N (strictly increasing).
    V0 : array-like, shape (6,)
        Desired twist at the first node.
    VN : array-like, shape (6,)
        Desired twist at the last node.

    Returns
    -------
    V : ndarray, shape (N+1, 6)
        Node velocities [V_0, ..., V_N].
    """
    N = len(T_list) - 1  # number of segments
    if N < 2:
        raise ValueError("Need at least 3 poses for C^2 spline.")

    V0 = np.asarray(V0).reshape(6,)
    VN = np.asarray(VN).reshape(6,)

    # Segment durations
    h = np.array([t_list[i + 1] - t_list[i] for i in range(N)])
    if np.any(h <= 0):
        raise ValueError("t_list must be strictly increasing.")

    # Secant twists Δ_i
    Delta = []
    for i in range(N):
        Xi = pin.log(T_list[i].inverse() * T_list[i + 1]) / h[i]
        Delta.append(Xi)
    Delta = np.stack(Delta)  # shape (N, 6)

    # Build system for internal velocities V_1..V_{N-1}
    A = np.zeros((N - 1, N - 1))
    R = np.zeros((N - 1, 6))

    for i in range(1, N):  # interior node index i = 1..N-1
        row = i - 1
        him1 = h[i - 1]          # h_{i-1}
        hi   = h[i] if i < N else None  # h_i

        # Tridiagonal coefficients for V_{i-1}, V_i, V_{i+1}
        if i - 1 > 0:
            A[row, row - 1] = him1           # coefficient of V_{i-1} (for i>=2)
        A[row, row] = 2 * (him1 + hi)        # coefficient of V_i
        if i < N - 1:
            A[row, row + 1] = hi             # coefficient of V_{i+1} (for i<=N-2)

        # Base RHS from spline C^2 condition
        RHSi = 3 * (him1 * Delta[i] + hi * Delta[i - 1])

        # Adjust RHS for known endpoints V0 and VN
        if i == 1:
            # equation for node 1 includes h_0 * V_0
            RHSi = RHSi - him1 * V0
        if i == N - 1:
            # equation for node N-1 includes h_{N-1} * V_N
            RHSi = RHSi - hi * VN

        R[row, :] = RHSi

    # Solve for internal velocities V_1..V_{N-1}
    V_internal = solve(A, R)  # shape (N-1, 6)

    # Assemble full velocity array
    V = np.zeros((N + 1, 6))
    V[0, :] = V0
    V[1:N, :] = V_internal
    V[N, :] = VN

    return V


# =======================================
# Build cubic Bézier control poses
# =======================================


def bezier_control_poses_SE3(T_list, V, t_list):
    """
    Given poses T_i and twist velocities V_i, construct cubic Bézier
    control poses C_{i,0..3} for each segment i.
    Returns list of (C0, C1, C2, C3).
    """
    N = len(T_list) - 1
    C_segments = []

    for i in range(N):
        T_i = T_list[i]
        T_ip1 = T_list[i + 1]
        h_i = t_list[i + 1] - t_list[i]

        C0 = T_i
        C1 = T_i * pin.exp((h_i / 3.0) * V[i])
        C2 = T_ip1 * pin.exp(-(h_i / 3.0) * V[i + 1])
        C3 = T_ip1

        C_segments.append((C0, C1, C2, C3))

    return C_segments


# =======================================
# Full pipeline: SE(3) C^2 Bézier spline
# =======================================


def SE3_C2_bezier_spline(T_list, t_list):
    V0 = np.array([0.25, 0.5, 0, 0, 0, 0.0])            # start 
    V0 = V0/np.linalg.norm(V0)
    VN = np.array([0, 0, 0, 0, 0, 0])  # end
    V = compute_node_velocities_SE3_with_endpoints(T_list, t_list, V0, VN)
    C_segments = bezier_control_poses_SE3(T_list, V, t_list)
    return C_segments



## One u in [0, 1] for entire geometric path
def evaluate_SE3_trajectory_global_u(C_segments, t_list, u):
    # clamp u
    u = max(0.0, min(1.0, float(u)))

    t0 = t_list[0]
    tN = t_list[-1]
    total_T = tN - t0
    if total_T <= 0:
        raise ValueError("t_list must be strictly increasing.")

    t = t0 + u * total_T

    if t >= tN:
        seg_idx = len(t_list) - 2
        C0, C1, C2, C3 = C_segments[seg_idx]
        return se3_bezier_casteljau(C0, C1, C2, C3, 1.0)

    N = len(t_list) - 1
    seg_idx = None
    for i in range(N):
        if t_list[i] <= t < t_list[i + 1]:
            seg_idx = i
            break
    if seg_idx is None:
        seg_idx = N - 1

    ti = t_list[seg_idx]
    tip1 = t_list[seg_idx + 1]
    hi = tip1 - ti
    if hi <= 0:
        raise ValueError("t_list must be strictly increasing.")

    u_local = (t - ti) / hi

    C0, C1, C2, C3 = C_segments[seg_idx]
    return se3_bezier_casteljau(C0, C1, C2, C3, u_local)

## Compute the path derivative
def se3_tangent_wrt_u(C_segments, t_list, u, eps=1e-4):
    """
    Approximate left-trivialized tangent X_u(u) (6D twist per unit u)
    using finite differences.
    """
    u = float(u)
    u_minus = max(0.0, u - eps)
    u_plus  = min(1.0, u + eps)

    T_mid   = evaluate_SE3_trajectory_global_u(C_segments, t_list, u)
    T_plus  = evaluate_SE3_trajectory_global_u(C_segments, t_list, u_plus)
    T_minus = evaluate_SE3_trajectory_global_u(C_segments, t_list, u_minus)

    if u_plus > u and u_minus < u:
        # central difference
        Xi_plus  = pin.log(T_mid.inverse() * T_plus)
        Xi_minus = pin.log(T_mid.inverse() * T_minus)
        X_u = (Xi_plus - Xi_minus) / (u_plus - u_minus)
    elif u_plus > u:  # near u=0
        Xi = pin.log(T_mid.inverse() * T_plus)
        X_u = Xi / (u_plus - u)
    else:  # near u=1
        Xi = pin.log(T_minus.inverse() * T_mid)
        X_u = Xi / (u - u_minus)

    return np.array(X_u).reshape(6,)

## Compute the path second derivative
def se3_second_tangent_wrt_u(C_segments, t_list, u, eps=1e-3):
    """
    Approximate second derivative X_uu(u) ≈ d^2T/du^2 (6D) using finite
    difference on X_u.
    """
    u = float(u)
    u_minus = max(0.0, u - eps)
    u_plus  = min(1.0, u + eps)

    X_u_plus  = se3_tangent_wrt_u(C_segments, t_list, u_plus)
    X_u_minus = se3_tangent_wrt_u(C_segments, t_list, u_minus)

    if u_plus > u_minus:
        X_uu = (X_u_plus - X_u_minus) / (u_plus - u_minus)
    else:
        X_uu = np.zeros(6)

    return X_uu

## Find the scaling that enforces |velocity| ≤ 1 component-wise
## We sample along the path, compute X_u, and find the max absolute component.
def compute_time_scaling(
    C_segments, t_list,
    v_max=np.ones(6),        # |V_j| <= v_max[j]
    a_min=-2*np.ones(6),     # NEW: lower bound on acceleration
    a_max=0.5*np.ones(6),    # NEW: upper bound on acceleration
    n_samples=200
):
    """
    Compute constant u_dot (hence total time) so that:
        |V_j(t)| <= v_max[j]
        a_min[j] <= A_j(t) <= a_max[j]
    hold component-wise along the entire trajectory, with
        V = u_dot * X_u
        A = u_dot^2 * X_uu  (since u_ddot = 0).
    """
    v_max = np.asarray(v_max).reshape(6,)
    a_min = np.asarray(a_min).reshape(6,)
    a_max = np.asarray(a_max).reshape(6,)

    max_Xu = np.zeros(6)          # max |X_u|
    max_pos_Xuu = np.zeros(6)     # max positive X_uu
    min_neg_Xuu = np.zeros(6)     # min (most negative) X_uu

    for k in range(n_samples + 1):
        u = k / n_samples
        X_u  = se3_tangent_wrt_u(C_segments, t_list, u)
        X_uu = se3_second_tangent_wrt_u(C_segments, t_list, u)

        max_Xu = np.maximum(max_Xu, np.abs(X_u))

        # track extreme positive and negative second derivatives separately
        max_pos_Xuu = np.maximum(max_pos_Xuu, np.maximum(X_uu, 0.0))
        min_neg_Xuu = np.minimum(min_neg_Xuu, np.minimum(X_uu, 0.0))

    # Handle constant path: no motion, time arbitrary
    if np.all(max_Xu == 0) and np.all(max_pos_Xuu == 0) and np.all(min_neg_Xuu == 0):
        return 1.0

    s_candidates = []  # s = u_dot

    # --- Velocity constraints: |s * X_u| <= v_max ---
    for j in range(6):
        if max_Xu[j] > 0 and v_max[j] < np.inf:
            s_candidates.append(v_max[j] / max_Xu[j])

    # --- Acceleration constraints: a_min <= s^2 * X_uu <= a_max ---
    for j in range(6):
        # Upper bound: X_uu > 0 ⇒ s^2 * X_uu <= a_max[j] ⇒ s^2 <= a_max[j]/X_uu
        if max_pos_Xuu[j] > 0 and a_max[j] < np.inf:
            if a_max[j] < 0:
                # impossible constraint if a_max < 0 and X_uu > 0
                raise ValueError(f"Infeasible: a_max[{j}] < 0 with positive curvature.")
            s_candidates.append(np.sqrt(a_max[j] / max_pos_Xuu[j]))

        # Lower bound: X_uu < 0 ⇒ s^2 * X_uu >= a_min[j]
        # with X_uu < 0, a_min can be negative; inequality ⇒ s^2 <= a_min / X_uu
        if min_neg_Xuu[j] < 0 and a_min[j] > -np.inf:
            if a_min[j] > 0:
                # impossible if a_min>0 but X_uu<0 (since s^2 X_uu <=0)
                raise ValueError(f"Infeasible: a_min[{j}] > 0 with negative curvature.")
            s_candidates.append(np.sqrt(a_min[j] / min_neg_Xuu[j]))  # both negative ⇒ ratio > 0

    if not s_candidates:
        # no effective constraints → pick something
        return 1.0

    s = min(s_candidates)   # u_dot
    T_total = 1.0 / s       # since u ∈ [0,1]
    return T_total, s

def evaluate_SE3_trajectory_time(C_segments, t_list, T_total, t):
    """
    Evaluate trajectory at physical time t ∈ [0, T_total].
    """
    t = float(t)
    t = max(0.0, min(T_total, t))
    u = t / T_total
    return evaluate_SE3_trajectory_global_u(C_segments, t_list, u)

def twist_at_time(C_segments, t_list, T_total, t, eps_u=1e-4):
    """
    Compute time-parameterized twist V(t), with |V_j| ≤ 1.
    """
    t = float(t)
    t = max(0.0, min(T_total, t))
    u = t / T_total
    X_u = se3_tangent_wrt_u(C_segments, t_list, u, eps=eps_u)
    u_dot = 1.0 / T_total   # constant
    V_t = u_dot * X_u       # twist per unit time
    return V_t  # 6D, each component ∈ [-1,1] (up to numeric error)

def accel_at_time(C_segments, t_list, T_total, t, eps_u=1e-3):
    t = max(0.0, min(T_total, float(t)))
    u = t / T_total
    X_u  = se3_tangent_wrt_u(C_segments, t_list, u, eps=eps_u)
    X_uu = se3_second_tangent_wrt_u(C_segments, t_list, u, eps=eps_u)
    s = 1.0 / T_total
    # A = s^2 X_uu, since u_ddot = 0
    return (s**2) * X_uu

def plot_twist_over_time(T_total):
    n_samples = 300
    times = np.linspace(0.0, T_total, n_samples)
    twists = np.zeros((n_samples, 6))

    for i, t in enumerate(times):
        twists[i, :] = twist_at_time(C_segments, t_list, T_total, t)

    max_abs = np.max(np.abs(twists))
    print("Max |twist component| over trajectory =", max_abs)

    labels = ["vx", "vy", "vz", "ωx", "ωy", "ωz"]

    plt.figure(figsize=(8, 10))
    for j in range(6):
        ax = plt.subplot(6, 1, j+1)
        ax.plot(times, twists[:, j])
        ax.axhline( 1.0, linestyle="--", color="gray")
        ax.axhline(-1.0, linestyle="--", color="gray")
        ax.set_ylabel(labels[j])
        ax.set_title(f"Twist component: {labels[j]}")
    
    plt.xlabel("time [s]")
    plt.tight_layout()
    

def plot_accel_over_time(T_total):
    n_samples = 300
    times = np.linspace(0.0, T_total, n_samples)
    accels = np.zeros((n_samples, 6))

    for i, t in enumerate(times):
        accels[i, :] = accel_at_time(C_segments, t_list, T_total, t)

    max_abs = np.max(np.abs(accels))
    print("Max |accel component| over trajectory =", max_abs)

    labels = ["ax", "ay", "az", "ωdotx", "ωdoty", "ωdotz"]

    plt.figure(figsize=(8, 10))
    for j in range(6):
        ax = plt.subplot(6, 1, j+1)
        ax.plot(times, accels[:, j], color="tab:red")
        ax.axhline( 0.5, linestyle="--", color="gray")
        ax.axhline(-2.0, linestyle="--", color="gray")
        ax.set_ylabel(labels[j])
        ax.set_title(f"Acceleration component: {labels[j]}")
    
    plt.xlabel("time [s]")
    plt.tight_layout()
    

# =======================================
# Example usage
# =======================================


if __name__ == "__main__":
    # Example poses
    N = 5
    T_list = [pin.SE3.Random() for i in range(N)]
    for i in range(N):
        if i < N/2:
            T_list[i].translation = np.array([i, 0, 0])
        else:
            T_list[i].translation = np.array([N/2, i-N/2, 0])
        T_list[i].rotation = np.eye(3)
    # Chord-length parameterization
    t_list = [0.0]
    for i in range(len(T_list) - 1):
        d = np.linalg.norm(T_list[i + 1].translation - T_list[i].translation)
        t_list.append(t_list[-1] + d)
    
    # Compute spline
    start = time.time()
    C_segments = SE3_C2_bezier_spline(T_list, t_list)
    end = time.time()
    print("bsplines computed in:", end - start, "seconds")

    # interpolated = []
    # for i, (C0, C1, C2, C3) in enumerate(C_segments):
    #     for u in np.linspace(0, 1, 10):
    #         T = se3_bezier_casteljau(C0, C1, C2, C3, u)
    #         interpolated.append(T)
    
    interpolated = [evaluate_SE3_trajectory_global_u(C_segments, t_list, u) for u in np.linspace(0, 1, 50)]
    
    visualizer = MeshcatVisualizer()

    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)
    # visualizer.addFramePath("", T_list, every_nth_to_plot=1)
    T_total, u_dot = compute_time_scaling(C_segments, t_list)
    print("Total duration:", T_total, "u_dot =", u_dot)
    plot_twist_over_time(T_total)
    plot_accel_over_time(T_total)
    plt.show()
    time.sleep(100)
