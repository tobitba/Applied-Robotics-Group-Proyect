from smc import load_config
import numpy as np
import pinocchio as pin
from numpy.linalg import solve
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
import time
import meshcat_shapes


# =======================================
# de Casteljau algorithm on SE(3)
# =======================================


def se3_bezier_casteljau(C0, C1, C2, C3, u):
    """Evaluate cubic Bézier on SE(3) using geodesic de Casteljau."""
    # Level 1
    L01 = pin.SE3.Interpolate(C0, C1, u)
    L12 = pin.SE3.Interpolate(C1, C2, u)
    L23 = pin.SE3.Interpolate(C2, C3, u)

    # Level 2
    M012 = pin.SE3.Interpolate(L01, L12, u)
    M123 = pin.SE3.Interpolate(L12, L23, u)

    # Level 3
    B = pin.SE3.Interpolate(M012, M123, u)
    return B


# =======================================
# Build the tridiagonal system for V_i
# =======================================


def compute_node_velocities_SE3(T_list, t_list):
    """
    Compute left-trivialized node velocities V_i (6-vectors)
    to make a C^2 cubic spline in SE(3).
    Natural boundary conditions V_0 = V_N = 0.
    """
    N = len(T_list) - 1  # segments
    h = np.array([t_list[i + 1] - t_list[i] for i in range(N)])

    # Compute secant twists Δ_i
    Delta = []
    for i in range(N):
        Xi = pin.log(T_list[i].inverse() * T_list[i + 1]) / h[i]
        Delta.append(Xi)
    Delta = np.stack(Delta)  # shape (N, 6)

    # Build tridiagonal system A V = R for V_1..V_{N-1}
    # Natural boundary: V_0 = 0, V_N = 0
    # TODO: construct straight line to the last point instead
    if N < 2:
        raise ValueError("Need at least 3 poses for C^2 spline.")

    # Matrix A is (N-1)x(N-1)
    A = np.zeros((N - 1, N - 1))
    R = np.zeros((N - 1, 6))

    for i in range(1, N):  # interior nodes i=1..N-1
        row = i - 1
        him1 = h[i - 1]
        hi = h[i] if i < N else None

        if i - 1 > 0:
            A[row, row - 1] = him1
        A[row, row] = 2 * (him1 + hi)
        if i < N - 1:
            A[row, row + 1] = hi

        # RHS for this node:
        RHSi = 3 * (him1 * Delta[i] + hi * Delta[i - 1])
        R[row, :] = RHSi

    # Solve for V_1..V_{N-1}
    V_internal = solve(A, R)  # shape (N-1, 6)

    # Assemble full V
    V = np.zeros((N + 1, 6))
    V[1:N] = V_internal  # internal nodes
    # boundaries remain zero
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
    V = compute_node_velocities_SE3(T_list, t_list)
    C_segments = bezier_control_poses_SE3(T_list, V, t_list)
    return C_segments


# =======================================
# Example usage
# =======================================


if __name__ == "__main__":
    visualizer = MeshcatVisualizer()
    # Example poses
    T_list = [pin.SE3.Random() for i in range(10)]
    for i, T in enumerate(T_list):
        meshcat_shapes.frame(visualizer.viewer["T" + str(i)], opacity=1.0)
        visualizer.viewer["T" + str(i)].set_transform(T.homogeneous)
    time.sleep(6)

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

    interpolated = []
    for i, (C0, C1, C2, C3) in enumerate(C_segments):
        for u in np.linspace(0, 1, 100):
            T = se3_bezier_casteljau(C0, C1, C2, C3, u)
            interpolated.append(T)

    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)
    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)

    time.sleep(100)
