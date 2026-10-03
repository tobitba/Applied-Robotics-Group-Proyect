from smc.motion_planning.trajectory_generation.trajectory_generator import (
    AbstractTimeLaw,
)
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
import numpy as np
import pinocchio as pin


class UniformTimeLawWithEEContraints(AbstractTimeLaw):

    ## Find the scaling that enforces |velocity| ≤ 1 component-wise
    ## We sample along the path, compute X_u, and find the max absolute component.
    def __init__(
        self,
        smooth_path: AbstractInterpolator,
        v_max=np.ones(6),  # |V_j| <= v_max[j]
        a_min=-2 * np.ones(6),  # NEW: lower bound on acceleration
        a_max=0.5 * np.ones(6),  # NEW: upper bound on acceleration
        n_samples=200,
    ):
        """
        Compute constant u_dot (hence total time) so that:
            |V_j(t)| <= v_max[j]
            a_min[j] <= A_j(t) <= a_max[j]
        hold component-wise along the entire trajectory, with
            V = u_dot * X_u
            A = u_dot^2 * X_uu  (since u_ddot = 0).
        """
        v_max = np.asarray(v_max).reshape(
            6,
        )
        a_min = np.asarray(a_min).reshape(
            6,
        )
        a_max = np.asarray(a_max).reshape(
            6,
        )

        max_Xu = np.zeros(6)  # max |X_u|
        max_pos_Xuu = np.zeros(6)  # max positive X_uu
        min_neg_Xuu = np.zeros(6)  # min (most negative) X_uu

        for k in range(n_samples + 1):
            u = k / n_samples
            T_k, X_u, X_uu = smooth_path.getReference(u)

            max_Xu = np.maximum(max_Xu, np.abs(X_u))

            # track extreme positive and negative second derivatives separately
            max_pos_Xuu = np.maximum(max_pos_Xuu, np.maximum(X_uu, 0.0))
            min_neg_Xuu = np.minimum(min_neg_Xuu, np.minimum(X_uu, 0.0))

        # Handle constant path: no motion, time arbitrary
        if (
            np.all(max_Xu == 0)
            and np.all(max_pos_Xuu == 0)
            and np.all(min_neg_Xuu == 0)
        ):
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
                    raise ValueError(
                        f"Infeasible: a_max[{j}] < 0 with positive curvature."
                    )
                s_candidates.append(np.sqrt(a_max[j] / max_pos_Xuu[j]))

            # Lower bound: X_uu < 0 ⇒ s^2 * X_uu >= a_min[j]
            # with X_uu < 0, a_min can be negative; inequality ⇒ s^2 <= a_min / X_uu
            if min_neg_Xuu[j] < 0 and a_min[j] > -np.inf:
                if a_min[j] > 0:
                    # impossible if a_min>0 but X_uu<0 (since s^2 X_uu <=0)
                    raise ValueError(
                        f"Infeasible: a_min[{j}] > 0 with negative curvature."
                    )
                s_candidates.append(
                    np.sqrt(a_min[j] / min_neg_Xuu[j])
                )  # both negative ⇒ ratio > 0

        if not s_candidates:
            # no effective constraints → pick something
            print("UNIFORM TIME LAW: big oops: didn't find anything")
            self.T_total = 1.0

        udot = min(s_candidates)  # u_dot
        self.T_total = 1.0 / udot  # since u ∈ [0,1]

    def getCurrentPathParameter(self, t: float) -> tuple[float, float, float]:
        # return s and sdot
        return t / self.T_total, 1 / self.T_total, 0.0


if __name__ == "__main__":
    from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
    import matplotlib.pyplot as plt

    import time

    # TODO: finish this refactor, it's broken now
    raise NotImplementedError("didn't refactor this main, oops ")

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
            ax = plt.subplot(6, 1, j + 1)
            ax.plot(times, twists[:, j])
            ax.axhline(1.0, linestyle="--", color="gray")
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
            ax = plt.subplot(6, 1, j + 1)
            ax.plot(times, accels[:, j], color="tab:red")
            ax.axhline(0.5, linestyle="--", color="gray")
            ax.axhline(-2.0, linestyle="--", color="gray")
            ax.set_ylabel(labels[j])
            ax.set_title(f"Acceleration component: {labels[j]}")

        plt.xlabel("time [s]")
        plt.tight_layout()

    # Example poses
    N = 5
    T_list = [pin.SE3.Random() for i in range(N)]
    for i in range(N):
        if i < N / 2:
            T_list[i].translation = np.array([i, 0, 0])
        else:
            T_list[i].translation = np.array([N / 2, i - N / 2, 0])
        T_list[i].rotation = np.eye(3)

    # Compute spline
    C_segments, t_list = fit_SE3_C2_bezier_spline(T_list)

    # interpolated = []
    # for i, (C0, C1, C2, C3) in enumerate(C_segments):
    #     for u in np.linspace(0, 1, 10):
    #         T = se3_bezier_casteljau(C0, C1, C2, C3, u)
    #         interpolated.append(T)

    interpolated = [
        evaluate_SE3_trajectory_global_u(C_segments, t_list, u)
        for u in np.linspace(0, 1, 50)
    ]

    visualizer = MeshcatVisualizer()

    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)
    # visualizer.addFramePath("", T_list, every_nth_to_plot=1)
    T_total, u_dot = computeTimeScalingEEVelConstraint(C_segments, t_list)
    print("Total duration:", T_total, "u_dot =", u_dot)
    plot_twist_over_time(T_total)
    plot_accel_over_time(T_total)
    plt.show()
    time.sleep(100)
