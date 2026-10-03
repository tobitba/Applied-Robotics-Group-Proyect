from smc import load_config
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer

import time
import pinocchio as pin
import numpy as np
import meshcat_shapes


def phi(u: float) -> list[float]:
    """Cubic B-spline cumulative basis functions φ_i."""
    phi1 = u**3
    phi2 = 3 * u**2 * (1 - u)
    phi3 = 3 * u * (1 - u) ** 2
    phi4 = (1 - u) ** 3
    return [phi1, phi2, phi3, phi4]


def Omega(u: float) -> list[float]:
    """Cumulative weights Ω_i(u) = φ_i - φ_{i+1}."""
    phi1, phi2, phi3, phi4 = phi(u)
    Omega1 = phi1 - phi2
    Omega2 = phi2 - phi3
    Omega3 = phi3 - phi4
    Omega4 = phi4  # φ4 − φ5, φ5=0
    return [Omega1, Omega2, Omega3, Omega4]


def SE3_bspline(T0: pin.SE3, T1: pin.SE3, T2: pin.SE3, T3: pin.SE3, u: float):
    """
    Evaluate cubic SE(3) B-spline using cumulative formulation.
    u in [0,1]
    """
    # log in tangent space se(3)
    xi0 = pin.log(T0.inverse() * T1)
    xi1 = pin.log(T1.inverse() * T2)
    xi2 = pin.log(T2.inverse() * T3)

    # cumulative weights
    w1, w2, w3, w4 = Omega(u)

    # According to the cumulative formulation we use Ω1,Ω2,Ω3
    # and ignore Ω4 for exponentials (it corresponds to starting pose T0)
    T = T0 * pin.exp(w1 * xi0) * pin.exp(w2 * xi1) * pin.exp(w3 * xi2)

    return T


points = [pin.SE3.Random() for i in range(4)]
ts = np.linspace(0, 1, 200)
interpolated = []

for i, u in enumerate(ts):
    interpolated.append(SE3_bspline(points[0], points[1], points[2], points[3], u))


visualizer = MeshcatVisualizer()
meshcat_shapes.frame(visualizer.viewer["T1"], opacity=0.5)
meshcat_shapes.frame(visualizer.viewer["T2"], opacity=0.5)
meshcat_shapes.frame(visualizer.viewer["T3"], opacity=0.5)
meshcat_shapes.frame(visualizer.viewer["T4"], opacity=0.5)
# meshcat_shapes.frame(visualizer.viewer["C"], opacity=0.5)
visualizer.viewer["T1"].set_transform(points[0].homogeneous)
visualizer.viewer["T2"].set_transform(points[1].homogeneous)
visualizer.viewer["T3"].set_transform(points[2].homogeneous)
visualizer.viewer["T4"].set_transform(points[3].homogeneous)
# visualizer.viewer["C"].set_transform(C.homogeneous)
time.sleep(10)
visualizer.addFramePath("", interpolated, every_nth_to_plot=1)
visualizer.addFramePath("", interpolated, every_nth_to_plot=1)


time.sleep(100)
