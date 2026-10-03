import meshcat_shapes
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
import time
from pinocchio import SE3
from smc.motion_planning.interpolation.bsplinesSE3 import CubicBezierSplineSE3
import numpy as np

if __name__ == "__main__":

    # Example poses
    visualizer = MeshcatVisualizer()
    T_list = [SE3.Random() for _ in range(10)]
    V_start = np.random.random(6) / 4
    V_end = np.random.random(6) / 4
    for i, T in enumerate(T_list):
        meshcat_shapes.frame(visualizer.viewer["T" + str(i)], opacity=1.0)
        visualizer.viewer["T" + str(i)].set_transform(T.homogeneous)
    time.sleep(6)

    # Compute spline
    start = time.time()
    bsplineSE3 = CubicBezierSplineSE3(T_list, V_start, V_end)
    end = time.time()
    print("bsplines computed in:", end - start, "seconds")

    interpolated = []
    for s in np.linspace(0, 1, 1000):
        T = bsplineSE3.getPoint(s)
        interpolated.append(T)

    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)
    visualizer.addFramePath("", interpolated, every_nth_to_plot=1)

    time.sleep(100)
