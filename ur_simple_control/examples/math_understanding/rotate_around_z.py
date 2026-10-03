from smc import load_config
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer

import time
import pinocchio as pin
import numpy as np
import meshcat_shapes

A = pin.SE3.Identity()
B = pin.SE3.Identity()
theta_z = np.random.random()
B.rotation = pin.rpy.rpyToMatrix(0.0, 0.0, theta_z)
B.translation += np.random.random(3)

interpolated = []
t_line = np.linspace(0, 1, 20)
for t in t_line:
    interpolated.append(pin.SE3.Interpolate(A, B, t))

visualizer = MeshcatVisualizer()
meshcat_shapes.frame(visualizer.viewer["A"], opacity=0.5)
meshcat_shapes.frame(visualizer.viewer["B"], opacity=0.5)
# meshcat_shapes.frame(visualizer.viewer["C"], opacity=0.5)
visualizer.viewer["A"].set_transform(A.homogeneous)
visualizer.viewer["B"].set_transform(B.homogeneous)
# visualizer.viewer["C"].set_transform(C.homogeneous)

visualizer.addFramePath("", interpolated)
visualizer.addFramePath("", interpolated)


time.sleep(100)
