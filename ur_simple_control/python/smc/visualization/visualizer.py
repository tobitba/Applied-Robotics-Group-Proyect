from smc.bookkeeping.base_config import ConfigBase
import numpy as np
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer

# TODO: this causes a circular import somehow
from pinocchio.visualize import MeshcatVisualizer as UnWrappedMeshcat
import meshcat_shapes
import pinocchio as pin
from multiprocessing import Queue
from typing import Any

# tkinter stuff for later reference
# from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
# from tkinter import *
# from tkinter.ttk import *

# TODO: we want to swap to viser asap


def manipulatorVisualizer(
    cfg: ConfigBase,
    cmd: dict[str, Any],
    queue: Queue,
) -> None:
    if "model" not in cmd.keys():
        cfg.visualizer = False
        cfg.plotter = False
        cfg.real = False
        cfg.start_from_current_pose = False
        from smc.robots.utils import getRobotFromConfig

        robot = getRobotFromConfig(cfg)
        model = robot._model
        collision_model = robot._collision_model
        visual_model = robot._visual_model
    else:
        model = cmd["model"]
        collision_model = cmd["collision_model"]
        visual_model = cmd["visual_model"]
    viz = MeshcatVisualizer(
        model=model, collision_model=collision_model, visual_model=visual_model
    )

    viz.loadViewerModel()
    # display the initial pose
    viz.display(cmd["q"])
    # TODO: load shapes depending on robot type
    # set shapes we might use
    meshcat_shapes.frame(viz.viewer["Mgoal"], opacity=0.5)
    meshcat_shapes.frame(viz.viewer["T_w_e"], opacity=0.5)
    meshcat_shapes.frame(viz.viewer["T_w_l"], opacity=0.5)
    meshcat_shapes.frame(viz.viewer["T_w_r"], opacity=0.5)
    meshcat_shapes.frame(viz.viewer["T_base"], opacity=0.5)
    print("MANIPULATORVISUALIZER: FULLY ONLINE")
    try:
        while True:
            cmd = queue.get()
            for key in cmd:
                if key == "befree":
                    if cfg.debug_prints:
                        print(
                            "MANIPULATORVISUALIZER: got befree, manipulatorVisualizer out"
                        )
                    viz.viewer.window.server_proc.kill()
                    viz.viewer.window.server_proc.wait()
                    break
                if key == "Mgoal":
                    viz.viewer["Mgoal"].set_transform(cmd["Mgoal"].homogeneous)
                if key == "T_w_e":
                    viz.viewer["T_w_e"].set_transform(cmd["T_w_e"].homogeneous)
                if key == "T_w_l":
                    viz.viewer["T_w_l"].set_transform(cmd["T_w_l"].homogeneous)
                if key == "T_w_r":
                    viz.viewer["T_w_r"].set_transform(cmd["T_w_r"].homogeneous)
                if key == "T_base":
                    viz.viewer["T_base"].set_transform(cmd["T_base"].homogeneous)
                if key == "q":
                    viz.display(cmd["q"])
                if key == "point":
                    viz.addPoint(cmd["point"])
                if key == "obstacle_sphere":
                    # stupid and evil but there is no time
                    viz.addSphereObstacle(
                        cmd["obstacle_sphere"][0], cmd["obstacle_sphere"][1]
                    )
                if key == "obstacle_box":
                    # stupid and evil but there is no time
                    viz.addBoxObstacle(cmd["obstacle_box"][0], cmd["obstacle_box"][1])
                if "ellipsoid" in key:
                    # stupid and evil but there is no time
                    viz.addEllipsoid(key, cmd[key][0], cmd[key][1], cmd[key][2])
                if "path" in key:
                    # stupid and evil but there is no time
                    if not "frame" in key:
                        viz.addPath(key, cmd[key])
                    else:
                        # stupid and evil but there is no time
                        viz.addFramePath(key, cmd[key])
                if key == "SE3waypoints":
                    for i, T in enumerate(cmd[key]):
                        meshcat_shapes.frame(viz.viewer["T" + str(i)], opacity=1.0)
                        viz.viewer["T" + str(i)].set_transform(T.homogeneous)

    except KeyboardInterrupt:
        if cfg.debug_prints:
            print("MANIPULATORVISUALIZER: caught KeyboardInterrupt, i'm out")
        viz.viewer.window.server_proc.kill()
        viz.viewer.window.server_proc.wait()


# TODO: put in as option.
# need to revert colors otherwise.
# also, repainting all the time is mega slow,
# that's not a option.
# so has to be triggered only upon a change.
# TODO: also, obv need to remove dependency on the robot
def colorCollisionsRed(robot, viz):
    pin.computeCollisions(
        robot._model,
        robot._data,
        robot._collision_model,
        robot.collision_data,
        robot._q,
        False,
    )

    # 2. Iterate through collision pairs
    for i, pair in enumerate(robot.collision_model.collisionPairs):
        # Get the names of the two geometries in the pair
        geom1_name = robot.collision_model.geometryObjects[pair.first].name
        geom2_name = robot.collision_model.geometryObjects[pair.second].name

        # Path in Meshcat is usually "pinocchio/collisions/<geom_name>"
        node1 = viz.viewer["pinocchio"]["collisions"][geom1_name]
        node2 = viz.viewer["pinocchio"]["collisions"][geom2_name]

        if robot.collision_data.collisionResults[i].isCollision():
            # Paint them Red if colliding
            node1.set_property("color", [1.0, 0.0, 0.0, 1.0])
            node2.set_property("color", [1.0, 0.0, 0.0, 1.0])
        # else:
        #    # Reset to Gray if not
        #    node1.set_property("color", [0.7, 0.7, 0.7, 0.5])
        #    node2.set_property("color", [0.7, 0.7, 0.7, 0.5])


# could be merged with the above function.
# but they're different enough in usage to warrent a different function,
# instead of polluting the above one with ifs
def manipulatorComparisonVisualizer(
    model, collision_model, visual_model, cfg, cmd, cmd_queue, ack_queue
):
    for geom in visual_model.geometryObjects:
        if "hand" in geom.name:
            geom.meshColor = np.array([0.2, 0.2, 0.2, 0.2])
    viz = MeshcatVisualizer(
        model=model, collision_model=collision_model, visual_model=visual_model
    )
    viz.initViewer(open=True)
    # load the first one
    viz.loadViewerModel(collision_color=[0.2, 0.2, 0.2, 0.6])
    # viz.viewer["pinocchio/visuals"].set_property("visible", False)
    # viz.viewer["pinocchio/collisions"].set_property("visible", True)
    viz.displayVisuals(False)
    viz.displayCollisions(True)
    # maybe needed
    # viz.displayVisuals(True)
    #    meshpath = viz.viewerVisualGroupName
    #    for geom in visual_model(6):
    #        meshpath_i = meshpath + "/" + geometry_object.name
    # viz.viewer["pinocchio/visuals"].set_property("opacity", 0.2)
    # viz.viewer["pinocchio"].set_property("opacity", 0.2)
    # viz.viewer["pinocchio/visuals/forearm_link_0"].set_property("opacity", 0.2)

    # viz.viewer["pinocchio"].set_property("color", (0.2,0.2,0.2,0.2))
    # viz.viewer["pinocchio/visuals"].set_property("color",(0.2,0.2,0.2,0.2))
    # viz.viewer["pinocchio/visuals/forearm_link_0"].set_property("color", (0.2,0.2,0.2,0.2))

    ## this is the path we want, with the /<object> at the end
    # node = viz.viewer["pinocchio/visuals/forearm_link_0/<object>"]
    # print(node)
    # node.set_property("opacity", 0.2)
    # node.set_property("modulated_opacity", 0.2)
    # node.set_property("color", [0.2] * 4)
    # node.set_property("scale", 100.2)
    # this one actually works
    # node.set_property("visible", False)

    node = viz.viewer["pinocchio/visuals"]
    # node.set_property("visible", False)
    node.set_property("modulated_opacity", 0.4)
    node.set_property("opacity", 0.2)
    node.set_property("color", [0.2] * 4)

    # meshcat->SetProperty("path/to/my/thing/<object>", "opacity", alpha);

    # other robot display
    viz2 = UnWrappedMeshcat(
        model=model, collision_model=collision_model, visual_model=visual_model
    )
    viz2.initViewer(viz.viewer)
    # i don't know if rootNodeName does anything apart from being different
    # viz2.loadViewerModel(rootNodeName="pinocchio2", visual_color=(1.0,1.0,1.0,0.1))
    viz2.loadViewerModel(rootNodeName="pinocchio2")
    # initialize
    q1, q2 = cmd_queue.get()
    viz.display(q1)
    viz2.display(q2)

    ack_queue.put("ready")
    print("MANIPULATOR_COMPARISON_VISUALIZER: FULLY ONLINE")
    try:
        while True:
            q = cmd_queue.get()
            if type(q) == str:
                print("got str q")
                if q == "befree":
                    if cfg.debug_prints:
                        print(
                            "MANIPULATOR_COMPARISON_VISUALIZER: got befree, manipulatorComparisonVisualizer out"
                        )
                    viz.viewer.window.server_proc.kill()
                    viz.viewer.window.server_proc.wait()
                    break
            q1, q2 = q
            print(q)
            viz.display(q1)
            viz2.display(q2)
            # this doesn't really work because meshcat is it's own server
            # and display commands just relay their command and return immediatelly.
            # but it's better than nothing.
            # NOTE: if there's lag in meshcat, just add a small sleep here before the
            # ack signal - that will ensure synchronization because meshat will actually be ready
            ack_queue.put("ready")
    except KeyboardInterrupt:
        if cfg.debug_prints:
            print(
                "MANIPULATOR_COMPARISON_VISUALIZER: caught KeyboardInterrupt, i'm out"
            )
        viz.viewer.window.server_proc.kill()
        viz.viewer.window.server_proc.wait()
