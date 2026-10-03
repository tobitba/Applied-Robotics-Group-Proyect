from smc import load_config, getRobotFromConfig
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI
from smc.control.cartesian_space.cartesian_space_point_to_point import moveLDualArm
from smc.robots.interfaces.whole_body_single_arm_interface import (
    SingleArmWholeBodyInterface,
)
from smc.visualization.meshcat_viewer_wrapper.visualizer import MeshcatVisualizer
from smc.robots.extend_robot_model import addCartIntoVisualizationAndCollision

import numpy as np
import argparse
import pinocchio as pin
import time


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


if __name__ == "__main__":
    cfg = load_config()
    cfg.visualizer = False
    robot = getRobotFromConfig(cfg)
    robot._q = robot._comfy_configuration
    robot._step()
    robot._step()
    # add cart to model
    addCartIntoVisualizationAndCollision(robot)

    #    from IPython import embed
    #
    #    embed()

    # set up collision pairs

    # Add collisition pairs
    robot._collision_model.addAllCollisionPairs()
    print("num collision pairs - initial:", len(robot._collision_model.collisionPairs))

    # Remove collision pairs listed in the SRDF file
    srdf_model_path = "/Users/markoguberina/lund/praxis/software/yumi/yumi_moveit_config/config/yumi.srdf"
    pin.removeCollisionPairs(robot.model, robot._collision_model, str(srdf_model_path))
    print(
        "num collision pairs - after removing useless collision pairs:",
        len(robot._collision_model.collisionPairs),
    )

    viz = MeshcatVisualizer(
        model=robot.model,
        collision_model=robot.collision_model,
        visual_model=robot.visual_model,
    )
    # viz.initViewer(open=True)
    viz.loadViewerModel(collision_color=[0.2, 0.2, 0.2, 0.6])
    viz.displayVisuals(False)
    viz.displayCollisions(True)
    # node = viz.viewer["pinocchio/visuals"]
    # node.set_property("modulated_opacity", 0.4)
    # node.set_property("opacity", 0.2)
    # node.set_property("color", [0.3, 0.0, 0.0, 0.6])

    rotation = pin.rpy.rpyToMatrix(np.array([np.pi, 0.0, 0.0]))
    if cfg.randomly_generate_goal:
        T_w_absgoal = getRandomlyGeneratedGoal(cfg, robot)
        if cfg.visualizer:
            robot.visualizer_manager.sendCommand({"Mgoal": T_w_absgoal})
    else:
        T_w_absgoal = defineGoalPointCLI(robot)
        T_w_absgoal.rotation = rotation
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15
    loop_manager = moveLDualArm(
        cfg, robot, T_w_absgoal, T_absgoal_l, T_absgoal_r, run=False
    )
    breakFlag = False
    # TODO: make the collisions light up when they happen
    robot.collision_data = pin.GeometryData(robot._collision_model)
    iter = 0
    color_collisions = False
    while not breakFlag:
        iter += 1
        breakFlag = loop_manager.run_one_iter(iter)
        # 1. Update geometry positions and compute collisions
        if color_collisions and iter % 20 == 0:
            colorCollisionsRed(robot, viz)

        viz.display(robot._q)
        time.sleep(1 / 500)
    colorCollisionsRed(robot, viz)
    print(
        "do we have a collision anywhere?",
        pin.computeCollisions(
            robot._model,
            robot._data,
            robot._collision_model,
            robot.collision_data,
            robot._q,
            False,
        ),
    )
    viz.display(robot._q)
    print(robot.model.upperPositionLimit)
    print(robot._q)
    print(robot.model.lowerPositionLimit)
    print("moveLDualArm done")
    #    from IPython import embed
    #
    #    embed()

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()
