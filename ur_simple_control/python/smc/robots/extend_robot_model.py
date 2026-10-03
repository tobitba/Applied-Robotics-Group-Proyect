import coal
import pinocchio as pin
import numpy as np

# TODO: this is hardcoded for yumi and cart.
# but it does not have to be.
# added held objects to visualization is not a bad idea in general.
# this function can not be made that general.
# but an example should be somewhere (not here but hey, i need it at the moment
# of writing haha xd)


def addCartIntoVisualizationAndCollision(robot):
    # 1. Define the box shape and its relative placement to the EE frame
    box_shape = coal.Box(0.7, 0.35, 0.96)
    ee_frame_id = robot._model.getFrameId("gripper_l_joint")
    ee_frame = robot._model.frames[ee_frame_id]
    # NOTE: this one is perfect for where the cart is.
    # but it catches the collision with the gripper.
    # i don't want to fix this now,
    # so i'll just lower it
    # TODO: remove that collision pair
    # placement = pin.SE3(np.eye(3), np.array([0.7 / 2, 0.15, 0.96 / 2 + 0.15]))
    placement = pin.SE3(np.eye(3), np.array([0.7 / 2, 0.15, 0.96 / 2 + 0.15 + 0.05]))

    # 2. Create the Collision Geometry Object
    box_collision = pin.GeometryObject(
        "cart", ee_frame.parentJoint, ee_frame_id, placement, box_shape
    )
    box_collision.meshColor = np.array([0.0, 0.5, 0.5, 0.3])  # some teal

    # 3. Add to the collision model
    box_id = robot._collision_model.addGeometryObject(box_collision)
    box_id = robot._visual_model.addGeometryObject(box_collision)
