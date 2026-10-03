from smc import load_config
import pinocchio as pin
import numpy as np


################################################################
#             STATE TRANSITION CONDITIONS
################################################################
def isGraspOK(cfg, robot, grasp_pose):
    isOK = False
    SEerror = robot.T_w_e().actInv(grasp_pose)
    err_vector = pin.log6(SEerror).vector
    # TODO: figure this out
    # it seems you have to use just the arm to get to finish with this precision
    # if np.linalg.norm(err_vector) < robot.cfg.goal_error:
    if np.linalg.norm(err_vector) < 2 * 1e-1:
        isOK = True
    return not isOK


def isGripperRelativeToBaseOK(cfg, robot):
    isOK = False
    # we want to be in the back of the base (x-axis) and on handlebar height
    T_w_base = robot.data.oMi[1]
    # rotation for the gripper is base with z flipped to point into the ground
    rotate = pin.SE3(pin.rpy.rpyToMatrix(np.pi, 0.0, 0.0), np.zeros(3))
    # translation is prefered distance from base
    translate = pin.SE3(
        np.eye(3),
        np.array(
            [cfg.base_to_handlebar_preferred_distance, 0.0, cfg.handlebar_height]
        ),
    )
    # grasp_pose = T_w_base.act(rotate.act(translate))
    grasp_pose = T_w_base.act(translate.act(rotate))
    SEerror = robot.T_w_e().actInv(grasp_pose)
    err_vector = pin.log6(SEerror).vector
    # TODO: figure this out
    # it seems you have to use just the arm to get to finish with this precision
    # if np.linalg.norm(err_vector) < robot.cfg.goal_error:
    if np.linalg.norm(err_vector) < 2 * 1e-1:
        isOK = True
    return isOK, grasp_pose


def areDualGrippersRelativeToBaseOK(cfg, goal_transform, robot):
    isOK = False
    # we want to be in the back of the base (x-axis) and on handlebar height
    T_w_base = robot.data.oMi[1]
    # rotation for the gripper is base with z flipped to point into the ground
    rotate = pin.SE3(pin.rpy.rpyToMatrix(np.pi, 0.0, 0.0), np.zeros(3))
    # translation is prefered distance from base
    translate = pin.SE3(
        np.eye(3),
        np.array(
            [cfg.base_to_handlebar_preferred_distance, 0.0, cfg.handlebar_height]
        ),
    )
    # grasp_pose = T_w_base.act(rotate.act(translate))
    grasp_pose = T_w_base.act(translate.act(rotate))

    grasp_pose_left = goal_transform.act(grasp_pose)
    grasp_pose_right = goal_transform.inverse().act(grasp_pose)

    T_w_e_left, T_w_e_right = robot.T_w_e()
    SEerror_left = T_w_e_left.actInv(grasp_pose_left)
    SEerror_right = T_w_e_right.actInv(grasp_pose_right)
    err_vector_left = pin.log6(SEerror_left).vector
    err_vector_right = pin.log6(SEerror_right).vector
    # TODO: figure this out
    # it seems you have to use just the arm to get to finish with this precision
    # if np.linalg.norm(err_vector) < robot.cfg.goal_error:
    if (np.linalg.norm(err_vector_left) < 2 * 1e-1) and (
        np.linalg.norm(err_vector_right) < 2 * 1e-1
    ):
        isOK = True
    return isOK, grasp_pose, grasp_pose_left, grasp_pose_right
