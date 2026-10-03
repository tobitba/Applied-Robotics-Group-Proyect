from smc import load_config, getRobotFromConfig
from smc.control.cartesian_space.cartesian_space_point_to_point import (
    moveLDualArm,
)
from smc.robots.interfaces.whole_body_dual_arm_interface import (
    DualArmWholeBodyInterface,
)
from smc.robots.interfaces.dual_arm_interface import (
    DualArmInterface,
)
from smc.robots.extend_robot_model import addCartIntoVisualizationAndCollision

import argparse
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
import multiprocessing as mp
import time
import pickle


# NOTE: magic numbers for sleipner base,
# i can't be bothered rn
def inBase(translation):
    x_check = translation[0] < 0.0
    y_check = (translation[1] < 0.29) and (translation[1] > -0.29)
    return x_check and y_check


# TODO: check collisions
# if you manage to create a cart box in coal that you be lit.
# then you can also create a box for the base.
# and check collisions that way.
# furthermore, we can add arm self-collisions as well.
def cartPoints(robot, T_w_handlebar) -> tuple[pin.SE3, pin.SE3, pin.SE3, pin.SE3]:
    # NOTE: hardcoded, i can't be bothered
    cart_shape = (0.7, 0.35, 0.96)
    # T_w_handlebar = T_w_abs
    # which is at the middle and top of cart.
    # height does not matter here
    # yes just checking points misses some cases.
    # but who cares, this is just optimization.
    # TODO: check that it's -0.7 in x, not +0.7
    T_handlebar_c1 = pin.SE3(np.eye(3), np.array([0.0, 0.35 / 2, 0.0]))
    T_handlebar_c2 = pin.SE3(np.eye(3), np.array([0.0, -0.35 / 2, 0.0]))
    T_handlebar_c3 = pin.SE3(np.eye(3), np.array([-0.86, -0.35 / 2, 0.0]))
    T_handlebar_c4 = pin.SE3(np.eye(3), np.array([-0.86, 0.35 / 2, 0.0]))
    c1 = T_w_handlebar.act(T_handlebar_c1)
    c2 = T_w_handlebar.act(T_handlebar_c2)
    c3 = T_w_handlebar.act(T_handlebar_c3)
    c4 = T_w_handlebar.act(T_handlebar_c4)
    if robot.cfg.visualizer:
        robot.visualizer_manager.sendCommand({"frame_path": [c1, c2, c3, c4]})

    c1 = c1.translation
    c2 = c2.translation
    c3 = c3.translation
    c4 = c4.translation
    return c1, c2, c3, c4


# NOTE:
# there are ways to make this more efficient.
# but the question is - do we actually need to make it more efficient?
# who cares? let it run for 2 hours. yes, it could have been 10 minutes
# or whatever if done more efficiently. but you would spend
# those 2 hours making it more efficient, man.
# this only needs to run once.
# TODO:
# check for self collision also.
def checkHeadings(
    robot: DualArmInterface,
    T_absgoal_l: pin.SE3,
    T_absgoal_r: pin.SE3,
    default_rotation: np.ndarray,
    translation: np.ndarray,
    headings: np.ndarray,
) -> np.ndarray:
    workspace_mask_ij = np.zeros(len(headings), dtype=np.bool)
    for k in range(len(headings)):
        if robot.cfg.visualizer:
            print("=" * 40)
        feasible = False
        breakFlag = False
        robot._q = robot._comfy_configuration
        robot._q[0] = -0.52
        robot._q[2] = -1.0
        robot._q[3] = 0.0
        robot._step()
        if robot.cfg.visualizer:
            headings[k] = headings[np.random.randint(0, len(headings))]

        T_w_goal = pin.SE3(
            default_rotation @ pin.rpy.rpyToMatrix(np.array([0.0, 0.0, headings[k]])),
            translation,
        )
        # if the point puts the cart in the base
        # we already know it's infeasible on that front
        point_invalid = False
        # TODO: if sqrt(x_i**2 + y_j**2) also just skip
        if (headings[k] < 0.5) and (headings[k] > -0.5):
            if robot.cfg.visualizer:
                print("heading to close to 0!")
            point_invalid = True
        for c_i in cartPoints(robot, T_w_goal):
            if inBase(c_i):
                point_invalid = True
                if robot.cfg.visualizer:
                    print("point", c_i, "is in base!")
        if np.sqrt(translation[0] ** 2 + translation[1] ** 2) > 0.7:
            point_invalid = True
            if robot.cfg.visualizer:
                print("point", T_w_goal.translation, "is invalid - it's too far")
        if point_invalid and not robot.cfg.visualizer:
            continue

        loop_manager = moveLDualArm(
            robot.cfg, robot, T_w_goal, T_absgoal_l, T_absgoal_r, run=False
        )
        iter = 0
        prev_q = robot.q
        for iter in range(robot.cfg.max_iterations):
            prev_q = robot.q
            breakFlag = loop_manager.run_one_iter(iter)
            if breakFlag or (np.linalg.norm(prev_q - robot.q) < 1e-8):
                break
        # NOTE: check collisions only if ik converged
        # che
        # collisionsFlag = False
        # if breakFlag:
        collisionsFlag = pin.computeCollisions(
            robot._model,
            robot._data,
            robot._collision_model,
            robot.collision_data,
            robot._q,
            False,
        )
        feasible = breakFlag and (not collisionsFlag)
        if robot.cfg.visualizer:
            robot.visualizer_manager.sendCommand({"Mgoal": T_w_goal})
            robot.visualizer_manager.sendCommand({"q": robot._q})
            print("point invalid?", point_invalid)
            print("converged?", breakFlag, ", in", iter, "iterations")
            print("self-collision?", collisionsFlag)
            print("is the point feasible?", feasible)
            time.sleep(10)
            break
        # print("result", breakFlag)
        workspace_mask_ij[k] = feasible
    return workspace_mask_ij


def inverseKinematicsWorker(
    cfg: argparse.Namespace,
    default_rotation: np.ndarray,
    headings: np.ndarray,
    input_queue: mp.Queue,
    output_queue: mp.Queue,
):
    robot = getRobotFromConfig(cfg)
    # addCartIntoVisualizationAndCollision(robot)
    robot._collision_model.addAllCollisionPairs()
    srdf_model_path = "/Users/markoguberina/lund/praxis/software/yumi/yumi_moveit_config/config/yumi.srdf"
    pin.removeCollisionPairs(robot.model, robot._collision_model, str(srdf_model_path))
    robot.collision_data = pin.GeometryData(robot._collision_model)
    if issubclass(robot.__class__, DualArmWholeBodyInterface):
        robot._mode = robot.control_mode.upper_body
    # NOTE: we're considering only fixed relative frames
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15

    # NOTE: default setting when simulating is dt = 1e-3
    # this is too high if we don't want this to last forever
    # in this case thankfully, we can override whatever we want in python -
    # private variables aren't real :)
    robot._dt = 1e-2
    while input_queue.empty():
        time.sleep(1)

    while not input_queue.empty():
        robot._q = robot._comfy_configuration
        robot._q[0] = -0.52
        robot._q[2] = -1.0
        robot._q[3] = 0.0
        robot._step()
        index, translation = input_queue.get_nowait()
        workspace_mask_ij = checkHeadings(
            robot, T_absgoal_l, T_absgoal_r, default_rotation, translation, headings
        )
        output_queue.put((index, workspace_mask_ij))


if __name__ == "__main__":
    cfg = load_config()
    # TODO: select appropriate IK solver.
    # matter of fact, it's a sensible experiment to try out different ones.
    # but CRITICALLY, THEY ALL must respect joint limits.
    # so TODO: put that in the QP
    cfg.plotter = False
    cfg.cs.respect_joint_limits = True
    cfg.cs.ik_solver = "QPWithPosture"
    cfg.mode = "upper_body"
    if not cfg.visualizer:
        cfg.ctrl_freq = -1.0

    # NOTE: max number of iterations should be set to something low.
    # alternatively, since it's just moveLs, you can kill the function
    # as soon as the commanded velocity and/or change in joint angles
    # is too low.
    # doing this is almost certainly necessary as we want some solid
    # 100 x 100 grid, and thats 1e4 movels! we can't have 0.5 seconds for
    # empty iterations in place...
    cfg.max_iterations = 5000

    # NOTE:
    # for the cart-pulling problem, we don't actually need to fit the full square.
    # we can just do the relevant semicircles.
    # this is more for a nice plot.
    # also, i don't think there's a point in starting
    # from scratch for every possible heading.
    # just go along the headings.
    # i mean you could say the same for positions, but i'm less sure of those

    # point cloud definition
    height = 0.96  # - 0.85
    default_rotation = pin.rpy.rpyToMatrix(np.array([np.pi, 0.0, 0.0]))
    N = 80
    xs = np.linspace(-0.2, 0.7, N)
    ys = np.linspace(-0.7, 0.7, N)
    # now we also have to test heading
    # TODO: select something else for headings
    N_headings = 50
    headings = np.linspace(-np.pi, np.pi, N_headings)
    workspace_mask = np.zeros((N, N, N_headings), dtype=np.bool)

    # other variables instantiated to make code checker happy
    robot: DualArmInterface
    n_workers = 0
    workers = []
    T_absgoal_l = None
    T_absgoal_r = None

    if cfg.visualizer:
        robot = getRobotFromConfig(cfg)
        # addCartIntoVisualizationAndCollision(robot)
        robot._collision_model.addAllCollisionPairs()
        srdf_model_path = "/Users/markoguberina/lund/praxis/software/yumi/yumi_moveit_config/config/yumi.srdf"
        pin.removeCollisionPairs(
            robot.model, robot._collision_model, str(srdf_model_path)
        )
        robot.collision_data = pin.GeometryData(robot._collision_model)
        if issubclass(robot.__class__, DualArmWholeBodyInterface):
            robot._mode = robot.control_mode.upper_body
        # NOTE: we're considering only fixed relative frames
        T_absgoal_l = pin.SE3.Identity()
        T_absgoal_l.translation[1] = 0.15
        T_absgoal_r = pin.SE3.Identity()
        T_absgoal_r.translation[1] = -0.15

        # NOTE: default setting when simulating is dt = 1e-3
        # this is too high if we don't want this to last forever
        # in this case thankfully, we can override whatever we want in python -
        # private variables aren't real :)
        robot._dt = 1e-2

        all_points = np.zeros((N**2, 3))
        for i in range(N):
            for j in range(N):
                all_points[i * N + j] = np.array([xs[i], ys[j], height])

        robot.visualizer_manager.sendCommand({"fixed_path": all_points})

    else:
        n_workers = 8
        workers = []
        # input_queue = mp.Queue(maxsize=50 * n_workers)
        # output_queue = mp.Queue(maxsize=50 * n_workers)
        input_queue = mp.Queue()
        output_queue = mp.Queue()
        for i in range(n_workers):
            p = mp.Process(
                target=inverseKinematicsWorker,
                args=(cfg, default_rotation, headings, input_queue, output_queue),
            )
            p.start()
            workers.append(p)

    # TODO: you can immediatelly discard everything that's a self-collision.
    # namely the
    for i in range(len(xs)):
        for j in range(len(ys)):
            translation = np.array([xs[i], ys[j], height])
            if cfg.visualizer:
                # robot._q = robot.comfy_configuration
                # robot._step()
                # print("checking", i * N + j, "out of", N**2)
                i = np.random.randint(0, len(xs))
                j = np.random.randint(0, len(ys))
                workspace_mask[i][j] = checkHeadings(
                    robot,
                    T_absgoal_l,
                    T_absgoal_r,
                    default_rotation,
                    translation,
                    headings,
                )
            else:
                input_queue.put(((i, j), translation))
    time.sleep(10)
    N_points = len(xs) * len(ys)
    n_checked_points = 0
    while n_checked_points < N_points:
        time.sleep(0.01)
        index, workspace_mask_ij = output_queue.get()
        n_checked_points += 1
        print("checked point:", n_checked_points, "/", len(xs) * len(ys))
        workspace_mask[index[0]][index[1]] = workspace_mask_ij
        if index[1] == len(ys) - 1:
            np.savetxt(
                "./data/computed_points_row_" + str(index[0]) + ".csv",
                workspace_mask[index[0], :, :],
                delimiter=",",
            )

    if cfg.visualizer:
        for i in range(n_workers):
            workers[i].join()

    # TODO: save results

    np.savetxt("./data/xs" + ".csv", xs, delimiter=",")
    np.savetxt("./data/ys" + ".csv", ys, delimiter=",")
    np.savetxt("./data/headings" + ".csv", headings, delimiter=",")
    file = open("./data/workspace_mask_ALL.pickle", "wb")
    pickle.dump(workspace_mask, file)
    file.close()
    XS, YS, ZS = np.meshgrid(xs, ys, headings)
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(XS[workspace_mask], YS[workspace_mask], ZS[workspace_mask], color="b")
    ax.scatter(
        XS[np.bitwise_not(workspace_mask)],
        YS[np.bitwise_not(workspace_mask)],
        ZS[np.bitwise_not(workspace_mask)],
        color="r",
    )
    ax.set_xlabel("x/m")
    ax.set_ylabel("y/m")
    ax.set_zlabel("heading/rad")
    plt.show()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
