from smc import load_config
from behavior_tree import *
from smc.path_generation.maps.premade_maps import createSampleStaticMap
from smc.path_generation.planner import starPlanner, getPlanningArgs
from smc import load_config, getRobotFromConfig
from smc.multiprocessing import ProcessManager
from smc.control.optimal_control import *
from smc.control.cartesian_space import (
    getClikArgs,
    cartesianPathFollowingWithPlanner,
    controlLoopClik,
    QPquadprog,
    dampedPseudoinverse,
    controlLoopClik,
    controlLoopClikDualArm,
)
import pinocchio as pin
import numpy as np
import crocoddyl
from functools import partial
import yaml
import time

# TODO:
# - make reference step size in path_generator an argument here
#   because we use that for path interpolation later on as well


def get_cfg():
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getPlanningArgs(parser)
    parser.add_argument(
        "--handlebar-height",
        type=float,
        default=0.5,
        help="heigh of handlebar of the cart to be pulled",
    )
    parser.add_argument(
        "--base-to-handlebar-preferred-distance",
        type=float,
        default=0.7,
        help="prefered path arclength from mobile base position to handlebar",
    )
    cfg = parser.parse_cfg()
    # TODO TODO TODO: REMOVE PRESET HACKS
    robot_type = "Unicycle"
    with open(cfg.planning_robot_params_file) as f:
        params = yaml.safe_load(f)
    robot_params = params[robot_type]
    with open(cfg.tunnel_mpc_params_file) as f:
        params = yaml.safe_load(f)
    mpc_params = params["tunnel_mpc"]
    cfg.np = mpc_params["np"]
    cfg.n_pol = mpc_params["n_pol"]
    return cfg


def cartPullingControlLoop(
    cfg,
    robot,
    goal,
    goal_transform,
    ocp_grasp,
    ocp_pulling,
    path_planner,
    i: int,
    past_data,
):
    """
    cartPulling
    0) if obstacles present, don't move
    1) if no obstacles, but not grasped/grasp is off-target, grasp the handlebar with a p2p strategy.
    2) if no obstacles, and grasp ok, then pull toward goal with cartPulling mpc
    3) parking?
    4) ? always true (or do nothing is always true, whatever)
    """

    q = robot.q
    T_w_e_left, T_w_e_right = robot.T_w_e

    # we use binary as string representation (i don't want to deal with python's binary representation).
    # the reason for this is that then we don't have disgusting nested ifs
    # TODO: this has to have named entries for code to be readable
    priority_register = ["0", "1", "1"]
    # TODO implement this based on laser scanning or whatever
    # priority_register[0] = str(int(areObstaclesTooClose()))
    graspOK, grasp_pose, grasp_pose_left, grasp_pose_right = (
        areDualGrippersRelativeToBaseOK(cfg, goal_transform, robot)
    )
    # NOTE: this keeps getting reset after initial grasp has been completed.
    # and we want to let mpc cook
    priority_register[1] = str(int(not graspOK))  # set if not ok
    # TODO: get grasp pose from vision, this is initial hack to see movement
    #    if i > 1000:
    #        priority_register[1] = '0'

    # interpret string as base 2 number, return int in base 10
    priority_int = ""
    for prio in priority_register:
        priority_int += prio
    priority_int = int(priority_int, 2)
    breakFlag = False
    save_past_item = {}
    log_item = {}

    # case 0)
    if priority_int >= 4:
        robot.sendVelocityCommand(np.zeros(robot.model.nv))

    # case 1)
    # TODO: make it an argument obviously
    usempc = False
    if (priority_int < 4) and (priority_int >= 2):
        if usempc:
            ocp_grasp.updateGoalInModels(grasp_pose_left, grasp_pose_right)
            CrocoEEP2PMPCControlLoop(cfg, robot, ocp_grasp, i, past_data)
        else:
            # controlLoopClik(robot, invKinmQP, i, past_data)
            clikController = partial(dampedPseudoinverse, 1e-3)
            # controlLoopClik(robot, clikController, i, past_data)
            if robot.robot_name != "yumi":
                controlLoopClik(robot, clikController, i, past_data)
            else:
                # TODO: DEFINE SENSIBLE TRANSFOR
                controlLoopClikDualArm(
                    robot, clikController, goal_transform, i, past_data
                )

    # case 2)
    # MASSIVE TODO:
    # WHEN STARTING, TO INITIALIZE PREPOPULATE THE PATH WITH AN INTERPOLATION OF
    # A LINE FROM WHERE THE GRIPPER IS NOW TO THE BASE

    # whether we're transitioning from cliking to pulling
    # this is the time and place to populate the past path from the pulling to make sense
    if past_data["priority_register"][-1][1] == "1" and priority_register[1] == "0":
        # create straight line path from ee to base
        p_cart = q[:2]
        if robot.robot_name != "yumi":
            p_ee = T_w_e.translation[:2]
            straigh_line_path = np.linspace(p_ee, p_cart, cfg.past_window_size)
        else:
            p_ee_l = T_w_e_left.translation[:2]
            p_ee_r = T_w_e_right.translation[:2]
            # MASSIVE TODO: have two different REFERENCES FOR TWO ARMS
            # EVILLLLLLLLLLLLLLLLLLLLLLLLLLll
            # MASSIVE TODO: have two different REFERENCES FOR TWO ARMS
            # MASSIVE TODO: have two different REFERENCES FOR TWO ARMS
            # MASSIVE TODO: have two different REFERENCES FOR TWO ARMS
            # ----------------> should be doable by just hitting the path with the goal_transform for dual.
            #                   in the whole task there are no differences in left and right arm
            straigh_line_path = np.linspace(p_ee_l, p_cart, cfg.past_window_size)
        # time it the same way the base path is timed
        time_past = np.linspace(
            0.0, cfg.past_window_size * robot.dt, cfg.past_window_size
        )
        s = np.linspace(0.0, cfg.n_knots * cfg.ocp_dt, cfg.past_window_size)
        path2D_handlebar = np.hstack(
            (
                np.interp(s, time_past, straigh_line_path[:, 0]).reshape((-1, 1)),
                np.interp(s, time_past, straigh_line_path[:, 1]).reshape((-1, 1)),
            )
        )

        past_data["path2D_untimed"].clear()
        past_data["path2D_untimed"].extend(
            path2D_handlebar[i] for i in range(cfg.past_window_size)
        )

    if priority_int < 2:
        # TODO make this one work
        breakFlag, save_past_item, log_item = BaseAndEEPathFollowingMPCControlLoop(
            cfg,
            robot,
            ocp_pulling,
            path_planner,
            grasp_pose,
            goal_transform,
            i,
            past_data,
        )
        # BaseAndEEPathFollowingMPCControlLoop(cfg, robot, ocp_pulling, path_planner, i, past_data)

    p = q[:2]
    # needed for cart pulling
    save_past_item["path2D_untimed"] = p
    save_past_item["priority_register"] = priority_register.copy()
    # TODO plot priority register out
    # log_item['prio_reg'] = ...
    log_item["qs"] = q.reshape((robot.model.nq,))
    log_item["vs"] = robot.v.reshape((robot.model.nv,))
    print(priority_register)
    return breakFlag, save_past_item, log_item


def cartPulling(cfg, robot, goal, path_planner):
    # transfer single arm to dual arm reference
    # (instead of gripping the middle grip slightly away from middle)
    goal_transform = pin.SE3.Identity()
    # TODO: it's cursed and it shouldn't be
    #    goal_transform.rotation = pin.rpy.rpyToMatrix(0.0, np.pi/2, 0.0)
    # TODO: it's cursed and it shouldn't be
    #    goal_transform.translation[1] = -0.1
    ############################
    #  setup cart-pulling mpc  #
    ############################
    x0 = np.concatenate([robot.q, robot.v])
    ocp_pulling = BaseAndDualArmEEPathFollowingOCP(cfg, robot, x0)
    ocp_pulling.solveInitialOCP(x0)

    #############################################
    #  setup point-to-point handlebar grasping  #
    # TODO: have option to swith this for clik  #
    #############################################
    grasp_pose = robot.T_w_e
    ocp_grasp = CrocoIKOCP(cfg, robot, x0, grasp_pose)
    ocp_grasp.solveInitialOCP(x0)

    controlLoop = partial(
        cartPullingControlLoop,
        cfg,
        robot,
        goal,
        goal_transform,
        ocp_grasp,
        ocp_pulling,
        path_planner,
    )

    log_item = {}
    q = robot.q
    if robot.robot_name != "yumi":
        T_w_e = robot.T_w_e()
    else:
        T_w_e_l, T_w_e_right = robot.T_w_e()
    log_item["qs"] = q.reshape((robot.model.nq,))
    log_item["vs"] = robot.v.reshape((robot.model.nv,))
    # T_base = self.robot_manager.data.oMi[1]
    # NOTE: why the fuck was the past path defined from the end-effector?????
    # save_past_item = {'path2D_untimed' : T_w_e.translation[:2],
    save_past_item = {"path2D_untimed": q[:2], "priority_register": ["0", "1", "1"]}
    loop_manager = ControlLoopManager(
        robot, controlLoop, cfg, save_past_item, log_item
    )
    loop_manager.run()


if __name__ == "__main__":
    cfg = get_cfg()
    robot = getRobotFromConfig(cfg)
    # TODO: here you have to exit unless the robot has a mobile base interface
    robot._q[0] = 9.0
    robot._q[1] = 4.0

    x0 = np.concatenate([robot.q, robot.v])
    robot._step()
    goal = np.array([0.5, 5.5])

    ###########################
    #  visualizing obstacles  #
    ###########################
    _, map_as_list = createSampleStaticMap()
    # we're assuming rectangles here
    # be my guest and implement other options
    if cfg.visualizer:
        robot.sendRectangular2DMapToVisualizer(map_as_list)

    planning_function = partial(starPlanner, goal)
    # TODO: ensure alignment in orientation between planner and actual robot
    path_planner = ProcessManager(cfg, planning_function, robot.q[:2], 3, None)
    # wait for meshcat to initialize
    if cfg.visualizer:
        time.sleep(5)
    # clik version
    cartesianPathFollowingWithPlanner(cfg, robot, path_planner, 0.0)
    # end-effector tracking version
    # CrocoEndEffectorPathFollowingMPC(cfg, robot, x0, path_planner)
    # base tracking version (TODO: implement a reference for ee too)
    # and also make the actual path for the cart and then construct the reference
    # for the mobile base out of a later part of the path)
    # atm this is just mobile base tracking
    cartPulling(cfg, robot, goal, path_planner)
    print("final position:")
    print(robot.T_w_e)
    path_planner.terminateProcess()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if not cfg.pinocchio_only:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
