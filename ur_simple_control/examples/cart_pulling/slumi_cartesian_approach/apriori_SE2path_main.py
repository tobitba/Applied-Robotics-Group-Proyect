from smc import load_config
from smc.robots.utils import getRobotFromConfig
from utils import (
    computeInitialBasePoseForCtrlWithBaseReference,
    getPathPoints,
    bringPathToRobot,
)

from cart_pulling_disjoint_ee_and_base_cartesian_ctrl import (
    CartPullingDisjointEEAndBaseCartesianCtrl,
    transfEERefToBRef,
)
from workspace2d import WorkspaceSE2
from smc.control.cartesian_space import (
    moveLDualArm,
    moveL,
)
from smc.control.cartesian_space.cartesian_space_trajectory_tracking import (
    cartesianDualArmTrajectoryTracking,
)
from smc.control.joint_space.joint_space_point_to_point import moveJPWTraj
from smc.motion_planning.interpolation.bsplines2D_unedited_vibecode import CubicSpline2D

from smc.motion_planning.trajectory_generation.end_effector_velocity_limit_scaling import (
    #    computeTimeScalingEEVelConstraint,
    UniformTimeLawWithEEContraints,
)

import numpy as np
import pinocchio as pin
from pinocchio import SE3
import pickle
import time

if __name__ == "__main__":

    cfg = load_config()
    cfg.cs.respect_joint_limit = True
    robot = getRobotFromConfig(cfg)
    robot._model.upperPositionLimit = robot._model.upperPositionLimit * 0.9
    robot._model.lowerPositionLimit = robot._model.lowerPositionLimit * 0.9
    assert cfg.robot == "slumi"
    # TODO: move this to config parsing - add limits to metadata
    assert (cfg.cart_pulling.ebd_vs_ee_path_tradeoff > 0.0) and (
        cfg.cart_pulling.ebd_vs_ee_path_tradeoff < 1.0
    )
    robot._model.velocityLimit[:3] = 0.3
    T_absgoal_l = SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = SE3.Identity()
    T_absgoal_r.translation[1] = -0.15
    selected_solver = cfg.cs.ik_solver
    # T_abs_c = SE3(np.eye(3), np.array([cart_control_point_x_offset, 0.0, 0.0]))
    # NOTE: but T_abs_c is completely pointless man.
    # this changes Psi_c to be something else.
    # and why? isn't it better to use different arclengths then?
    # then the point is still related to the path and
    # thus makes sense

    # TODO: just pass the file, open pickle in the WorkspaceSE2 class
    # file = open("./parameters/workspace_center_base.pickle", "rb")
    file = open("./parameters/workspace_best.pickle", "rb")
    # file = open("./parameters/workspace_physical_robot_best.pickle", "rb")
    # NOTE: this one is definitely too close
    # file = open("./parameters/workspace_aggressive.pickle", "rb")
    # TODO: make the workspace path a config entry
    workspace_raw = pickle.load(file)
    file.close()
    workspace = WorkspaceSE2(cfg, workspace_raw)
    preferred_base_to_handlebar_distance = (
        0.52 - abs(workspace.center[0]) + workspace.radius
    )
    # i hope this is correct le mao
    # this is ok, but i can also just make it a parameter...
    # like it was.
    # we're going full circle on this
    print(
        "the arclength to the workspace is:",
        0.52 - abs(workspace.center[0]) + workspace.radius,
    )

    # NOTE: but we won't do 180s in place.
    # the minimum can be computed from the oscullating circle + dimensions.
    # the half base diagonal only makes sense if we expect to
    # rotate in place, which we won't.
    # base_half_diagonal = np.sqrt((1.04 / 2) ** 2 + (0.58 / 2) ** 2)
    # assert cfg.safety_radius >= base_half_diagonal
    # print(
    #    "safety radius must be at least half base diagonal, which is",
    #    base_half_diagonal,
    # )
    base_width = 0.58 / 2
    assert cfg.cart_pulling.safety_radius > base_width
    path2D, radii = getPathPoints()
    # N_pts = 50
    # linear = np.linspace(0.0, 5.5, N_pts).reshape((N_pts, 1))
    # path_ee_2D = np.hstack((linear, 0.0 * np.sin(linear * 2)))
    # path3D = np.hstack((path_ee_2D, np.zeros((len(path_ee_2D), 1))))

    print(path2D)
    init_vel = np.zeros(2)
    final_vel = np.zeros(2)
    mu = 0.9999
    # NOTE: the spline will fit to the radius.
    # but we actually want it to stop at least base width away
    # NOTE: do the fit to get start path orientation as well
    smooth_2Dpath = CubicSpline2D(
        path2D, mu, init_vel, final_vel, radii=radii - base_width
    )
    smooth_2Dpath.setSE3LiftingParams(
        cfg.test_path.path_rotx, cfg.test_path.path_height
    )
    T_1 = smooth_2Dpath.getPathPoint(0.001)[0]
    # NOTE: so that we can move the path to the robot,
    # which is convenient for hardware experiments in limited space.
    # but we do need to recompute the path then.
    # i feel like this is better than adding a transformation to the smooth
    # curve class
    path2D = bringPathToRobot(path2D, T_1)
    smooth_2Dpath = CubicSpline2D(
        path2D, mu, init_vel, final_vel, radii=radii - base_width
    )
    smooth_2Dpath.setSE3LiftingParams(
        cfg.test_path.path_rotx, cfg.test_path.path_height
    )

    if cfg.visualizer:
        # robot.visualizer_manager.sendCommand({"Mgoal": T_list[0]})
        viz_path = np.hstack((path2D, np.zeros((len(path2D), 1))))
        print(viz_path)
        viz_Ts = []
        ss = np.linspace(0, 1, 500)
        for s in ss:
            viz_Ts.append(smooth_2Dpath.getCurrentSE3Ref(s)[0])
        robot.visualizer_manager.sendCommand({"framepath": viz_Ts})
        # NOTE: reintroduce
        # robot.updateViz({"fixed_path": path2D})

    print("putting arms to a comfy configuration")
    robot.mode = robot.control_mode.upper_body
    moveJPWTraj(robot.comfy_configuration, cfg, robot)

    # construct timing law
    #    T_total = computeTimeScalingEEVelConstraint(
    #        C_segments,
    #        s_list,
    #        v_max=np.ones(6) * np.min(robot.max_v[:3]),
    #        a_min=np.ones(6) * -0.5,
    #        a_max=np.ones(6) * 0.5,
    #    )
    # T_total = 100
    time_law = UniformTimeLawWithEEContraints(smooth_2Dpath)
    print("alloted time", time_law.T_total)
    time_law.T_total = 23

    # move base to appropriate start location
    T_start, _ = smooth_2Dpath.getCurrentSE3Ref(0.001)
    if cfg.cart_pulling.base_reference:
        s_base_offset, T_s = computeInitialBasePoseForCtrlWithBaseReference(
            cfg, preferred_base_to_handlebar_distance, smooth_2Dpath.getCurrentSE3Ref
        )

        T_s_b = transfEERefToBRef(cfg.test_path, T_s)
        cfg.cs.ik_solver = "QPVanilla"
        print(
            "moving the base to its start position:",
            T_s_b.translation[:2],
        )
        robot.mode = robot.control_mode.base_only
        moveL(cfg, robot, T_s_b)
        print("moveL just arms to start of arm path")
        robot.mode = robot.control_mode.upper_body
        cfg.cs.ik_solver = "QPWithPosture"
        # TODO: this breaks the current thing, oops
        # cfg.respect_joint_limits = True
        # T_start, _ = constructedTradedOffEEReference(cfg, robot, T_start, np.zeros(6))
        T_start = workspace.projectIntoWorkspaceEuclidean(T_start, T_s)[0]
        T_start.rotation = T_start.rotation @ pin.rpy.rpyToMatrix(0.0, 0.0, np.pi)
        moveLDualArm(cfg, robot, T_start, T_absgoal_l, T_absgoal_r)
        # it continued to move for some reason
    else:
        robot.mode = robot.control_mode.whole_body
        cfg.cs.ik_solver = selected_solver
        moveLDualArm(cfg, robot, T_start, T_absgoal_l, T_absgoal_r)

    for i in range(cfg.ctrl_freq):
        robot.sendVelocityCommand(np.zeros(robot.nv))

    for i in range(10):
        robot.sendVelocityCommand(np.zeros(robot.nv))
        time.sleep(0.1)
    if cfg.real:
        input(
            "SET THE CART INTO THE END-EFFECTORS MANUALLY. hit Enter and will start traj tracking :) "
        )
    print("starting path tracking")
    # NOTE: this might be very stupid
    # cfg.is_task_2D = True
    robot.mode = robot.control_mode.whole_body
    cfg.cs.ik_solver = selected_solver
    if cfg.cart_pulling.base_reference:
        CartPullingDisjointEEAndBaseCartesianCtrl(
            s_base_offset,
            workspace,
            T_absgoal_l,
            T_absgoal_r,
            smooth_2Dpath,
            time_law,
            cfg,
            robot,
        )
    else:
        cartesianDualArmTrajectoryTracking(
            T_absgoal_l,
            T_absgoal_r,
            smooth_2Dpath.getCurrentSE3Ref,
            time_law,
            cfg,
            robot,
        )

    robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
