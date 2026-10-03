from utils import (
    computeInitialBasePoseForCtrlWithBaseReference,
    constructedTradedOffEEReference,
)

from cart_pulling_disjoint_ee_and_base_cartesian_ctrl import (
    CartPullingDisjointEEAndBaseCartesianCtrl,
    transfEERefToBRef,
)
from smc import getRobotFromConfig, load_config
from smc.control.cartesian_space import (
    moveLDualArm,
    moveL,
)
from smc.control.cartesian_space.cartesian_space_trajectory_tracking import (
    cartesianDualArmTrajectoryTracking,
)
from smc.motion_planning.motion_planner import MotionPlannerFixedPath
from smc.control.joint_space.joint_space_point_to_point import moveJPWTraj
from smc.motion_planning.interpolation.bsplinesSE3 import CubicBezierSplineSE3

# from smc.motion_planning.interpolation.preprocessing import reduceSE3PathPoints
# from smc.motion_planning.utilities.path2d_to_6d import path2D_to_SE3

# from smc.path_generation.fixed_path_planner import (
#    contructPath,
# )
from smc.motion_planning.trajectory_generation.end_effector_velocity_limit_scaling import (
    UniformTimeLawWithEEContraints,
)

import numpy as np
import pinocchio as pin

if __name__ == "__main__":

    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    assert cfg.robot == "slumi"
    assert (cfg.cart_pulling.ebd_vs_ee_path_tradeoff > 0.0) and (
        cfg.cart_pulling.ebd_vs_ee_path_tradeoff < 1.0
    )
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15
    selected_solver = cfg.cs.ik_solver

    # used for testing -> only worthwhile to show how shit the fit is
    # path3D = contructPath(cfg, robot)
    # path3D[:, 1] -= cfg.map_height / 2

    # path2D_to_SE3 is the function that fucked us by far the most
    # T_list: list[pin.SE3] = path2D_to_SE3(path3D, cfg.path_height, cfg.path_rotx)
    # T_list = reduceSE3PathPoints(0.5, T_list)
    # print("reduced path to", len(T_list), "total points")
    T_list = []
    for i in range(10):
        # T = pin.SE3.Random()
        rpy = np.random.random(3) * 1.5
        rpy[0] += np.pi
        R = pin.rpy.rpyToMatrix(rpy)
        translation = np.zeros(3)
        translation[0] = (i + 1) / 5 + np.random.random() * 0.1
        translation[1] = np.random.random() * ((i + 1) / 5)
        translation[2] = 1.1 + np.random.random() * 0.1
        T_list.append(pin.SE3(R, translation))

    smooth_path = CubicBezierSplineSE3(T_list, np.zeros(6), np.zeros(6))
    # we need a function that spits out T_s, V_s,
    # the controller does not care for its internal variables (as it shouldn't)

    if cfg.visualizer:
        robot.visualizer_manager.sendCommand({"Mgoal": T_list[0]})
        robot.visualizer_manager.sendCommand({"SE3waypoints": T_list})
        import time

        time.sleep(5)
        viz_Ts = []
        ss = np.linspace(0, 1, 500)
        for s in ss:
            viz_Ts.append(smooth_path.getPoint(s))
        robot.visualizer_manager.sendCommand({"framepath": viz_Ts})

    print("putting arms to a comfy configuration")
    robot.mode = robot.control_mode.upper_body
    moveJPWTraj(robot.comfy_configuration, cfg, robot)

    # construct timing law
    time_law = UniformTimeLawWithEEContraints(
        smooth_path,
        v_max=np.ones(6) * np.min(robot.max_v[:3]),
        a_min=np.ones(6) * -0.1,
        a_max=np.ones(6) * 0.1,
    )
    time_law.T_total = time_law.T_total * 4

    motion_planner = MotionPlannerFixedPath(smooth_path, time_law)

    # move base to appropriate start location
    T_start = smooth_path.getPoint(0.0)
    if cfg.cart_pulling.base_reference:
        s_base_offset, T_s = computeInitialBasePoseForCtrlWithBaseReference(
            cfg, smooth_path
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
        T_start, _ = constructedTradedOffEEReference(cfg, robot, T_start, np.zeros(6))
        moveLDualArm(cfg, robot, T_start, T_absgoal_l, T_absgoal_r)
    else:
        robot.mode = robot.control_mode.whole_body
        cfg.cs.ik_solver = selected_solver
        moveLDualArm(cfg, robot, T_start, T_absgoal_l, T_absgoal_r)

    for i in range(cfg.ctrl_freq):
        robot.sendVelocityCommand(np.zeros(robot.nv))
    if cfg.real:
        input(
            "SET THE CART INTO THE END-EFFECTORS MANUALLY. hit Enter and will start traj tracking :) "
        )
    print("starting path tracking")
    robot.mode = robot.control_mode.whole_body
    cfg.cs.ik_solver = selected_solver
    if cfg.cart_pulling.base_reference:
        raise NotImplementedError(
            "the heading does not make sense now. need to write a function to map the arbitrary SE3 to something sensible for the base to use this. but then again, for this you want only EE ref with IKQP with EBDistance and Posture costs anyway"
        )
        CartPullingDisjointEEAndBaseCartesianCtrl(
            T_absgoal_l,
            T_absgoal_r,
            smooth_path,
            time_law,
            s_base_offset,
            cfg,
            robot,
        )
    else:
        cartesianDualArmTrajectoryTracking(
            T_absgoal_l,
            T_absgoal_r,
            motion_planner,
            cfg,
            robot,
        )

    robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
