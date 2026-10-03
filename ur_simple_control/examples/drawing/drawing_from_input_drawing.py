from smc import load_config
# TODO: write error handlers around file loading
from utils import getArgsForDrawing
from get_marker import getMarker
from find_marker_offset import findMarkerOffset
from drawing_ctrl_loop import write

from smc.robots.interfaces.force_torque_sensor_interface import (
    ForceTorqueOnSingleArmWrist,
)
from smc.util.draw_path import drawPath
from smc.control.cartesian_space.ik_solvers import getIKSolver
from smc.path_generation.cartesian_to_joint_space_path_mapper import (
    clikCartesianPathIntoJointPath,
)
from smc.util.map2DPathTo3DPlane import map2DPathTo3DPlane
from smc import getRobotFromConfig
from smc.util.calib_board_hacks import (
    calibratePlane,
)

import pinocchio as pin
import numpy as np
import matplotlib
import pickle

if __name__ == "__main__":

    #######################################################################
    #                           software setup                            #
    #######################################################################
    cfg = getArgsForDrawing()
    if not cfg.board_wiping:
        assert cfg.mm_into_board > 0.0 and cfg.mm_into_board < 5.0
    cfg.mm_into_board = cfg.mm_into_board / 1000
    print(cfg)
    robot = getRobotFromConfig(cfg)
    # NOTE: could be force sensing in joints as well, but that's not implemented yet
    assert issubclass(robot.__class__, ForceTorqueOnSingleArmWrist)
    if not cfg.real:
        rand_pertb = np.random.random(robot.nq) * 0.1
        if cfg.robot != "ur5e":
            raise NotImplementedError
        robot._q = np.array([1.32, -1.40, -1.27, -1.157, 1.76, -0.238]) + rand_pertb
        robot._step()

    #######################################################################
    #          drawing a path, making a joint trajectory for it           #
    #######################################################################

    # draw the path on the screen
    if cfg.draw_new:
        # pure evil way to solve a bug that was pure evil
        matplotlib.use("tkagg")
        pixel_path = drawPath(cfg)
        matplotlib.use("qtagg")
    else:
        #        if not cfg.board_wiping:
        pixel_path_file_path = "./parameters/path_in_pixels.csv"
        pixel_path = np.genfromtxt(pixel_path_file_path, delimiter=",")
    #        else:
    #            pixel_path_file_path = "./parameters/wiping_path.csv_save"
    #            pixel_path = np.genfromtxt(pixel_path_file_path, delimiter=",")
    # do calibration if specified
    if cfg.calibrate_plane:
        plane_pose, q_init = calibratePlane(
            cfg, robot, cfg.board_width, cfg.board_height, cfg.n_calibration_tests
        )
        print("finished calibration")
    else:
        print("using existing plane calibration")
        file = open("./parameters/plane_pose.pickle_save", "rb")
        plane_calib_dict = pickle.load(file)
        file.close()
        plane_pose = plane_calib_dict["plane_top_left_pose"]
        q_init = plane_calib_dict["q_init"]

    # make the path 3D
    path_points_3D = map2DPathTo3DPlane(pixel_path, cfg.board_width, cfg.board_height)
    if cfg.pick_up_marker:
        getMarker(cfg, robot)

    # marker moves between runs, i change markers etc,
    # this goes to start, goes down to touch the board - that's the marker
    # length aka offset (also handles incorrect frames)
    if cfg.find_marker_offset:
        # find the marker offset
        marker_offset = findMarkerOffset(plane_pose, cfg, robot)
        robot.sendVelocityCommand(np.zeros(robot.nv))
        print("marker_offset", marker_offset)
        # we're going in a bit deeper
        path_points_3D = path_points_3D + np.array(
            [0.0, 0.0, -1 * marker_offset + cfg.mm_into_board]
        )
    else:
        print("i hope you know the magic number of marker length + going into board")
        # path = path + np.array([0.0, 0.0, -0.1503])
        marker_offset = 0.066
        path_points_3D = path_points_3D + np.array(
            [0.0, 0.0, -1 * marker_offset + cfg.mm_into_board]
        )

    # create a joint space trajectory based on the 3D path
    if cfg.draw_new or cfg.calibrate_plane or cfg.find_marker_offset:
        path = []
        for i in range(len(path_points_3D)):
            path_pose = pin.SE3.Identity()
            path_pose.translation = path_points_3D[i]
            path.append(plane_pose.act(path_pose))

        if cfg.viz_test_path:
            print("""
        look at the viz now! we're constructing a trajectory for the drawing. 
        it has to look reasonable, otherwise we can't run it!
        """)
        ik_solver = getIKSolver(cfg, robot)
        joint_trajectory = clikCartesianPathIntoJointPath(
            ik_solver, path, q_init, cfg.tau0, cfg, robot
        )
        if cfg.viz_test_path:
            answer = input("did the movement of the manipulator look reasonable? [Y/n]")
            if not (answer == "Y" or answer == "y"):
                print("well if it doesn't look reasonable i'll just exit!")
                answer = False
            else:
                answer = True
        else:
            answer = True
    else:
        joint_trajectory_file_path = "./parameters/joint_trajectory.csv"
        joint_trajectory = np.genfromtxt(joint_trajectory_file_path, delimiter=",")

    if answer:
        write(joint_trajectory, cfg, robot)

    if cfg.real:
        robot.stopRobot()

    if cfg.save_log:
        robot._log_manager.saveLog()
        robot._log_manager.plotAllControlLoops()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()
