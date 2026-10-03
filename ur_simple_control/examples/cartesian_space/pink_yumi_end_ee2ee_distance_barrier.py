from smc import load_config, getRobotFromConfig
from smc.robots.interfaces.dual_arm_interface import DualArmInterface
from smc.control.joint_space.joint_space_point_to_point import moveJPWTraj
from smc.util.define_random_goal import getRandomlyGeneratedGoal
from smc.robots.utils import defineGoalPointCLI
from smc.control.cartesian_space.pink_p2p import (
    DualArmIKSelfAvoidanceViaEndEffectorSpheres,
)


import pinocchio as pin

if __name__ == "__main__":
    cfg = load_config()
    robot = getRobotFromConfig(cfg)
    assert issubclass(robot.__class__, DualArmInterface)
    # go to home position
    robot.mode = robot.control_mode.upper_body
    robot._step()
    print(robot._l_ee_frame_id)
    print(robot._l_ee_frame_name)
    moveJPWTraj(robot.comfy_configuration, cfg, robot)

    if cfg.randomly_generate_goal:
        T_w_absgoal = getRandomlyGeneratedGoal(cfg, robot)
        if cfg.visualizer:
            robot.visualizer_manager.sendCommand({"Mgoal": T_w_absgoal})
    else:
        T_w_absgoal = defineGoalPointCLI(robot)
        T_w_absgoal.rotation = robot.T_w_abs.rotation.copy()
    T_absgoal_l = pin.SE3.Identity()
    T_absgoal_l.translation[1] = 0.15
    T_absgoal_r = pin.SE3.Identity()
    T_absgoal_r.translation[1] = -0.15

    # TODO: need to update specified frame
    # for new yumi urdf
    robot.mode = robot.control_mode.whole_body
    DualArmIKSelfAvoidanceViaEndEffectorSpheres(
        T_w_absgoal, T_absgoal_l, T_absgoal_r, cfg, robot
    )

    if cfg.real:
        robot.stopRobot()

    if cfg.visualizer:
        robot.killManipulatorVisualizer()

    if cfg.save_log:
        robot._log_manager.saveLog()
