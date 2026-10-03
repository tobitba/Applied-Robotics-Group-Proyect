from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry
from typing import Callable, TypeAlias
from smc.robots.interfaces.single_arm_interface import SingleArmInterface
from numpy import ndarray


@ConfigRegistry.register("cs")
@dataclass
class ConfigCartesianSpace:
    ik_solver: str = field(
        default="QPVanilla",
        metadata={
            "help": "select which click algorithm you want. if you want to tune a QP, use QP, and define individaul costs. -1 means don't use it. if you want a ready-made QP with a predetermined cost, select one of the specific QPs. NOTE: this OVERRIDES the appropriate cost arguments",
            "choices": [
                "dampedPseudoinverse",
                "adaptiveDampedPseudoinverse",
                "jacobianTranspose",
                "QP",
                "QPVanilla",
                "QPWithPosture",
                "QPManipMax",
                "QPWithPostureAndEBDistance",
            ],
        },
    )
    qp_solver: str = field(
        default="quadprog",
        metadata={
            "help": "select which qp solver you want (relevant only if using qp for ik)",
            "choices": [
                "quadprog",
                "ecos",
                "proxqp",
            ],
        },
    )
    tikhonov_damp: float = field(
        default=1e-3, metadata={"help": "damping scalar in tikhonov regularization"}
    )
    vel_track_cost: float = field(
        default=-1.0, metadata={"help": "velocity tracking cost in QPs"}
    )
    posture_cost: float = field(
        default=-1.0,
        metadata={
            "help": "posture (q_ref - q) cost in QPs. rule of thumb for size: around 100 times less than main objective"
        },
    )
    manipulability_cost: float = field(
        default=-1.0,
        metadata={
            "help": "manipulability (just maxing it) cost in QPs. rule of thumb for size: around 100 times less than main objective"
        },
    )
    ebd_cost: float = field(
        default=-1.0,
        metadata={
            "help": "end-effector to base distance cost in QPs (only for mobile manipulators). rule of thumb for size: around 100 times less than main objective"
        },
    )
    respect_joint_limits: bool = field(
        default=False,
        metadata={"help": "whether the QP will also respect joint position limits"},
    )
    respect_joint_accelerations: bool = field(
        default=False,
        metadata={"help": "whether the QP will also respect joint acceleration limits"},
    )
    max_init_clik_iterations: int = field(
        default=10000,
        metadata={"help": "number of max clik iterations to get to the first point"},
    )
    max_running_clik_iterations: int = field(
        default=1000,
        metadata={"help": "number of max clik iterations between path points"},
    )
    viz_test_path: bool = field(
        default=False,
        metadata={"help": "number of max clik iterations between path points"},
    )
    K_fb: float = field(
        default=1.0,
        metadata={"help": "weight for feedback in path following"},
    )


# types
IKSolver: TypeAlias = Callable[
    [ConfigCartesianSpace, SingleArmInterface, ndarray], ndarray
]
