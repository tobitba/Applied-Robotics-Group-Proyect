from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry
from smc.robots.interfaces.single_arm_interface import SingleArmInterface


@ConfigRegistry.register("compliance")
@dataclass
class ConfigCartesianSpaceCompliant:
    alpha: float = field(
        default=0.01, metadata={"help": "force feedback proportional coefficient"}
    )
    beta: float = field(
        default=1.0, metadata={"help": "low-pass filter beta parameter"}
    )
    Kp: float = field(
        default=1.0,
        metadata={"help": "proportial control constant for position errors"},
    )
    Kv: float = field(default=0.001, metadata={"help": "damping in impedance control"})
    z_only: bool = field(
        default=False,
        metadata={"help": "whether you have general impedance or just ee z axis"},
    )
