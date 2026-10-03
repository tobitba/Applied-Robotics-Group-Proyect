from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


@ConfigRegistry.register("ConfigDMP")
@dataclass
class ConfigDMP:
    temporal_coupling: bool = field(
        default=True, metadata={"help": "whether you want to use temporal coupling"}
    )
    tau0: float = field(
        default=10.0,
        metadata={
            "help": "total time needed for trajectory. if you use temporal coupling, you can still follow the path even if it's too fast"
        },
    )
    gamma_nominal: float = field(
        default=1.0,
        metadata={
            "help": "positive constant for tuning temporal coupling: the higher, the fast the return rate to nominal tau"
        },
    )
    gamma_a: float = field(
        default=0.5,
        metadata={
            "help": "positive constant for tuning temporal coupling, potential term"
        },
    )
    eps_tc: float = field(
        default=0.001, metadata={"help": "temporal coupling term, should be small"}
    )
