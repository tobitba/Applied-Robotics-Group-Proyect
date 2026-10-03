from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


@ConfigRegistry.register("net")
@dataclass
class ConfigNetworking:
    host: str = field(
        default="127.0.0.1",
        metadata={"help": "IP address of the host you are connecting to"},
    )
    port: int = field(
        default=6666, metadata={"help": "host's port you are connecting to"}
    )
