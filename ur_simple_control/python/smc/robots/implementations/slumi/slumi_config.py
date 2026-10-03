from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


@ConfigRegistry.register("slumi")
@dataclass
class ConfigSlumi:
    host: str = field(default="0.0.0.0", metadata={"help": "host ip address"})
    port_data: int = field(
        default=6666, metadata={"help": "host's port for receiving joint positions"}
    )
    port_command: int = field(
        default=7777,
        metadata={"help": "host's port for sending joint velocity commands"},
    )
    sending_frequency: int = field(
        default=250, metadata={"help": "recv/send freq for testing"}
    )
    real_for_real: bool = field(
        default=True,
        metadata={
            "help": "if yes, we are connected to the real robot. if not, we start roscore and simulation bringup launch in the translation layer"
        },
    )
    manual_translation_start: bool = field(
        default=True,
        metadata={
            "help": "if yes, we are connected to the real robot. if not, we start roscore and simulation bringup launch in the translation layer"
        },
    )
