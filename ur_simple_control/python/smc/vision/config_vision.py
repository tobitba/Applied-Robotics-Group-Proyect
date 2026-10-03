from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


@ConfigRegistry.register("vision")
@dataclass
class ConfigVision:
    device: str = field(
        default="0",
        metadata={
            "help": "the device you are using in OpenCV's VideoCapture(device). 0 for computer's integrated camera, an URI for only video feed, ex. http://192.168.219.153:8080/video"
        },
    )
