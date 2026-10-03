from enum import Enum
from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


class SignalShape(Enum):
    straight = 0
    sinusoidal = 1


@ConfigRegistry.register("test_path")
@dataclass
class ConfigTestPathGenerator:
    draw_new: bool = field(
        default=False,
        metadata={
            "help": "if true, you'll be prompted to draw a 2d path on a scratchpad. you can only do one continuous line though!"
        },
    )
    test_path_shape: str = field(
        default="sinusoidal", metadata={"help": "select shape of test path"}
    )
    test_path_length: float = field(
        default=1.0, metadata={"help": "roughly corresponds to path arclength"}
    )
    test_path_point_density: int = field(
        default=100,
        metadata={
            "help": "n points the test path consists of until i think of something better"
        },
    )
    path_height: float = field(
        default=0.5,
        metadata={"help": "heigh of the path"},
    )
    path_rotx: float = field(
        default=3.14,
        metadata={"help": "x-axis rotation along the path"},
    )
    map_width: float = field(
        default=3.0,
        metadata={
            "help": "width of the map in meters (x-axis) - only used for drawing of the path",
        },
    )
    map_height: float = field(
        default=3.0,
        metadata={
            "help": "height of the map in meters (y-axis) - only used for drawing of the path"
        },
    )
    minimum_distance_between_points: float = field(
        default=0.1,
        metadata={
            "help": "interpolation does not work well if the points are too close - it can even fail. you can filter the points so that they are of a certain minimal distance apart - that's this parameter"
        },
    )
