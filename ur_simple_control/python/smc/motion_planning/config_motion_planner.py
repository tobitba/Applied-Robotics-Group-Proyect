from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


@ConfigRegistry.register("mp")
@dataclass
class ConfigMotionPlanner:
    path_planner: str = field(
        default="star_planner",
        metadata={
            "help": "select the path planner. NOTE: currently broken",
            "choices": ["star_planner"],
        },
    )
    interpolator: str = field(
        default="CubicSplineSE3",
        metadata={
            "help": "select the inteporlator",
            "choices": ["CubicSplineSE3", "CubicSpline2D", "LinearInterpolator"],
        },
    )
    # TODO: have to put in at least generic cubic etc
    time_law: str = field(
        default="uniform",
        metadata={
            "help": "select the time law",
            "choices": ["uniform"],
        },
    )
