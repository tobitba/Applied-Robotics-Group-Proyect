from typing import TypeAlias, Any, Callable
from numpy import ndarray
from smc.bookkeeping.types import PastData
from smc.bookkeeping.load_config import GlobalConfig
from smc.robots.abstract_robotmanager import AbstractRobotManager

# control loops
ControlLoopReturn: TypeAlias = tuple[bool, dict[str, ndarray], dict[str, ndarray]]
InnerControlLoopReturn: TypeAlias = tuple[
    ndarray, dict[str, ndarray], dict[str, ndarray]
]
ControlLoop: TypeAlias = Callable[
    [Any, GlobalConfig, AbstractRobotManager, int, PastData],
    ControlLoopReturn,
]
