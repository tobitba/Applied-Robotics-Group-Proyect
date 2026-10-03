# TODO: make a generic class for trajectory generators.
# they all take a parametrized curve, and produce a time law on it.
# if they all inherit from this, then they're easily swappable
# (and the code is nice and unifrom)
import abc
import typing
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
from smc.bookkeeping.types import ReferenceT


class AbstractTimeLaw(abc.ABC, typing.Generic[ReferenceT]):
    # this just gives t -> s , and from pathplanner.getReference(s) you get p, v, a
    @abc.abstractmethod
    def __init__(self, path: AbstractInterpolator): ...

    # just give me all yer points
    @abc.abstractmethod
    def getCurrentPathParameter(self, t: float) -> tuple[float, float, float]: ...


class AbstractTrajectoryGenerator(abc.ABC):
    # this gives you the full reference by itself
    # t -> p, v, a
    @abc.abstractmethod
    def getCurrentReference(self, t) -> ReferenceT: ...
