# TODO: make a generic class for path planners
# they all take a map, and produce a set of points/curve.
# if they all inherit from this, then they're easily swappable
# (and the code is nice and unifrom)
import abc
import typing
from smc.bookkeeping.types import PointT, PointsT, ReferenceT


# TODO: i mean we could have a path in joint space, no?
class AbstractPathPlanner(abc.ABC, typing.Generic[PointT]):
    # NOTE: map could (should) be updating!
    # but i guess it's better if the map itself owns
    # a process which updates the map.
    # and then we just request the latest map from the map class.

    def __init__(self, map: typing.Any, start_point: PointT, goal_point: PointT):
        self.map = map
        self.start_point = start_point
        self.goal_point = goal_point

    # TODO: but this should also have a goal point, no?
    @abc.abstractmethod
    def computePath(self, start_point: PointT):
        pass

    # @abc.abstractmethod
    # def getPathPoint(self) -> ReferenceT:
    #    pass

    @abc.abstractmethod
    def getPathPoints(self) -> PointsT:
        pass
