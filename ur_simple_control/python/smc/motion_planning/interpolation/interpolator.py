from smc.bookkeeping.types import (
    PointsSE3,
    ReferenceT,
    PointT,
    PointsT,
    VelocityT,
    AccelerationT,
)

import abc
import typing
import numpy as np
from pinocchio import SE3


class AbstractInterpolator(abc.ABC, typing.Generic[PointT]):

    # has to return the path type you initialize it to.
    # could be input is 2d, but output is SE3.
    # also output should be point, velocity and acceleration.
    # but there should methods for all 3 separately
    @abc.abstractmethod
    def getPoint(self, s: float) -> PointT: ...

    @abc.abstractmethod
    def getVelocity(self, s: float) -> VelocityT: ...

    @abc.abstractmethod
    def getAcceleration(self, s: float) -> AccelerationT: ...

    @abc.abstractmethod
    def getReference(self, s: float) -> ReferenceT: ...


class AbstractSpline(AbstractInterpolator[PointT]):

    def __init__(self, points: PointsT, init_vel: VelocityT, final_vel: VelocityT):

        self.P: PointsT = points
        self.n: int = len(self.P) - 1
        self.ss: np.ndarray = np.zeros(
            self.n
        )  # parameter value at every provided path waypoint T
        if type(points[0]) is not SE3:
            self.ss[1:] = np.cumsum(np.linalg.norm(np.diff(self.P, axis=0)))
        else:
            dists = np.array([T.translation for T in points])
            self.ss[1:] = np.cumsum(np.linalg.norm(np.diff(dists, axis=0)))
        self.ss /= self.ss[-1]
        self.hs: np.ndarray = np.diff(
            self.ss
        )  # differences between s's - used to normalize s to given spline
        self.init_vel = init_vel
        self.final_vel = final_vel
        # NOTE: in a real language, this would be an array
        self.splines: list[AbstractInterpolator[PointT]]
        self.computeInterpolation()

    @abc.abstractmethod
    def computeInterpolation(self): ...

    def getSplineIndexAndParameter(self, s: float) -> tuple[int, float]:
        # 1. find spline i corresponding to parameter s
        # np.searchsorted gives the index of the first element larger than s.
        # lower s_list index corresponds to spline index
        i = np.searchsorted(self.ss, s) - 1
        # yes could be done cleaner, w/e
        if i == -1:
            return 0, 0.0

        if i == len(self.ss) - 1:
            return -1, 1.0

        # h = self.s_list[i + 1] - self.s_list[i]
        # TODO: verify h = self.hs[i]
        # local parameter for spline i
        h = max(self.hs[i], 1e-5)
        u = (s - self.ss[i]) / h
        u = np.clip(u, 0.0, 1.0)
        return i, u

    def getPoint(
        self,
        s: float,
    ) -> PointT:

        i, u = self.getSplineIndexAndParameter(s)
        return self.splines[i].getPoint(u)

    def getVelocity(
        self,
        s: float,
    ) -> VelocityT:

        i, u = self.getSplineIndexAndParameter(s)
        return self.splines[i].getVelocity(u)

    def getAcceleration(
        self,
        s: float,
    ) -> AccelerationT:

        i, u = self.getSplineIndexAndParameter(s)
        return self.splines[i].getAcceleration(u)

    def getReference(
        self,
        s: float,
    ) -> ReferenceT:

        i, u = self.getSplineIndexAndParameter(s)
        return self.splines[i].getReference(u)
