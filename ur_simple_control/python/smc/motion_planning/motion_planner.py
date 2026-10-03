# i'm pretty sure this can be a single class which different swappable elements.
# maybe we'll need different ones later on, but let's keep it simple first
from smc.motion_planning.config_motion_planner import ConfigMotionPlanner
from smc.motion_planning.path_planning.path_planner import AbstractPathPlanner
from smc.motion_planning.interpolation.interpolator import AbstractInterpolator
from smc.motion_planning.trajectory_generation.trajectory_generator import (
    AbstractTrajectoryGenerator,
    AbstractTimeLaw,
)

import typing
from smc.bookkeeping.types import (
    JointReference,
    CartesianReference,
    ReferenceT,
    PointsT,
)
import abc
from copy import deepcopy

# TODO: but sometimes we don't even have a path, we only have a trajectory,
# because we're using a method that directly gives the trajectory.
# so we do need to split this into classes:
#  - MotionPlannerTrajOnly
#  - MotionPlannerFixedPath
#  - MotionPlannerUpdatingPath.
# otherwise we have to pass empty arguments,
# and that's just more difficult to parse for the user.
# i mean either way they need to spit out a reference,
# and a trajectory following controller does not need to care
# what kind of motion planning you're using, it's going to do
# the same thing anyway


class AbstractMotionPlanner(abc.ABC, typing.Generic[ReferenceT]):
    @abc.abstractmethod
    def getReference(self, t: float) -> ReferenceT:
        pass


class MotionPlannerTrajOnly(AbstractMotionPlanner):
    def __init__(self, trajectory_generator: AbstractTrajectoryGenerator):
        self.trajectory_generator = trajectory_generator

    def getReference(self, t: float) -> ReferenceT:
        return self.trajectory_generator.getCurrentReference(t)


class MotionPlannerFixedPath(AbstractMotionPlanner):
    def __init__(self, interpolator: AbstractInterpolator, time_law: AbstractTimeLaw):
        # TODO: pass path points to interpolator
        self.interpolator: AbstractInterpolator = interpolator
        self.time_law: AbstractTimeLaw = time_law

    @classmethod
    def fromConfig(cls, cfg: ConfigMotionPlanner, path_points: PointsT):
        raise NotImplementedError
        # TODO: need to implement these get from string methods.
        # ideally this is done through some import magic instead of ifs and strings
        interpolator = getInterpolatorFromString(cfg.interpolator)(path_points)
        time_law = getTimeLawFromString(cfg.time_law)(interpolator)
        return cls(interpolator, time_law)

    def getReference(self, t: float) -> ReferenceT:
        # 2. call traj generator to give current s reference
        s, sdot, sddot = self.time_law.getCurrentPathParameter(t)
        # call path for its reference
        position, velocity, acceleration = self.interpolator.getReference(s)
        return position, sdot * velocity, sddot * acceleration


class MotionPlannerUpdatingPath(AbstractMotionPlanner):
    def __init__(
        self,
        path_planner: AbstractPathPlanner,
        interpolator: AbstractInterpolator,
        time_law: AbstractTimeLaw,
        path_updating: typing.Callable,
    ):
        raise NotImplementedError
        self.path_planner: AbstractPathPlanner = path_planner
        self.interpolator: AbstractInterpolator = interpolator
        self.time_law: AbstractTimeLaw = trajectory_generator

        self.t_traj_start: float = 0.0
        self.old_path_points = self.path_planner.getPathPoints()
        self.interpolator(self.old_path_points)

    def getReference(self, t: float) -> JointReference | CartesianReference:
        # 1. compute what the t on the current trajectory is (t - t_traj_start)
        t_on_traj = t - self.t_traj_start
        # 2. call traj generator to give current s reference
        s, sdot, sddot = self.time_law.getCurrentPathParameter(t_on_traj)
        # call path for its reference
        position, velocity, acceleration = self.interpolator.getPathPoint(s)
        return position, velocity * sdot, acceleration * sddot

    def onPathUpdate(self):
        raise NotImplementedError
        # prerequisites:
        # 1. fix velocity tracking controller: no Kv, need to update code to correspond to this
        # 2.
        # option A:
        # 1. swap the paths, but keep the old one
        # 2. add an impedance term to the positional error so
        #    that you smoothly go to the next trajectory.
        #    has to work on a dynamical system level.
        #    --> but that's in control, not here
        # 3. this has the implied assumption that timing is fine.

        # new_path_points = self.path_planner.getPathPoints()
        # new_path = self.interpolator.
        # TODO: figure out how to know what time it is upon updating...
        # maybe straight up call time.time(), and keep track of that?
        # --> just do time.time(), we're on the same computer.
        # but you should note that somewhere.

    def explicitlyConstructInbetweenPath(self):
        # option B: explicitely construct new path
        # 0. get current robot pose and velocity as input
        # 1. get the new path
        # 2. update the path based on selected path_updating function
        # alternative one:
        #    2.1 interpolate old and new paths
        #    2.2 synchronize time accross them (get old and new path points which correspond to each other in time)
        #    2.3 low-pass filter from old to new path
        # alternative two:
        #    2.1 compute based on interpolated parametric curve (some continuous math)
        #      --> sounds more correct, but also more annoying -> NOT PREFEREBABLE
        # 3. interpolate on updated new path points
        # 4. traj gen that with current velocity as start
        # 5. update t_traj_start
        self.old_path = deepcopy(updated_path)
