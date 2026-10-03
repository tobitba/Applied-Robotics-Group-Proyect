import numpy as np
import typing
from typing import TypeAlias, Annotated, Any
from collections import deque
from pinocchio import SE3

# dealing with annoying literals
floatt: TypeAlias = float | np.floating[Any]

# basic robotics quantities
NQ = "NQ"
NV = "NV"

JointPositions: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], NQ]
JointVelocities: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], NV]
JointAccelerations: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], NV]
# TODO: ideally you should replace this with pin.Motion everywhere.
# makes no difference for any math (numpy autoexposed), but if you need a method it's there
Twist: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], "6"]


# log
LogItem: TypeAlias = dict[str, np.ndarray]
LoopLog: TypeAlias = dict[str, np.ndarray]
RunLog: TypeAlias = dict[str, LoopLog]
# past window
PastItem: TypeAlias = dict[str, np.ndarray]
PastData: TypeAlias = dict[str, deque[np.ndarray]]


# motion planning
N = "N"  # as in N points
JointReference: TypeAlias = tuple[JointPositions, JointVelocities, JointAccelerations]
CartesianReference: TypeAlias = tuple[SE3, Twist, Twist]
ReferenceT = typing.TypeVar("ReferenceT", JointReference, CartesianReference)

Point2D: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], "1 x 2"]
Point3D: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], "1 x 2"]
# TODO: but these could also be joint angles no?
PointT = typing.TypeVar("PointT", Point2D, Point3D, SE3)
Points2D: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], "N x 2"]
Points3D: TypeAlias = Annotated[np.ndarray[Any, np.dtype[np.float64]], "N x 3"]
PointsSE3: TypeAlias = list[SE3]
PointsT = typing.TypeVar("PointsT", JointPositions, Points2D, Points3D, PointsSE3)
VelocityT = typing.TypeVar("VelocityT", JointVelocities, Twist)
AccelerationT = typing.TypeVar("AccelerationT", JointAccelerations, Twist)

# multiprocessing and networking
Data: TypeAlias = typing.Dict[str, Any]
Command: TypeAlias = typing.Dict[str, Any]
# TODO: this is not correct - these depend on the specific IPC used...
# so there could be 2 queues for example
# SideFunction: TypeAlias = (
#    typing.Callable[[GlobalConfig, Command, Queue], None] | partial
# )
