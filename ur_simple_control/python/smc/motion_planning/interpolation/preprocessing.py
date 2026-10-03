import numpy as np
from pinocchio import SE3


def reduceSE3PathPoints(minimum_distance: float, T_list: list[SE3]) -> list[SE3]:
    """
    reducePathPoints
    ----------------
    sometimes we get paths with way too many points then we care about.
    the curve traced out by these points can also be somewhat noisy.
    this is the case for example when defining mobile base paths by drawing a path.
    this leads can lead to big changes in orientation over a small distance.
    in other words we want the interpolation to "smooth out" the reference itself,
    and the simplest way to do it is to simply reduce the number of points in the path.
    if the interpolation is any good, it will produce a low curvature path.
    """
    T_list_reduced: list[SE3] = []
    T_list_reduced.append(T_list[0])
    p2p_arclength = 0.0
    # NOTE: ideally there should be handling to make sure the final point is
    # in the reduced list.
    # i can't be bothered though.
    for i in range(1, len(T_list)):
        p2p_arclength += np.linalg.norm(
            T_list[i].translation - T_list[i - 1].translation
        )
        if p2p_arclength > minimum_distance:
            T_list_reduced.append(T_list[i])
            p2p_arclength = 0.0

    return T_list_reduced


def reduce2DPathPoints(minimum_distance: float, p_list: np.ndarray) -> np.ndarray:
    """
    reduce2DPathPoints
    ----------------
    sometimes we get paths with way too many points then we care about.
    the curve traced out by these points can also be somewhat noisy.
    this is the case for example when defining mobile base paths by drawing a path.
    this leads can lead to big changes in orientation over a small distance.
    in other words we want the interpolation to "smooth out" the reference itself,
    and the simplest way to do it is to simply reduce the number of points in the path.
    if the interpolation is any good, it will produce a low curvature path.
    """
    p_list_reduced: list[SE3] = []
    p_list_reduced.append(p_list[0])
    p2p_arclength = 0.0
    # NOTE: ideally there should be handling to make sure the final point is
    # in the reduced list.
    # i can't be bothered though.
    for i in range(1, len(p_list)):
        p2p_arclength += np.linalg.norm(p_list[i] - p_list[i - 1])
        if p2p_arclength > minimum_distance:
            p_list_reduced.append(p_list[i])
            p2p_arclength = 0.0

    return np.array(p_list_reduced)
