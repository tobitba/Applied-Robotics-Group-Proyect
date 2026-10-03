import numpy as np


def trucatePathToClosestPathPoint(
    path_parameter: list[int],
    path_base: np.ndarray,
    p_base: np.ndarray,
) -> np.ndarray:
    if p_base.shape[0] == 2:
        p_base = np.append(p_base, 0.0)
    distances = np.linalg.norm(p_base - path_base, axis=1)
    index = np.argmin(distances)
    # NOTE: bullshit heuristic to deal with the path intersecting with itself.
    # i can't be bothered with anything better
    if (index - path_parameter[0]) > 0 and (index - path_parameter[0]) < 10:
        path_parameter[0] += 1
    path_base = path_base[path_parameter[0] :]
    return path_base
