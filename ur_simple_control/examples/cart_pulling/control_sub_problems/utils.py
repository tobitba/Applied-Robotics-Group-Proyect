from smc import load_config
from smc.control.optimal_control.util import get_OCP_cfg
from smc.path_generation.planner import getPlanningArgs
from smc.control.cartesian_space import getClikArgs
from smc import load_config

import argparse
from pinocchio import SE3
import numpy as np


def get_cfg() -> argparse.Namespace:
    parser = load_config()
    parser = get_OCP_cfg(parser)
    parser = getClikArgs(parser)  # literally just for goal error
    parser = getPlanningArgs(parser)
    parser.add_argument(
        "--handlebar-height",
        type=float,
        default=0.5,
        help="heigh of handlebar of the cart to be pulled",
    )
    parser.add_argument(
        "--base-to-handlebar-preferred-distance",
        type=float,
        default=0.5,
        help="prefered path arclength from mobile base position to handlebar",
    )
    parser.add_argument(
        "--planner",
        action=argparse.BooleanOptionalAction,
        help="if on, you're in a pre-set map and a planner produce a plan to navigate. if off, you draw the path to be followed",
        default=True,
    )
    parser.add_argument(
        "--draw-new",
        action=argparse.BooleanOptionalAction,
        help="are you drawing a new path or reusing the previous one",
        default=False,
    )
    parser.add_argument(
        "--map-width",
        type=float,
        help="width of the map in meters (x-axis) - only used for drawing of the path",
        default=3.0,
    )
    parser.add_argument(
        "--map-height",
        type=float,
        help="height of the map in meters (y-axis) - only used for drawing of the path",
        default=3.0,
    )
    cfg = parser.parse_cfg()
    return cfg


def constructInitialT_w_abs(
    cfg: argparse.Namespace, path_base: np.ndarray, rotation: np.ndarray
) -> SE3:
    direction = path_base[1] - path_base[0]
    handlebar_direction = -1 * direction
    handlebar_direction = handlebar_direction / np.linalg.norm(handlebar_direction)
    offset = cfg.base_to_handlebar_preferred_distance * handlebar_direction
    translation = path_base[0] + offset
    translation[2] = cfg.handlebar_height
    return SE3(rotation, translation)


def initializePastData(
    cfg: argparse.Namespace, T_w_e: SE3, p_base: np.ndarray, max_base_v: float
) -> np.ndarray:
    # prepopulate past data to make base and cart be on the same path in the past
    # (which didn't actually happen because this just started)
    p_ee = T_w_e.translation[:2]
    straight_line_path = np.linspace(p_ee, p_base, cfg.past_window_size)
    # straight_line_path_timed = path2D_timed(cfg, straight_line_path, max_base_v)
    # return straight_line_path_timed # this one is shortened to cfg.n_knots! and we want the whole buffer
    return straight_line_path
