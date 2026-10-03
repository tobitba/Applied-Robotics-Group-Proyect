from smc import load_config
from smc import load_config
from smc.control.cartesian_space import getClikArgs
from smc.control.dmp.dmp import getDMPArgs
from smc.util.calib_board_hacks import getBoardCalibrationArgs

import argparse


def getArgsForDrawing() -> argparse.Namespace:
    parser = load_config()
    parser = getClikArgs(parser)
    parser = getDMPArgs(parser)
    parser = getBoardCalibrationArgs(parser)
    parser.description = "Make a drawing on screen,\
            watch the robot do it on the whiteboard."
    parser.add_argument(
        "--mm-into-board",
        type=float,
        help="number of milimiters the path is into the board",
        default=3.0,
    )
    parser.add_argument(
        "--draw-new",
        action=argparse.BooleanOptionalAction,
        help="whether draw a new picture, or use the saved path path_in_pixels.csv",
        default=True,
    )
    parser.add_argument(
        "--pick-up-marker",
        action=argparse.BooleanOptionalAction,
        help="""
    whether the robot should pick up the marker.
    NOTE: THIS IS FROM A PREDEFINED LOCATION.
    """,
        default=False,
    )
    parser.add_argument(
        "--find-marker-offset",
        action=argparse.BooleanOptionalAction,
        help="""
    whether you want to do find marker offset (recalculate TCP
    based on the marker""",
        default=True,
    )
    parser.add_argument(
        "--board-wiping",
        action=argparse.BooleanOptionalAction,
        help="are you wiping the board (default is no because you're writing)",
        default=False,
    )
    cfg = parser.parse_cfg()
    return cfg
