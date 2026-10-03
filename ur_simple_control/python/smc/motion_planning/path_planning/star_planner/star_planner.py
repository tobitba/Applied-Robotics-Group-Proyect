from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry
from smc.motion_planning.path_planning.maps.premade_maps import createSampleStaticMap
from smc.motion_planning.path_planning.star_planner.star_navigation.robot.unicycle import (
    Unicycle,
)
from smc.motion_planning.path_planning.star_planner.scene_updater import SceneUpdater
from smc.motion_planning.path_planning.star_planner.path_generator import PathGenerator

from importlib.resources import files
from multiprocessing import Lock, shared_memory
import yaml
import pickle
import numpy as np


@ConfigRegistry.register("star_planner")
@dataclass
class ConfigStarPlanner:
    robot_params_file_path = str(
        files("smc.path_generation").joinpath("robot_params.yaml")
    )
    tunnel_mpc_params_file_path = str(
        files("smc.path_generation").joinpath("tunnel_mpc_params.yaml")
    )
    planning_robot_params_file: str = field(
        default=robot_params_file_path,
        # default='/home/gospodar/lund/praxis/projects/smc/python/smc/path_generation/robot_params.yaml',
        # default='/home/gospodar/colcon_venv/smc/python/smc/path_generation/robot_params.yaml',
        metadata={
            "help": "path to robot params file, required for path planning because it takes kinematic constraints into account"
        },
    )
    tunnel_mpc_params_file: str = field(
        default=tunnel_mpc_params_file_path,
        # default='/home/gospodar/lund/praxis/projects/smc/python/smc/path_generation/tunnel_mpc_params.yaml',
        # default='/home/gospodar/colcon_venv/smc/python/smc/path_generation/tunnel_mpc_params.yaml',
        metadata={
            "help": "path to mpc (in original tunnel) params file, required for path planning because it takes kinematic constraints into account"
        },
    )
    n_pol: int = field(
        default=0,
        metadata={
            "help": "IDK, TODO, rn this is just a preset hack value put into cfg for easier data transfer"
        },
    )
    np: int = field(
        default=0,
        metadata={
            "help": "IDK, TODO, rn this is just a preset hack value put into cfg for easier data transfer"
        },
    )


def pathPointFromPathParam(n_pol, path_dim, path_pol, s):
    return [
        np.polyval(path_pol[j * (n_pol + 1) : (j + 1) * (n_pol + 1)], s)
        for j in range(path_dim)
    ]


def starPlanner(goal, cfg, init_cmd, shm_name, lock: Lock, shm_data):
    """
    starPlanner
    ------------
    function to be put into ProcessManager,
    spitting out path points.
    it's wild dark dynamical system magic done by albin,
    with software dark magic on top to get it to spit
    out path points and just the path points.
    goal and path are [x,y],
    but there are utility functions to construct SE3 paths out of this
    elsewhere in the library.
    """
    # shm stuff
    shm = shared_memory.SharedMemory(name=shm_name)
    # dtype is default, but i have to pass it
    p_shared = np.ndarray((2,), dtype=np.float64, buffer=shm.buf)
    p = np.zeros(2)
    # environment
    # TODO: DIRTY HACK YOU CAN'T JUST HAVE THIS HERE !!!!!!!!!!!!!!!!
    obstacles, _ = createSampleStaticMap()

    robot_type = "Unicycle"
    with open(cfg.planning_robot_params_file) as f:
        params = yaml.safe_load(f)
    robot_params = params[robot_type]
    planning_robot = Unicycle(
        width=robot_params["width"],
        vel_min=[robot_params["lin_vel_min"], -robot_params["ang_vel_max"]],
        vel_max=[robot_params["lin_vel_max"], robot_params["ang_vel_max"]],
        name=robot_type,
    )

    with open(cfg.tunnel_mpc_params_file) as f:
        params = yaml.safe_load(f)
    mpc_params = params["tunnel_mpc"]
    mpc_params["dp_max"] = planning_robot.vmax * mpc_params["dt"]

    verbosity = 1
    scene_updater = SceneUpdater(mpc_params, verbosity)
    path_gen = PathGenerator(mpc_params, verbosity)

    # TODO: make it an argument
    convergence_threshold = 0.05
    try:
        while True:
            # has to be a blocking call
            # cmd = cmd_queue.get()
            # p = cmd['p']
            # TODO: make a sendCommand type thing which will
            # handle locking, pickling or numpy-arraying for you
            # if for no other reason than not copy-pasting code
            lock.acquire()
            p[:] = p_shared[:]
            lock.release()

            if np.linalg.norm(p - goal) < convergence_threshold:
                data_pickled = pickle.dumps("done")
                lock.acquire()
                shm_data.buf[: len(data_pickled)] = data_pickled
                # shm_data.buf[len(data_pickled):] = bytes(0)
                lock.release()
                break

            # Update the scene
            # r0 is the current position, rg is the goal, rho is a constant
            # --> why do we need to output this?
            r0, rg, rho, obstacles_star = scene_updater.update(p, goal, obstacles)
            # compute the path
            path_pol, epsilon = path_gen.update(p, r0, rg, rho, obstacles_star)
            data_pickled = pickle.dumps((path_pol, path_gen.target_path))
            lock.acquire()
            shm_data.buf[: len(data_pickled)] = data_pickled
            # shm_data.buf[len(data_pickled):] = bytes(0)
            lock.release()

    except KeyboardInterrupt:
        shm.close()
        shm.unlink()
        if cfg.debug_prints:
            print("PLANNER: caught KeyboardInterrupt, i'm out")


if __name__ == "__main__":
    pass
