from dataclasses import dataclass, fields
from typing import TYPE_CHECKING
from smc.bookkeeping.registry import ConfigRegistry
import argparse
import yaml
from sys import version_info

# TODO: need to check this when loading slumi
# from importlib.util import find_spec
# if find_spec("google") and find_spec("docker"):
#    from smc.robots.implementations.slumi.slumi_real import (
#        RealSLuMiRobotManager,
#        getSLuMiConnectionArgs,
#    )

# TODO: need to automatically set this if there's a log registered
# these are obligatory
# cfg.visualizer = False
# cfg.real = False
# cfg.simulation = False
# return cfg


# TODO: prolly want to make this less ugly eventually,
# but it is what it is for now, who cares, it works
if TYPE_CHECKING:

    from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
    from smc.control.cartesian_space.cartesian_space_compliant_control_config import (
        ConfigCartesianSpaceCompliant,
    )
    from smc.control.optimal_control.oc_config import ConfigOptimalControl
    from smc.control.dmp.dmp_config import ConfigDMP
    from smc.motion_planning.path_planning.test_planners.test_planner_config import (
        ConfigTestPathGenerator,
    )
    from smc.multiprocessing.networking.config_networking import ConfigNetworking
    from smc.robots.implementations.slumi.slumi_config import ConfigSlumi
    from smc.util.calib_board_hacks import ConfigBoardCalibration

    class GlobalConfig:
        robot: str
        mode: str
        real: bool
        robot_ip: str
        ctrl_freq: int
        visualizer: bool
        viz_update_rate: int
        plotter: bool
        gripper: str
        max_iterations: int
        start_from_current_pose: bool
        acceleration: float
        max_v_percentage: float
        randomly_generate_goal: bool
        debug_prints: bool
        save_log: bool
        save_dir: str
        run_name: str
        index_runs: bool
        past_window_size: int
        controller_speed_scaling: float
        contact_detecting_force: float
        minimum_detectable_force_norm: float
        visualize_collision_approximation: bool
        config_file: str
        cs: ConfigCartesianSpace
        compliance: ConfigCartesianSpaceCompliant
        ocp: ConfigOptimalControl
        dmp: ConfigDMP
        net: ConfigNetworking
        test_path: ConfigTestPathGenerator
        board_calib: ConfigBoardCalibration
        slumi: ConfigSlumi
        goal_error: float
        load_log_file: str

else:

    class GlobalConfig:  # type: ignore[attr-defined]
        def __init__(
            self, registry_dict: dict, yaml_data: dict, cli_args: argparse.Namespace
        ):
            # 1. We loop through every registered sub-config and set defaults
            for name, config_cls in registry_dict.items():
                conf_obj = config_cls()
                if name != "base":
                    setattr(self, name, conf_obj)
                else:
                    for name, value in conf_obj.__dict__.items():
                        setattr(self, name, value)

            # 2. Layer in YAML data if the section exists
            # NOTE: yaml has to be sorted by group
            # (ex. Kp could be a parameter of anything otherwise)
            if len(yaml_data) > 0:
                for name in yaml_data.keys():
                    # need to select mine
                    config_cls = None
                    if name != "base":
                        config_cls = getattr(self, name)
                    for key, value in yaml_data[name].items():
                        if name != "base":
                            if hasattr(config_cls, key):
                                setattr(config_cls, key, value)
                            else:
                                raise AttributeError(
                                    "you specified a parameter in the yaml config file which does not exist in available config parameters!"
                                )
                        else:
                            setattr(self, key, value)

            # 3. Layer in CLI args
            for key, value in vars(cli_args).items():
                if value is None:
                    continue
                if "." in key:
                    sub_name, attr = key.split(".", 1)
                    sub_config = getattr(self, sub_name, None)
                    if sub_config is not None and hasattr(sub_config, attr):
                        setattr(sub_config, attr, value)
                else:
                    if hasattr(self, key):
                        setattr(self, key, value)

        def __repr__(self):
            # Sweet helper to see the whole tree when you print(cfg)
            return "\n".join([f"{k}: {v}" for k, v in self.__dict__.items()])


def load_config() -> GlobalConfig:
    """
    Loads the config dynamically based on the used library modules.
    The order of importance is as follows:
    1. arguments
    2. provided config.yaml file
    3. defaults

    Note that for the purpose of autocompletion, and thus ease of use,
    all configurations of all modules are loaded.
    If the Config object is created dynamically,
    the static typechecker can't help.
    """

    # 1. Setup appropriate argparse (per module) and then parse cfg
    parser = argparse.ArgumentParser(
        description="Run something with Simple Manipulator Control"
    )
    for name, config_cls in ConfigRegistry._registry.items():
        group = parser.add_argument_group(f"{name}")
        for entry in fields(config_cls):
            arg_grp_name = "" if name == "base" else f"{name}."
            # TODO: need to add option to parse lists, i.e. multiple values for one argument
            # TODO: add option for when an exact number of parameters needs to be provided
            kwargs = {}
            if type(entry.default) is list:
                kwargs["nargs"] = "+"
            kwargs["choices"] = (
                entry.metadata["choices"]
                if "choices" in entry.metadata.keys()
                else None
            )
            kwargs["action"] = (
                "store" if entry.type is not bool else argparse.BooleanOptionalAction
            )
            kwargs["type"] = entry.type
            kwargs["help"] = entry.metadata["help"]
            if version_info.minor > 13 and kwargs["action"] != "store":
                group.add_argument(
                    f"--{arg_grp_name}{entry.name}",
                    action=kwargs["action"],
                    help=entry.metadata["help"],
                )
            else:
                group.add_argument(f"--{arg_grp_name}{entry.name}", **kwargs)

    args = parser.parse_args()

    # 3. First override with YAML config (if provided)
    yaml_config = {}
    if args.config_file:
        with open(args.config_file, "r") as f:
            yaml_config = yaml.safe_load(f)

    ## Create the Class at RUNTIME
    cfg = GlobalConfig(ConfigRegistry._registry, yaml_config, args)

    return cfg


# NOTE: while preferable, this leads to circular imports
# literally by design. so no can do mister.
# load all the configs to have autocomletion
# from smc.bookkeeping.base_config import ConfigBase
# from smc.control.cartesian_space.cartesian_space_config import ConfigCartesianSpace
# from smc.control.optimal_control.oc_config import ConfigOptimalControl
# from smc.control.dmp.dmp_config import ConfigDMP
# from smc.motion_planning.path_planning.test_planners.test_planner_config import (
#    ConfigTestPathGenerator,
# )
# from smc.robots.implementations.slumi.slumi_config import ConfigSlumi
# from smc.util.calib_board_hacks import ConfigBoardCalibration
# @dataclass
# class MasterConfig(ConfigBase):
#    cs: ConfigCartesianSpace
#    ocp: ConfigOptimalControl
#    dmp: ConfigDMP
#    test_path: ConfigTestPathGenerator
#    board_calib: ConfigBoardCalibration
#    slumi: ConfigSlumi
#
#    def update_from_dict(self, data: dict):
#        """Update existing config with a dictionary of new values."""
#        for key, value in data.items():
#            if hasattr(self, key) and value is not None:
#                setattr(self, key, value)


# def load_master_config() -> MasterConfig:
#    """
#    Loads the config dynamically based on the used library modules.
#    The order of importance is as follows:
#    1. arguments
#    2. provided config.yaml file
#    3. defaults
#
#    Note that for the purpose of autocompletion, and thus ease of use,
#    all configurations of all modules are loaded.
#    If the Config object is created dynamically,
#    the static typechecker can't help.
#    """
#    # 1. Initialize with defaults
#    cfg = MasterConfig(
#        ConfigBase(),
#        ConfigCartesianSpace(),
#        ConfigOptimalControl(),
#        ConfigDMP(),
#        ConfigTestPathGenerator(),
#        ConfigBoardCalibration(),
#        ConfigSlumi(),
#    )
#
#    # 2. Setup appropriate argparse (per module) and then parse cfg
#    parser = argparse.ArgumentParser(
#        description="Run something with Simple Manipulator Control"
#    )
#    for name, config_cls in ConfigRegistry._registry.items():
#        group = parser.add_argument_group(f"{name}")
#        for entry in fields(config_cls):
#            arg_grp_name = "" if name == "base" else f"{name}-"
#            choices = (
#                entry.metadata["choices"]
#                if "choices" in entry.metadata.keys()
#                else None
#            )
#            action = (
#                "store" if entry.type is not bool else argparse.BooleanOptionalAction
#            )
#            group.add_argument(
#                f"--{arg_grp_name}{entry.name}",
#                type=entry.type,
#                action=action,
#                help=entry.metadata["help"],
#                choices=choices,
#            )
#            group.add_argument(f"--{name}-{entry.name}", type=entry.type)
#
#    args = parser.parse_args()
#
#    # 3. First override with YAML config (if provided)
#    if args.config_file:
#        with open(args.config_file, "r") as f:
#            yaml_config = yaml.safe_load(f)
#            cfg.update_from_dict(yaml_config)
#
#    # 4. Then override with CLI arguments
#    # We convert cfg to a dict and filter out None values/config_file
#    cli_data = {
#        k: v for k, v in vars(cfg).items() if v is not None and k != "config_file"
#    }
#    cfg.update_from_dict(cli_data)
#
#    # NOTE: alternative dynamicly constructed config from registries
#    # instead of a master config
#    # Generate a list of (name, type, default) tuples for the registry
#    # fields_for_master = []
#    # for name, config_cls in ConfigRegistry._registry.items():
#    #    # We create a field that defaults to an instance of that sub-config
#    #    fields_for_master.append((name, config_cls, field(default_factory=config_cls)))
#
#    ## Create the Class at RUNTIME
#    # RuntimeMasterConfig = make_dataclass("GlobalConfig", fields_for_master)
#
#    # Instantiate it
#    # cfg = RuntimeMasterConfig()
#
#    return cfg
