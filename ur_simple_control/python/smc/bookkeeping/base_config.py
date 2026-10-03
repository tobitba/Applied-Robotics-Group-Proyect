from dataclasses import dataclass, field
from smc.bookkeeping.registry import ConfigRegistry


@ConfigRegistry.register("base")
@dataclass
class ConfigBase:
    robot: str = field(
        default="ur5e",
        metadata={
            "help": "which robot you're running or simulating. if you are extending beyond already supported robots and load directly instead of from arguments, use the term custom (required to not break stuff, ex. visualizer loading)",
            "choices": [
                "ur5e",
                "heron",
                "heronfullyactuatedbase",
                "yumi",
                "myumi",
                "mir",
                "mirfullyactuated",
                "slumi",
                "custom",
            ],
        },
    )
    mode: str = field(
        default="whole_body",
        metadata={
            "help": "control mode the robot should run in, i.e. select if you're controlling only a subset of the available joints. most of these only make sense for mobile robots with arms of course (you get an error upon an erroneous selection)",
            "choices": [
                "whole_body",
                "base_only",
                "upper_body",
                "left_arm_only",
                "right_arm_only",
            ],
        },
    )
    real: bool = field(
        default=False,
        metadata={"help": "whether you're running on the real robot or not"},
    )
    robot_ip: str = field(
        default="127.0.0.1",
        metadata={
            "help": "robot's ip address (only needed if running on the real robot)"
        },
    )
    ctrl_freq: int = field(
        default=500,
        metadata={
            "help": "frequency of the control loop. select -1 if you want to go as fast as possible (useful for running tests in sim)"
        },
    )
    visualizer: bool = field(
        default=True,
        metadata={
            "help": "whether you want to visualize the manipulator and workspace with meshcat"
        },
    )
    viz_update_rate: int = field(
        default=-1,
        metadata={
            "help": "frequency of visual updates. visualizer and plotter update every viz-update-rate^th iteration of the control loop. put to -1 to get a reasonable heuristic"
        },
    )
    plotter: bool = field(
        default=True,
        metadata={
            "help": "whether you want to have some real-time matplotlib graphs (parts of log_dict you select)"
        },
    )
    gripper: str = field(
        default="none",
        metadata={
            "help": "gripper you're using (no gripper is the default)",
            "choices": ["none", "robotiq", "onrobot", "rs485"],
        },
    )
    max_iterations: int = field(
        default=100000,
        metadata={"help": "maximum allowable iteration number (it runs at 500Hz)"},
    )
    start_from_current_pose: bool = field(
        default=False,
        metadata={
            "help": "if connected to the robot, read the current pose and set it as the initial pose for the robot. \
                 very useful and convenient when running simulation before running on real"
        },
    )
    acceleration: float = field(
        default=0.3,
        metadata={
            "help": "robot's joints acceleration. scalar positive constant, max 1.7, and default 0.3. \
                   BE CAREFUL WITH THIS. the urscript doc says this is 'lead axis acceleration'.\
                   TODO: check what this means"
        },
    )
    max_v_percentage: float = field(
        default=0.3,
        metadata={
            "help": "select the percentage of the maximum joint velocity the robot can achieve to be the control input maximum (control inputs are clipped to perc * max_v)"
        },
    )
    goal_error: float = field(
        default=1e-2, metadata={"help": "the final position error you are happy with"}
    )
    randomly_generate_goal: bool = field(
        default=False,
        metadata={
            "help": "either be prompted for a goal point, or randomly generate one"
        },
    )
    debug_prints: bool = field(
        default=False, metadata={"help": "print some debug info"}
    )
    save_log: bool = field(
        default=False,
        metadata={
            "help": "whether you want to save the log of the run. it saves \
                        what you pass to ControlLoopManager. check other parameters for saving directory and log name."
        },
    )
    save_dir: str = field(
        default="./data",
        metadata={
            "help": "path to where you store your logs. default is ./data, but if that directory doesn't exist, then /tmp/data is created and used."
        },
    )
    run_name: str = field(
        default="latest_run",
        metadata={
            "help": "name the whole run/experiment (name of log file). note that indexing of runs is automatic and under a different argument."
        },
    )
    index_runs: bool = field(
        default=False,
        metadata={
            "help": "if you want more runs of the same name, this option will automatically assign an index to a new run (useful for data collection)."
        },
    )
    past_window_size: int = field(
        default=5, metadata={"help": "how many timesteps of past data you want to save"}
    )
    controller_speed_scaling: float = field(
        default=1.0, metadata={"help": "not actually_used atm"}
    )
    contact_detecting_force: float = field(
        default=2.8,
        metadata={
            "help": "the force used to detect contact (collision) in the moveUntilContact function"
        },
    )
    minimum_detectable_force_norm: float = field(
        default=3.0,
        metadata={
            "help": "we need to disregard noise to converge despite filtering. \
                  a quick fix is to zero all forces of norm below this argument threshold."
        },
    )
    visualize_collision_approximation: bool = field(
        default=False,
        metadata={
            "help": "whether you want to visualize the collision approximation used in controllers with obstacle avoidance"
        },
    )
    config_file: str = field(
        default="",
        metadata={
            "help": "path to config file you want. arguments take prescedence over anything in that file. it's very convenient if you have to set a lot of arguments, do it. just be careful to group the arguments according to module they come from!"
        },
    )
    load_log_file: str = field(
        default="",
        metadata={
            "help": "if doing log analysis, specify the path to the log file you want to load"
        },
    )
