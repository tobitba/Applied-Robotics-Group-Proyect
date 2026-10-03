from dataclasses import dataclass, field
from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.registry import ConfigRegistry
from smc import getRobotFromConfig
from smc.multiprocessing import ConsumerProcess
from smc.logging.logger import Logger
from smc.visualization.visualizer import manipulatorComparisonVisualizer
from smc.visualization.plotters import logPlotter

from functools import partial
import numpy as np
import os
import time
from itertools import zip_longest


@ConfigRegistry.register("log")
@dataclass
class ConfigLog:
    file1: str = field(
        default="", metadata={"help": "first log file to load for comparison"}
    )
    file2: str = field(
        default="", metadata={"help": "second log file to load for comparison"}
    )


# TODO: use ProcessManager for this
class ManipulatorComparisonManager:
    """
    visualizes 2 control loops simulateneously + arbitrary plots.
    requires that the robots are the same.
    makes most sense if you are solving the same task with a different controller,
    or with different parameters on the same controller.
    the key thing this class brings to the table is the micro-management of what to show
    when due to the fact that you might have different controlloops in one example
    and them having different total times.
    """

    def __init__(self, cfg: GlobalConfig):
        self.cfg = cfg
        # these are obligatory
        cfg.visualize_manipulator = False
        cfg.pinocchio_only = True
        cfg.simulation = False

        if os.path.exists(cfg.log.file1):
            self.logm1 = Logger(None)
            self.logm1.loadLog(cfg.log.file1)
        else:
            print("you did not give me a valid path for log1, exiting")
            exit()
        if os.path.exists(cfg.log.file2):
            self.logm2 = Logger(None)
            self.logm2.loadLog(cfg.log.file2)
        else:
            print("you did not give me a valid path for log2, exiting")
            exit()

        cfg.robot = self.logm1.cfg.robot
        self.robot1 = getRobotFromConfig(cfg)
        self.robot2 = getRobotFromConfig(cfg)

        # no two loops will have the same amount of timesteps.
        # we need to store the last available step for both robots.
        self.lastq1 = np.zeros(self.robot1.model.nq)
        self.lastq2 = np.zeros(self.robot2.model.nq)

        side_function = partial(
            manipulatorComparisonVisualizer,
            self.robot1.model,
            self.robot1.visual_model,
            self.robot1.collision_model,
        )
        cmd = (np.zeros(self.robot1.nq), np.ones(self.robot2.nq))
        self.visualizer_manager = ConsumerProcess(cfg, side_function, cmd)
        # wait until it's ready (otherwise you miss half the sim potentially)
        # 5 seconds should be more than enough,
        # and i don't want to complicate this code by complicating IPC
        time.sleep(5)

        ###########################################
        #  in case you will want log plotters too  #
        ###########################################
        self.log_plotters = []

    # NOTE i assume what you want to plot is a time-indexed with
    # the same frequency that the control loops run in.
    # if it's not then it's pointless to have a running plot anyway.
    # TODO: put these in a list so that we can have multiple plots at the same time
    class RunningPlotter:
        def __init__(self, cfg, log, log_plotter_time_start, log_plotter_time_stop):
            self.time_start = log_plotter_time_start
            self.time_stop = log_plotter_time_stop
            side_function = partial(logPlotter, log)
            self.plotter_manager = ConsumerProcess(cfg, side_function, None)

    def createRunningPlot(self, log, log_plotter_time_start, log_plotter_time_stop):
        self.log_plotters.append(
            self.RunningPlotter(
                self.cfg, log, log_plotter_time_start, log_plotter_time_stop
            )
        )

    def updateViz(self, q1, q2, time_index):
        self.visualizer_manager.sendCommand({"q1": q1, "q2": q2})
        for log_plotter in self.log_plotters:
            if (time_index >= log_plotter.time_start) and (
                time_index < log_plotter.time_stop
            ):
                log_plotter.plotter_manager.sendCommand(
                    time_index - log_plotter.time_start
                )

    # NOTE: this uses slightly fancy python to make it bareable to code
    # NOTE: dict keys are GUARANTEED to be in insert order from python 3.7 onward
    def visualizeWholeRuns(self):
        time_index = 0
        for control_loop1, control_loop2 in zip_longest(
            self.logm1.loop_logs, self.logm2.loop_logs
        ):
            print(f"run {self.logm1.cfg.run_name}, controller: {control_loop1}")
            print(f"run {self.logm2.cfg.run_name}, controller: {control_loop2}")
            # relying on python's default thing.toBool()
            if not control_loop1:
                print(f"run {self.logm1.cfg.run_name} is finished")
                q1 = self.lastq1
                for q2 in self.logm2.loop_logs[control_loop2]["qs"]:
                    self.updateViz(q1, q2, time_index)
                    time_index += 1
                print(f"run {self.logm2.cfg.run_name} is finished")
            if not control_loop2:
                print(f"run {self.logm2.cfg.run_name} is finished")
                q2 = self.lastq2
                for q1 in self.logm1.loop_logs[control_loop1]["qs"]:
                    self.updateViz(q1, q2, time_index)
                    time_index += 1
                print(f"run {self.logm1.cfg.run_name} is finished")
            if control_loop1 and control_loop2:
                for q1, q2 in zip_longest(
                    self.logm1.loop_logs[control_loop1]["qs"],
                    self.logm2.loop_logs[control_loop2]["qs"],
                ):
                    if not (q1 is None):
                        self.lastq1 = q1
                    if not (q2 is None):
                        self.lastq2 = q2
                    self.updateViz(self.lastq1, self.lastq2, time_index)
                    time_index += 1


# TODO: update
if __name__ == "__main__":
    cfg = getLogComparisonArgs()
#    cmp_manager = ManipulatorComparisonManager(cfg)
#    log_plot = {"random_noise": np.random.normal(size=(1000, 2))}
#    cmp_manager.createRunningPlot(log_plot, 200, 1200)
#    cmp_manager.visualizeWholeRuns()
#    time.sleep(100)
#    cmp_manager.manipulator_visualizer_cmd_queue.put("befree")
#    print("main done")
#    time.sleep(0.1)
#    cmp_manager.manipulator_visualizer_process.terminate()
#    if cfg.debug_prints:
#        print("terminated manipulator_visualizer_process")
