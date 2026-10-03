from smc.bookkeeping.load_config import GlobalConfig
from time import sleep

from multiprocessing import Process
from typing import Callable
from functools import partial
import abc


class AbstractProcessManager(abc.ABC):
    def __init__(
        self,
        cfg: GlobalConfig,
        side_function: Callable | partial,
    ):
        self.cfg: GlobalConfig = cfg
        self.side_function_name: str
        self.side_process: Process

        if type(side_function) is partial:
            self.side_process.name = side_function.func.__name__
        else:
            self.side_process.name = side_function.__name__ + "_process"

        self.start()

    def start(self):
        self.side_process.start()
        if self.cfg.debug_prints:
            print(f"PROCESS_MANAGER: i am starting {self.side_process.name}")

    @abc.abstractmethod
    def cleanExit(self): ...

    def terminateProcess(self):
        self.cleanExit()
        # kind of arbitrary number, but should exit within a control cycle basically
        sleep(0.0005)
        try:
            self.side_process.terminate()
            if self.cfg.debug_prints:
                print(f"terminated {self.side_process.name}")
        except AttributeError:
            if self.cfg.debug_prints:
                print(f"{self.side_process.name} is dead already")
