from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.base_config import ConfigBase
from smc.multiprocessing.abstract_process_manager import AbstractProcessManager
from smc.bookkeeping.types import Command, Data

from multiprocessing import Process, Queue
from typing import Callable
from copy import deepcopy


class ProducerProcess(AbstractProcessManager):
    def __init__(
        self,
        cfg: ConfigBase,
        side_function: Callable[[GlobalConfig, Command, Queue], None],
        init_command: Command,
        init_value: Data,
    ):
        self.latest_data = init_value

        self.data_queue = Queue(maxsize=5)
        self.side_process = Process(
            target=side_function,
            args=(
                cfg,
                init_command,
                self.data_queue,
            ),
        )
        super().__init__(cfg, side_function)

    def getData(self) -> Data:
        if not self.data_queue.empty():
            self.latest_data = self.data_queue.get_nowait()
        return deepcopy(self.latest_data)

    # TODO: think of how to exit cleanly and implement it.
    # the queue is shared, it can be done.
    # but then that needs to be taken care of in the
    # side process... whatever for now
    def cleanExit(self):
        pass
