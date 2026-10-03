from smc.bookkeeping.load_config import GlobalConfig
from smc.multiprocessing.abstract_process_manager import AbstractProcessManager
from smc.bookkeeping.types import Command

from multiprocessing import Process, Queue
from queue import Full
from typing import Callable


class ConsumerProcess(AbstractProcessManager):
    def __init__(
        self,
        cfg: GlobalConfig,
        side_function: Callable[[GlobalConfig, Command, Queue], None],
        init_command: Command,
    ):
        self.command_queue = Queue(maxsize=5)
        self.side_process = Process(
            target=side_function,
            args=(
                cfg,
                init_command,
                self.command_queue,
            ),
        )
        super().__init__(cfg, side_function)

    def sendCommand(self, command: Command):
        try:
            self.command_queue.put_nowait(command)
        except Full:
            pass

    # TODO: need to think about how do this correctly...
    def cleanExit(self):
        # if self.cfg.debug_prints:
        #    print(
        #        f"i am putting befree in {self.side_process.name}'s command queue to stop it"
        #    )
        # attempt clean exit
        # TODO: this fails due to Full
        # try to fix it so that we are not just terminating
        # self.command_queue.put_nowait("befree")
        pass
