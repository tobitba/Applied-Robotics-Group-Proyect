from smc.bookkeeping.load_config import GlobalConfig
from smc.multiprocessing.abstract_process_manager import AbstractProcessManager
from smc.multiprocessing.consumer_process import ConsumerProcess
from smc.multiprocessing.producer_process import ProducerProcess
from smc.bookkeeping.types import Command

from multiprocessing import Process, Queue
from typing import Any, Callable


class ProducerConsumerProcess(ProducerProcess, ConsumerProcess):
    def __init__(
        self,
        cfg: GlobalConfig,
        side_function: Callable[[GlobalConfig, Command, Queue, Queue], None],
        init_command: Command,
        init_value: Any,
    ):
        self.latest_data = init_value
        self.command_queue = Queue(maxsize=5)
        self.data_queue = Queue(maxsize=5)
        self.side_process = Process(
            target=side_function,
            args=(
                cfg,
                init_command,
                self.command_queue,
                self.data_queue,
            ),
        )
        AbstractProcessManager.__init__(self, cfg, side_function)
