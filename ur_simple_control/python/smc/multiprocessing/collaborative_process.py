from smc.bookkeeping.load_config import GlobalConfig
from smc.multiprocessing.abstract_process_manager import AbstractProcessManager
from smc.bookkeeping.types import Command

from multiprocessing import Process, Lock, shared_memory
from multiprocessing.synchronize import Lock as SLock
from numpy import ndarray
import pickle
from copy import deepcopy
from typing import Any, Callable, TypeVar, Generic

T1 = TypeVar("T1", Command, ndarray)
T2 = TypeVar("T2", Command, ndarray)


# TODO:
# this is kinda stupid because the command has to be an ndarray,
# but the return data is anything.
# instead, it should be either an ndarray, or anything.
# the benefit of an ndarray is that it's faster to use
# because it requires no pickling.
# on the other hand, arbitrary data is slower as it needs to be pickled,
# but it can be anything you want
class CollaborativeProcess(AbstractProcessManager, Generic[T1, T2]):
    def __init__(
        self,
        cfg: GlobalConfig,
        side_function: Callable[[GlobalConfig, Command, str, SLock], None],
        init_command: T1,
        init_value: T2,
    ):
        self.latest_data = init_value
        self.shm_command: shared_memory.SharedMemory
        self.shm_data: shared_memory.SharedMemory
        self.command_type = type(init_command)
        self.data_type = type(init_value)

        # TODO: the name should be random and send over as function argument
        self.shm_cmd_name = "command"
        if self.command_type is ndarray:
            # TODO: possibly you need to set just a string and then use setattr,
            # idk if anything in memory is set up from just the type declaration
            self.shared_command_array: ndarray
            self.init_ndarray_memory(
                self.shm_command,
                self.shm_cmd_name,
                self.shared_command_array,
                init_command,
            )
        else:
            self.init_pickling_memory(self.shm_command, self.shm_cmd_name, init_command)

        # TODO: the name should be random and send over as function argument
        self.shm_data_name = "data"
        if self.data_type is ndarray:
            # TODO: possibly you need to set just a string and then use setattr,
            # idk if anything in memory is set up from just the type declaration
            self.shared_data_array: ndarray
            self.init_ndarray_memory(
                self.shm_data,
                self.shm_data_name,
                self.shared_data_array,
                init_command,
            )
        else:
            self.init_pickling_memory(self.shm_data, self.shm_data_name, init_value)

        # the process has to create its shared memory
        # same lock for both
        self.lock = Lock()
        # TODO: obviously will have to change the arguments being passed,
        # and change the functions that use this processmanager
        self.side_process = Process(
            target=side_function,
            args=(
                cfg,
                init_command,
                self.shm_cmd_name,
                self.lock,
                self.shm_data,
            ),
        )
        AbstractProcessManager.__init__(self, cfg, side_function)

    def init_ndarray_memory(
        self,
        shm: shared_memory.SharedMemory,
        shm_name: str,
        shared_array: ndarray,
        value_array: ndarray,
    ):
        # NOTE: if we didn't close properly it will just linger on.
        # since there is no exist_ok argument for SharedMemory, we catch the error on the fly here
        try:
            shm = shared_memory.SharedMemory(
                shm_name, create=True, size=value_array.nbytes
            )
        except FileExistsError:
            shm = shared_memory.SharedMemory(
                shm_name, create=False, size=value_array.nbytes
            )
        shared_array = ndarray(
            value_array.shape, dtype=value_array.dtype, buffer=shm.buf
        )
        shared_array[:] = value_array[:]

    def init_pickling_memory(
        self, shm: shared_memory.SharedMemory, shm_name: str, value_dict: dict[str, Any]
    ):
        try:
            shm = shared_memory.SharedMemory(shm_name, create=True, size=10000)
        except FileExistsError:
            shm = shared_memory.SharedMemory(shm_name, create=False, size=10000)
        # initialize empty
        p = pickle.dumps(value_dict)
        shm.buf[: len(p)] = p

    def sendCommand(self, command: T1) -> None:
        """
        sendCommand
        """
        if self.command_type is ndarray:
            assert command.shape == self.shared_command_array.shape
            self.lock.acquire()
            self.shared_command_array[:] = command[:]
            self.lock.release()

        else:
            command_pickled = pickle.dumps(command)
            self.lock.acquire()
            self.shared_command_array[: len(command_pickled)] = command_pickled
            self.lock.release()

    def getData(self) -> T2:
        if self.data_type is ndarray:
            self.lock.acquire()
            self.latest_data[:] = self.shared_data_array[:]
            self.lock.release()
        else:
            self.lock.acquire()
            # here we should only copy, release the lock, then deserialize
            self.latest_data = pickle.loads(self.shm_data.buf)
            self.lock.release()
        return deepcopy(self.latest_data)

    # TODO: befree needs to be passed here
    def cleanExit(self):
        self.shm_command.close()
        self.shm_command.unlink()
        self.shm_data.close()
        self.shm_data.unlink()
