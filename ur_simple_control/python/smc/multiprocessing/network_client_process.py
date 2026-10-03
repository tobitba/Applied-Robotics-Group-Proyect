from smc.bookkeeping.load_config import GlobalConfig
from smc.bookkeeping.types import Command
from smc.multiprocessing.consumer_process import ConsumerProcess
from smc.multiprocessing.abstract_process_manager import AbstractProcessManager
from smc.multiprocessing.networking.util import DictPb2EncoderDecoder

from multiprocessing import Process, Lock, shared_memory
from multiprocessing.synchronize import Lock as SLock
import numpy as np
import pickle
import typing
from copy import deepcopy


# TODO: as it stands, this thing has absolutely nothing to do with networking.
# it's just a client process that uses shared memory instead of a queue.
# it can even be argued that a queue is preferred over shared memory.
# so this needs to be at least renamed
class NetworkClientProcess(ConsumerProcess):
    def __init__(
        self,
        cfg: GlobalConfig,
        side_function: typing.Callable[[GlobalConfig, Command, str, SLock], None],
        init_value: dict[str, typing.Any],
    ):
        self.init_value = init_value

        self.encoder_decoder = DictPb2EncoderDecoder()
        self.msg_code = self.encoder_decoder.dictToMsgCode(init_value)
        # TODO: the name should be random and send over as function argument
        shm_name = "client_socket" + str(np.random.randint(0, 1000))
        # NOTE: size is max size of the recv buffer too,
        # and the everything blows up if you manage to fill it atm
        self.shm_msg = shared_memory.SharedMemory(shm_name, create=True, size=1024)
        # need to initialize shared memory with init value
        # NOTE: EVIL STUFF SO PICKLING ,READ NOTES IN networking/client.py
        # init_val_as_msg = self.encoder_decoder.dictToSerializedPb2Msg(init_value)
        # self.shm_msg.buf[:len(init_val_as_msg)] = init_val_as_msg
        pickled_init_value = pickle.dumps(self.init_value)
        self.shm_msg.buf[: len(pickled_init_value)] = pickled_init_value
        self.lock = Lock()
        self.side_process = Process(
            target=side_function, args=(cfg, init_value, shm_name, self.lock)
        )
        AbstractProcessManager.__init__(self, cfg, side_function)

    def getData(self) -> dict[str, typing.Any]:
        self.lock.acquire()
        # data_copy = deepcopy(self.shm_msg.buf)
        # REFUSES TO WORK IF YOU DON'T PRE-CROP HERE!!!
        # MAKES ABSOLUTELY NO SENSE!!! READ MORE IN smc/networking/client.py
        # so we're decoding there, pickling, and now unpickling.
        # yes, it's incredibly stupid.
        # new_data = self.encoder_decoder.serializedPb2MsgToDict(self.shm_msg.buf, self.msg_code)
        new_data = pickle.loads(self.shm_msg.buf)
        self.lock.release()
        if len(new_data) > 0:
            self.latest_data = new_data
        # print("new_data", new_data)
        # print("self.latest_data", self.latest_data)
        # self.latest_data = self.encoder_decoder.serializedPb2MsgToDict(data_copy, self.msg_code)

        return deepcopy(self.latest_data)
