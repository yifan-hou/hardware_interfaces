import sys
import os

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, "../../../../"))

import multiprocessing
import time
from multiprocessing import Process, Lock
from multiprocessing.managers import SharedMemoryManager

import numpy as np

from PyriteUtility.data_pipeline.shared_memory.shared_memory_ring_buffer import (
    SharedMemoryRingBuffer,
)
from PyriteUtility.data_pipeline.shared_memory.shared_memory_util import (
    ArraySpec,
)


class TestServer:
    def __init__(self, shm_manager: SharedMemoryManager) -> None:
        array_spec1 = ArraySpec(
            name="key1",
            shape=(),
            dtype=np.float64,
        )
        array_spec2 = ArraySpec(
            name="key2",
            shape=(),
            dtype=np.float64,
        )
        array_spec3 = ArraySpec(
            name="key3",
            shape=(),
            dtype=np.float64,
        )
        self.data = SharedMemoryRingBuffer(
            shm_manager=shm_manager,
            array_specs=[array_spec1, array_spec2, array_spec3],
            get_max_k=10,
            get_time_budget=0.1,
            put_desired_frequency=10,
        )

        process = Process(target=self.loop)
        process.start()

        self.process = process

    def loop(self, length=6):
        while True:
            print("looping. count: ", self.data.count)
            new_frame = {
                "key1": self.data.count,
                "key2": self.data.count,
                "key3": self.data.count,
            }
            self.data.put(new_frame)
            time.sleep(0.1)

    def get_data(self):
        return self.data.get_last_k(3)

    def join(self):
        self.process.join()


if __name__ == "__main__":
    print("starting main.")
    shm_manager = SharedMemoryManager()
    shm_manager.start()

    server = TestServer(shm_manager=shm_manager)
    time.sleep(2.0)
    print("starting loop.")

    for i in range(100):
        data = server.get_data()
        print("data key1: ", data["key1"])
        time.sleep(0.5)
    server.join()
