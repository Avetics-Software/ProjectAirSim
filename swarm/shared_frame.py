"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Latest-frame handoff between processes through shared memory.

Only the client that loaded a scene receives its camera topics, so
px4_gimbal_cesium.py writes the spotter's POV frames here and
spotter_control.py reads the newest one.
"""

import struct
from multiprocessing import resource_tracker, shared_memory

import numpy as np

HEADER = struct.Struct("QII")  # frame counter, width, height
MAX_FRAME_BYTES = 1920 * 1080 * 3


class SharedFrameWriter:
    def __init__(self, name):
        self.shm = shared_memory.SharedMemory(
            name=name, create=True, size=HEADER.size + MAX_FRAME_BYTES
        )
        self.count = 0
        HEADER.pack_into(self.shm.buf, 0, 0, 0, 0)

    def write_image_msg(self, _topic, msg):
        """Subscription callback for a camera topic (BGR images)."""
        w, h = msg["width"], msg["height"]
        data = msg["data"]
        n = w * h * 3
        if n > MAX_FRAME_BYTES or len(data) < n:
            return
        self.shm.buf[HEADER.size:HEADER.size + n] = data[:n]
        self.count += 1
        HEADER.pack_into(self.shm.buf, 0, self.count, w, h)

    def close(self):
        self.shm.close()
        self.shm.unlink()


class SharedFrameReader:
    def __init__(self, name):
        self.shm = shared_memory.SharedMemory(name=name)
        # The writer owns the block; stop this process from unlinking it on exit
        resource_tracker.unregister(self.shm._name, "shared_memory")
        self.last_count = 0

    def read_new(self):
        """The newest BGR frame as an array, or None if nothing new arrived."""
        count, w, h = HEADER.unpack_from(self.shm.buf, 0)
        if count == self.last_count or not w:
            return None
        self.last_count = count
        n = w * h * 3
        return np.frombuffer(self.shm.buf, dtype=np.uint8, count=n,
                             offset=HEADER.size).reshape(h, w, 3).copy()

    def close(self):
        self.shm.close()
