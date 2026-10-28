"""Single-writer telemetry channel without per-update filesystem operations."""
import hashlib
import json
from multiprocessing import shared_memory
from pathlib import Path
import struct
from queue import Queue, Full
from threading import Thread
from time import perf_counter

CAPACITY = 16384
HEADER = 16

def channel_name(folder):
    return 'frontier_watch_' + hashlib.sha256(str(Path(folder).resolve()).encode()).hexdigest()[:24]

class WatchdogChannel:
    def __init__(self, folder, create=False):
        self.owner = create
        self.memory = shared_memory.SharedMemory(name=channel_name(folder), create=create,
                                                size=CAPACITY if create else 0)
        self.version = 0
        if create:
            self.memory.buf[:HEADER] = bytes(HEADER)

    def publish(self, value):
        payload = json.dumps(value, separators=(',', ':')).encode('utf-8')
        if len(payload) > CAPACITY-HEADER:
            raise ValueError('watchdog payload exceeds fixed capacity')
        self.version += 2
        struct.pack_into('<Q', self.memory.buf, 0, self.version-1)
        self.memory.buf[HEADER:HEADER+len(payload)] = payload
        struct.pack_into('<I', self.memory.buf, 8, len(payload))
        struct.pack_into('<Q', self.memory.buf, 0, self.version)

    def read(self):
        # Reject an in-progress or torn write; the parent retains its last sample.
        first = struct.unpack_from('<Q', self.memory.buf, 0)[0]
        if not first or first % 2:
            return None
        length = struct.unpack_from('<I', self.memory.buf, 8)[0]
        if not 0 < length <= CAPACITY-HEADER:
            return None
        payload = bytes(self.memory.buf[HEADER:HEADER+length])
        if first != struct.unpack_from('<Q', self.memory.buf, 0)[0]:
            return None
        return json.loads(payload)

    def close(self):
        self.memory.close()
        if self.owner:
            try:self.memory.unlink()
            except FileNotFoundError:pass

class AsyncEventWriter:
    """Keep ordered evidence; fail on overflow/error rather than discard records.

    Payloads must be immutable snapshots, as required by the runner contracts.
    Only the background thread calls the disk/console logger.
    """
    def __init__(self, callback):
        self.callback=callback
        self.queue=Queue(maxsize=131072)
        self.error=None
        self.total_s=0.
        self.max_s=0.
        self.thread=Thread(target=self._run,daemon=True)
        self.thread.start()

    def _run(self):
        while True:
            item=self.queue.get()
            if item is None:return
            started=perf_counter()
            try:self.callback(*item[0],**item[1])
            except BaseException as exc:self.error=exc
            elapsed=perf_counter()-started
            self.total_s+=elapsed
            self.max_s=max(self.max_s,elapsed)

    def submit(self,*args,**kwargs):
        if self.error is not None:raise RuntimeError('evidence writer failed') from self.error
        try:self.queue.put_nowait((args,kwargs))
        except Full:raise RuntimeError('evidence writer backlog exceeded capacity')

    def close(self):
        self.queue.put(None)
        self.thread.join()
        if self.error is not None:raise RuntimeError('evidence writer failed') from self.error
