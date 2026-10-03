"""Reader for the shared-memory frames written by gazebo/cam_bridge.py (seqlock, header <QdIII)."""

from __future__ import annotations

import mmap
import os
import struct

import numpy as np

HDR = struct.Struct("<QdIII")  # seq (odd while writing), sim stamp, width, height, channels


class ShmFrame:
    def __init__(self, path: str):
        self.path = path
        self._mm = None
        self.seq = 0
        self.stamp = float("nan")
        self.img: np.ndarray | None = None

    def _open(self) -> bool:
        if self._mm is None:
            if not os.path.exists(self.path):
                return False
            fd = os.open(self.path, os.O_RDONLY)
            self._mm = mmap.mmap(fd, 0, prot=mmap.PROT_READ)
        return True

    def reopen(self) -> None:
        """The bridge recreates the file on every run; call when a new run starts."""
        if self._mm is not None:
            self._mm.close()
        self._mm, self.seq = None, 0

    def read(self) -> bool:
        """Refresh self.img; returns True when a new frame arrived."""
        if not self._open():
            return False
        if self._mm.size() < HDR.size:
            return False
        for _ in range(5):
            seq, stamp, w, h, c = HDR.unpack_from(self._mm, 0)
            if seq % 2 or seq == 0:
                continue
            n = w * h * c
            if self._mm.size() < HDR.size + n:
                return False
            data = bytes(self._mm[HDR.size : HDR.size + n])
            if struct.unpack_from("<Q", self._mm, 0)[0] != seq:
                continue
            if seq == self.seq:
                return False
            shape = (h, w) if c == 1 else (h, w, c)
            self.img, self.seq, self.stamp = np.frombuffer(data, dtype=np.uint8).reshape(shape), seq, stamp
            return True
        return False
