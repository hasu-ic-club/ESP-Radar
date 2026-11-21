"""Stream finite-frame acquisitions into an HDF5 file."""

import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile

import h5py
import numpy as np

from radar_capture.radar_phys.config import RX_CHANNELS, RadarConfig
from radar_capture.usb_comm.protocol import Completion, Spectrum

CHUNK_SIZE = 32


@dataclass(frozen=True)
class Recording:
    path: Path
    chirps: int

    def save_as(self, destination: Path) -> None:
        """Publish a completed copy in the destination directory."""
        with NamedTemporaryFile(
            prefix=".radar-capture-", dir=destination.parent, delete=False
        ) as temporary:
            staged = Path(temporary.name)

        try:
            shutil.copyfile(self.path, staged)
            os.replace(staged, destination)
        finally:
            staged.unlink(missing_ok=True)


class CaptureRecorder:
    """Append bounded batches to disk while USB acquisition is running."""

    def __init__(self, config: RadarConfig) -> None:
        with NamedTemporaryFile(
            prefix="radar-capture-", suffix=".h5", delete=False
        ) as temporary:
            self.path = Path(temporary.name)

        self._file = None
        try:
            self._initialize(config)
        except Exception:
            self.discard()
            raise

    def _initialize(self, config: RadarConfig) -> None:
        self._file = h5py.File(self.path, "w")
        shape = (RX_CHANNELS, config.range_bins)
        self._file.attrs["format_version"] = 1
        self._file.attrs["radar_config_json"] = json.dumps(config.metadata())
        self._file.create_dataset("range_m", data=config.range_axis())
        self._indices = self._file.create_dataset(
            "indices",
            (0, 2),
            maxshape=(None, 2),
            chunks=(CHUNK_SIZE, 2),
            dtype="<u4",
        )
        self._indices.attrs["columns"] = "frame_index, chirp_index"
        self._values = self._file.create_dataset(
            "range_fft",
            (0, *shape),
            maxshape=(None, *shape),
            chunks=(CHUNK_SIZE, *shape),
            dtype="<c8",
        )
        self._index_buffer = np.empty((CHUNK_SIZE, 2), dtype=np.uint32)
        self._value_buffer = np.empty((CHUNK_SIZE, *shape), dtype=np.complex64)
        self._buffered = 0
        self._written = 0

    def append(self, spectrum: Spectrum) -> None:
        self._index_buffer[self._buffered] = (
            spectrum.frame_index,
            spectrum.chirp_index,
        )
        self._value_buffer[self._buffered] = spectrum.values
        self._buffered += 1
        if self._buffered == CHUNK_SIZE:
            self._flush()

    def _flush(self) -> None:
        if not self._buffered:
            return
        end = self._written + self._buffered
        self._values.resize(end, axis=0)
        self._indices.resize(end, axis=0)
        self._values[self._written : end] = self._value_buffer[
            : self._buffered
        ]
        self._indices[self._written : end] = self._index_buffer[
            : self._buffered
        ]
        self._written = end
        self._buffered = 0

    def finish(self, completion: Completion) -> Recording | None:
        try:
            self._flush()
            self._file.attrs["device_frames"] = completion.frames
            self._file.attrs["dropped_frames"] = completion.dropped
            self._file.attrs["completion_code"] = completion.code
            self._file.attrs["completion_message"] = completion.text
        finally:
            self._file.close()
            self._file = None

        if self._written == 0:
            self.path.unlink(missing_ok=True)
            return None
        return Recording(self.path, self._written)

    def discard(self) -> None:
        try:
            if self._file is not None:
                self._file.close()
                self._file = None
        finally:
            self.path.unlink(missing_ok=True)
