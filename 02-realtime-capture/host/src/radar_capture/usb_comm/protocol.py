"""Binary messages exchanged with the firmware over USB CDC."""

from collections.abc import Iterator
from dataclasses import dataclass
from enum import IntEnum
from struct import Struct

import numpy as np

from radar_capture.radar_phys.config import MAX_ADC_SAMPLES, RX_CHANNELS

MAGIC = b"RFFT"
VERSION = 1
HEADER = Struct("<4sBBHII")
_REPLY = Struct("<iBB")
_FFT = Struct("<IIHH")
_DONE = Struct("<iII")
MAX_PAYLOAD = _FFT.size + RX_CHANNELS * (MAX_ADC_SAMPLES // 2) * 8


class Message(IntEnum):
    STATUS = 1
    CONFIGURE = 2
    START = 3
    STOP = 4
    REPLY = 0x80
    RANGE_FFT = 0x90
    CAPTURE_DONE = 0x91


class State(IntEnum):
    IDLE = 1
    CAPTURING = 2


class ProtocolError(ValueError):
    """A device packet did not match the expected capture protocol."""


@dataclass(frozen=True)
class Packet:
    kind: Message
    request: int
    payload: bytes = b""

    def encode(self) -> bytes:
        return (
            HEADER.pack(
                MAGIC, VERSION, self.kind, 0, len(self.payload), self.request
            )
            + self.payload
        )


class PacketDecoder:
    """Reassemble messages from arbitrary USB read boundaries."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Iterator[Packet]:
        self._buffer.extend(data)
        offset = 0

        while len(self._buffer) - offset >= HEADER.size:
            start = self._buffer.find(MAGIC, offset)
            if start < 0:
                offset = len(self._buffer) - len(MAGIC) + 1
                break
            offset = start
            if len(self._buffer) - offset < HEADER.size:
                break

            _, version, kind, reserved, size, request = HEADER.unpack_from(
                self._buffer, offset
            )
            if version != VERSION:
                raise ProtocolError(f"Unsupported device protocol {version}.")
            if reserved or size > MAX_PAYLOAD or not request:
                raise ProtocolError("Invalid USB message header.")

            payload_start = offset + HEADER.size
            end = payload_start + size
            if end > len(self._buffer):
                break
            try:
                message = Message(kind)
            except ValueError as error:
                raise ProtocolError("Unknown USB message type.") from error
            yield Packet(
                message, request, bytes(self._buffer[payload_start:end])
            )
            offset = end

        del self._buffer[:offset]


@dataclass(frozen=True)
class Reply:
    code: int
    state: State
    radar_ready: bool
    configured: bool
    text: str

    @classmethod
    def decode(cls, payload: bytes) -> "Reply":
        if len(payload) < _REPLY.size:
            raise ProtocolError("Incomplete command reply.")
        code, state, flags = _REPLY.unpack_from(payload)
        try:
            device_state = State(state)
        except ValueError as error:
            raise ProtocolError("Invalid device state.") from error
        return cls(
            code,
            device_state,
            bool(flags & 1),
            bool(flags & 2),
            _message_text(payload, _REPLY.size),
        )


@dataclass(frozen=True)
class Spectrum:
    frame_index: int
    chirp_index: int
    values: np.ndarray

    @classmethod
    def decode(cls, payload: bytes) -> "Spectrum":
        if len(payload) < _FFT.size:
            raise ProtocolError("Incomplete Range FFT message.")
        frame, chirp, bins, channels = _FFT.unpack_from(payload)
        if not (1 <= bins <= MAX_ADC_SAMPLES // 2 and channels == RX_CHANNELS):
            raise ProtocolError("Invalid Range FFT dimensions.")
        if len(payload) != _FFT.size + bins * channels * 8:
            raise ProtocolError("Unexpected Range FFT payload length.")
        values = np.frombuffer(payload, dtype="<c8", offset=_FFT.size)
        return cls(frame, chirp, values.reshape(channels, bins))


@dataclass(frozen=True)
class Completion:
    code: int
    frames: int
    dropped: int
    text: str

    @classmethod
    def decode(cls, payload: bytes) -> "Completion":
        if len(payload) < _DONE.size:
            raise ProtocolError("Incomplete capture completion message.")
        code, frames, dropped = _DONE.unpack_from(payload)
        return cls(code, frames, dropped, _message_text(payload, _DONE.size))


def _message_text(payload: bytes, offset: int) -> str:
    return payload[offset:].decode("utf-8", errors="replace")
