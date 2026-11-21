"""Communicate with ESP32 USB CDC outside the Qt GUI thread."""

import os
from dataclasses import dataclass, replace
from queue import Empty, SimpleQueue
from threading import Lock
from time import monotonic

from PyQt6.QtCore import QThread, pyqtSignal
from serial import Serial, SerialException

from radar_capture.radar_phys.config import RadarConfig
from radar_capture.usb_comm.protocol import (
    HEADER,
    Completion,
    Message,
    Packet,
    PacketDecoder,
    ProtocolError,
    Reply,
    Spectrum,
    State,
)
from radar_capture.usb_comm.recording import CaptureRecorder, Recording

READ_SIZE = 64 * 1024
DISCONNECTED = -10005
MISSING_DATA = -10007

COMMAND_LABELS = {
    Message.STATUS: "Read radar status",
    Message.CONFIGURE: "Apply radar configuration",
    Message.START: "Start capture",
    Message.STOP: "Stop capture",
}


@dataclass(frozen=True)
class Command:
    kind: Message
    payload: bytes = b""
    config: RadarConfig | None = None


@dataclass(frozen=True)
class Pending:
    command: Command
    deadline: float


class UsbWorker(QThread):
    control_log = pyqtSignal(str)
    replied = pyqtSignal(object, object)
    capture_finished = pyqtSignal(object, object)
    request_failed = pyqtSignal(object, str)
    transport_error = pyqtSignal(str)

    def __init__(self, port: str) -> None:
        super().__init__()
        self.port = port
        self._requests: SimpleQueue[Command] = SimpleQueue()
        self._pending: dict[int, Pending] = {}
        self._request_id = 0
        self._capture_id = None
        self._active_config = None
        self._recorder: CaptureRecorder | None = None
        self._seen_chirps = 0
        self._last_ordinal = -1
        self._lock = Lock()
        self._latest = None
        self._bytes_received = 0

    def submit(self, command: Command) -> None:
        self._requests.put(command)

    def snapshot(self) -> tuple[Spectrum | None, int]:
        """Return the latest spectrum without queuing a Qt event per chirp."""
        with self._lock:
            return self._latest, self._bytes_received

    def run(self) -> None:
        self._requests.put(Command(Message.STATUS))
        decoder = PacketDecoder()

        try:
            with Serial(
                self.port,
                baudrate=115200,
                timeout=0.01,
                write_timeout=0.8,
                exclusive=True if os.name == "posix" else None,
            ) as serial:
                serial.reset_input_buffer()
                self.control_log.emit(f"Opened USB CDC port {self.port}")
                while not self.isInterruptionRequested():
                    self._send_next(serial)
                    self._check_timeouts()
                    chunk = serial.read(min(serial.in_waiting or 1, READ_SIZE))
                    for packet in decoder.feed(chunk):
                        self._receive(packet)
        except (OSError, ProtocolError, RuntimeError) as error:
            if not self.isInterruptionRequested():
                self.transport_error.emit(str(error))
        finally:
            self._finish_on_disconnect()
            self.control_log.emit("USB CDC connection closed")

    def _send_next(self, serial: Serial) -> None:
        try:
            command = self._requests.get_nowait()
        except Empty:
            return

        if command.kind == Message.START and command.config.frame.count:
            try:
                self._recorder = CaptureRecorder(command.config)
            except (OSError, RuntimeError) as error:
                self.request_failed.emit(
                    command.kind, f"Could not create the recording: {error}"
                )
                return

        self._request_id = self._request_id % 0xFFFFFFFF + 1
        request = self._request_id
        data = Packet(command.kind, request, command.payload).encode()
        self.control_log.emit(f"Sent: {COMMAND_LABELS[command.kind]}")
        if serial.write(data) != len(data):
            raise SerialException("Incomplete USB control write.")
        timeout = 30.0 if command.kind == Message.CONFIGURE else 8.0
        self._pending[request] = Pending(command, monotonic() + timeout)

    def _check_timeouts(self) -> None:
        now = monotonic()
        for pending in self._pending.values():
            if pending.deadline < now:
                label = COMMAND_LABELS[pending.command.kind]
                raise TimeoutError(f"No reply to '{label}'.")

    def _receive(self, packet: Packet) -> None:
        if packet.kind == Message.REPLY:
            self._handle_reply(packet)
            return

        # A serial port can open while the firmware is disposing of an earlier
        # USB session. Only the acknowledged START creates a capture here.
        if not self._capture_id or packet.request != self._capture_id:
            return

        if packet.kind == Message.RANGE_FFT:
            self._handle_spectrum(packet)
        elif packet.kind == Message.CAPTURE_DONE:
            self._handle_completion(Completion.decode(packet.payload))
        else:
            raise ProtocolError("Unexpected device message.")

    def _handle_reply(self, packet: Packet) -> None:
        pending = self._pending.pop(packet.request, None)
        if pending is None:
            return

        command = pending.command
        reply = Reply.decode(packet.payload)
        outcome = "accepted" if reply.code == 0 else "reported an error"
        self.control_log.emit(
            f"Device {outcome}: {COMMAND_LABELS[command.kind]}"
        )
        if command.kind == Message.START:
            if reply.code == 0 and reply.state == State.CAPTURING:
                self._capture_id = packet.request
                self._active_config = command.config
                self._seen_chirps = 0
                self._last_ordinal = -1
                with self._lock:
                    self._latest = None
                    self._bytes_received = 0
            else:
                self._discard_unstarted_recording()
                if reply.code == 0:
                    raise ProtocolError("START reply did not enter capture.")
        self.replied.emit(command.kind, reply)

    def _handle_spectrum(self, packet: Packet) -> None:
        spectrum = Spectrum.decode(packet.payload)
        chirps = self._active_config.chirps_per_frame
        ordinal = spectrum.frame_index * chirps + spectrum.chirp_index
        if (
            spectrum.chirp_index >= chirps
            or spectrum.values.shape[1] != self._active_config.range_bins
            or ordinal <= self._last_ordinal
        ):
            raise ProtocolError("Range FFT does not match the active capture.")

        self._last_ordinal = ordinal
        self._seen_chirps += 1
        if self._recorder is not None:
            self._recorder.append(spectrum)
        with self._lock:
            self._latest = spectrum
            self._bytes_received += HEADER.size + len(packet.payload)

    def _handle_completion(self, completion: Completion) -> None:
        chirps = self._active_config.chirps_per_frame
        if completion.code == 0 and (
            self._seen_chirps != completion.frames * chirps
            or self._last_ordinal + 1 != self._seen_chirps
            or completion.dropped
        ):
            completion = replace(
                completion,
                code=MISSING_DATA,
                text="The USB stream is missing Range FFT packets.",
            )
        self.control_log.emit("Device sent capture completion")
        self._complete(completion)

    def _complete(self, completion: Completion) -> None:
        recording: Recording | None = None
        self._capture_id = None
        self._active_config = None

        if self._recorder is not None:
            recorder, self._recorder = self._recorder, None
            try:
                recording = recorder.finish(completion)
            except (OSError, RuntimeError) as error:
                completion = replace(
                    completion,
                    code=MISSING_DATA,
                    text=f"Could not finalize recording at {recorder.path}: "
                    f"{error}",
                )
        self.capture_finished.emit(completion, recording)

    def _finish_on_disconnect(self) -> None:
        if self._capture_id:
            frames = self._seen_chirps // self._active_config.chirps_per_frame
            self._complete(
                Completion(DISCONNECTED, frames, 0, "USB connection closed.")
            )
        else:
            self._discard_unstarted_recording()

    def _discard_unstarted_recording(self) -> None:
        if self._recorder is not None:
            recorder, self._recorder = self._recorder, None
            try:
                recorder.discard()
            except OSError as error:
                self.request_failed.emit(
                    Message.START, f"Could not remove {recorder.path}: {error}"
                )
