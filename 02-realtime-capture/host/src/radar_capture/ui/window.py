"""Main window, device state and recording actions."""

from datetime import datetime
from pathlib import Path
from time import monotonic

from PyQt6.QtCore import QStandardPaths, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from radar_capture.radar_phys.config import ConfigError
from radar_capture.ui.config_panel import ConfigPanel
from radar_capture.ui.plots import RangePlots
from radar_capture.usb_comm.protocol import Completion, Message, Reply, State
from radar_capture.usb_comm.recording import Recording
from radar_capture.usb_comm.worker import (
    COMMAND_LABELS,
    DISCONNECTED,
    Command,
    UsbWorker,
)


class SaveWorker(QThread):
    saved = pyqtSignal(object, object)
    failed = pyqtSignal(str)

    def __init__(self, recording: Recording, destination: Path) -> None:
        super().__init__()
        self.recording = recording
        self.destination = destination

    def run(self) -> None:
        try:
            self.recording.save_as(self.destination)
        except OSError as error:
            self.failed.emit(str(error))
        else:
            self.saved.emit(self.recording, self.destination)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP Radar Capture")
        self.resize(1230, 830)
        self._worker: UsbWorker | None = None
        self._save_worker: SaveWorker | None = None
        self._operation: Message | None = None
        self._applied = None
        self._candidate = None
        self._state = None
        self._radar_ready = False
        self._disconnecting = False
        self._closing = False
        self._recordings: list[Recording] = []
        self._last_spectrum = None
        self._meter_time = monotonic()
        self._meter_bytes = 0

        self._config = ConfigPanel()
        self._config.changed.connect(self._settings_changed)
        self._config.apply_button.clicked.connect(self._apply)
        self._config.start_button.clicked.connect(self._start)
        self._config.stop_button.clicked.connect(self._stop)

        self._plots = RangePlots()
        self._port = QComboBox()
        self._port.setPlaceholderText("No USB CDC ports found")
        self._refresh = QPushButton("Refresh")
        self._connection = QPushButton("Connect")
        self._firmware = QLabel("—")
        self._radar = QLabel("Disconnected")
        self._save = QPushButton("Save Recording…")
        self._refresh.clicked.connect(self._refresh_ports)
        self._connection.clicked.connect(self._toggle_connection)
        self._save.clicked.connect(self._choose_recording)
        self._status = QLabel("Disconnected")
        self._throughput = QLabel()
        self.statusBar().addWidget(self._status, 1)
        self.statusBar().addPermanentWidget(self._throughput)

        self._build_layout()
        self._refresh_ports()
        self._update_controls()
        self._display_timer = QTimer(self)
        self._display_timer.timeout.connect(self._update_display)
        self._display_timer.start(33)

    def _build_layout(self) -> None:
        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(12)
        self.setCentralWidget(central)

        status = QGroupBox("Radar Status")
        status_layout = QGridLayout(status)
        status_layout.setColumnStretch(1, 2)
        status_layout.setColumnStretch(5, 3)
        status_layout.addWidget(QLabel("USB CDC port"), 0, 0)
        status_layout.addWidget(self._port, 0, 1)
        status_layout.addWidget(self._refresh, 0, 2)
        status_layout.addWidget(self._connection, 0, 3)
        status_layout.addWidget(QLabel("Radar"), 0, 4)
        status_layout.addWidget(self._radar, 0, 5)
        status_layout.addWidget(QLabel("Firmware"), 1, 4)
        status_layout.addWidget(self._firmware, 1, 5)
        layout.addWidget(status)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._config)
        splitter.addWidget(self._plots)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([355, 845])
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)

        log_panel = QGroupBox("Control Log")
        log_layout = QVBoxLayout(log_panel)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)
        self._log.setFixedHeight(100)
        log_layout.addWidget(self._log)
        actions = QHBoxLayout()
        actions.addStretch()
        actions.addWidget(self._save)
        log_layout.addLayout(actions)
        layout.addWidget(log_panel)

    def _refresh_ports(self) -> None:
        previous = self._port.currentData()
        self._port.clear()
        recommended = -1
        for port in sorted(list_ports.comports()):
            label = f"{port.device} — {port.description}"
            self._port.addItem(label, port.device)
            if "ESP Radar Capture" in (port.product or port.description):
                recommended = self._port.count() - 1
        index = self._port.findData(previous) if previous else -1
        if index >= 0 or recommended >= 0:
            self._port.setCurrentIndex(index if index >= 0 else recommended)
        self._update_controls()

    def _toggle_connection(self) -> None:
        if self._worker is not None:
            self._disconnecting = True
            self._worker.requestInterruption()
        else:
            port = self._port.currentData()
            if not port:
                return
            worker = UsbWorker(port)
            self._worker = worker
            self._operation = Message.STATUS
            worker.control_log.connect(self._log_control)
            worker.replied.connect(self._handle_reply)
            worker.capture_finished.connect(self._capture_finished)
            worker.request_failed.connect(self._request_failed)
            worker.transport_error.connect(self._transport_error)
            worker.finished.connect(self._disconnected)
            worker.start()
        self._update_controls()

    def _settings_changed(self) -> None:
        self._update_controls()

    def _apply(self) -> None:
        config = self._config.settings()
        try:
            payload = config.encode()
        except ConfigError as error:
            self._dialog(
                "Invalid Radar Configuration", str(error), warning=True
            )
            return

        self._candidate = config
        self._operation = Message.CONFIGURE
        self._worker.submit(Command(Message.CONFIGURE, payload))
        self._update_controls()

    def _start(self) -> None:
        self._operation = Message.START
        self._plots.prepare(self._applied.range_axis())
        self._last_spectrum = None
        self._meter_time = monotonic()
        self._meter_bytes = 0
        self._throughput.clear()
        self._worker.submit(Command(Message.START, config=self._applied))
        self._update_controls()

    def _stop(self) -> None:
        self._operation = Message.STOP
        self._worker.submit(Command(Message.STOP))
        self._update_controls()

    def _handle_reply(self, kind: Message, reply: Reply) -> None:
        self._state = reply.state
        self._radar_ready = reply.radar_ready
        if kind == Message.STATUS:
            self._firmware.setText(reply.text)
        if kind == Message.CONFIGURE:
            self._applied = self._candidate if reply.code == 0 else None
            self._candidate = None
            if reply.code == 0:
                self._plots.prepare(self._applied.range_axis())
        if not reply.configured:
            self._applied = None

        if reply.code != 0:
            self._dialog(
                COMMAND_LABELS[kind],
                f"{reply.text}\n\nDevice code: {reply.code}",
            )
        if not (
            kind == Message.STOP
            and reply.code == 0
            and reply.state == State.CAPTURING
        ):
            self._operation = None
        self._update_controls()

    def _capture_finished(
        self, completion: Completion, recording: Recording | None
    ) -> None:
        self._state = State.IDLE
        self._operation = None
        self._update_display()
        self._throughput.clear()

        if recording is not None:
            self._recordings.append(recording)
        self._update_controls()
        if self._closing:
            return

        failed = completion.code != 0 and not (
            self._disconnecting and completion.code == DISCONNECTED
        )
        if recording is not None:
            text = (
                f"{completion.text}\n\n"
                f"{recording.chirps:,} chirps are available to save."
            )
            if completion.dropped:
                text += f" Dropped frames: {completion.dropped:,}."
            if failed:
                text += f" Device code: {completion.code}."
            self._offer_save(recording, text, failed)
        elif failed:
            self._dialog(
                "Capture Error",
                f"{completion.text}\n\nCode: {completion.code}",
            )
        else:
            self.statusBar().showMessage(completion.text, 5000)

        if (
            completion.code not in (0, DISCONNECTED)
            and not self._disconnecting
        ):
            self._operation = Message.STATUS
            self._worker.submit(Command(Message.STATUS))
            self._update_controls()

    def _request_failed(self, kind: Message, error: str) -> None:
        self._operation = None
        self._dialog(COMMAND_LABELS[kind], error)
        self._update_controls()

    def _transport_error(self, error: str) -> None:
        self._dialog("USB Communication Error", error)

    def _disconnected(self) -> None:
        worker, self._worker = self._worker, None
        worker.deleteLater()
        self._state = None
        self._operation = None
        self._applied = None
        self._candidate = None
        self._radar_ready = False
        self._disconnecting = False
        self._firmware.setText("—")
        self._throughput.clear()
        self._refresh_ports()
        if self._closing:
            self.close()

    def _update_controls(self) -> None:
        attached = self._worker is not None
        capturing = self._state == State.CAPTURING
        idle = attached and self._state == State.IDLE
        available = (
            idle and self._operation is None and not self._disconnecting
        )
        self._connection.setText("Disconnect" if attached else "Connect")
        self._connection.setEnabled(
            not self._disconnecting and (attached or self._port.count() > 0)
        )
        self._refresh.setEnabled(not attached)
        self._port.setEnabled(not attached)
        self._config.set_editable(
            not capturing
            and self._operation is None
            and not self._disconnecting
        )
        self._config.apply_button.setEnabled(available)
        self._config.start_button.setEnabled(
            available
            and self._radar_ready
            and self._applied is not None
            and self._config.settings() == self._applied
        )
        self._config.stop_button.setEnabled(
            capturing and self._operation is None and not self._disconnecting
        )
        self._save.setEnabled(
            bool(self._recordings) and self._save_worker is None
        )

        if self._disconnecting:
            status = "Disconnecting"
        elif not attached:
            status = "Disconnected"
        elif self._state is None:
            status = "Connecting"
        elif self._operation == Message.CONFIGURE:
            status = "Configuring"
        elif self._operation == Message.STOP:
            status = "Stopping"
        elif self._operation == Message.START:
            status = "Starting"
        elif capturing:
            status = "Capturing"
        else:
            status = (
                "Idle — configuration applied"
                if self._applied == self._config.settings()
                else "Idle — apply configuration to start"
            )
        self._status.setText(status)
        if self._radar_ready:
            self._radar.setText("Ready")
        elif self._state is None:
            self._radar.setText(status)
        else:
            self._radar.setText("Unavailable")

    def _update_display(self) -> None:
        if self._worker is None or self._operation == Message.START:
            return
        spectrum, received = self._worker.snapshot()
        if spectrum is not None and spectrum is not self._last_spectrum:
            self._plots.display(spectrum)
            self._last_spectrum = spectrum

        now = monotonic()
        elapsed = now - self._meter_time
        if self._state == State.CAPTURING and elapsed >= 0.25:
            rate = 8 * (received - self._meter_bytes) / (1e6 * elapsed)
            self._throughput.setText(f"{max(0.0, rate):.2f} Mbps")
            self._meter_time = now
            self._meter_bytes = received

    def _log_control(self, message: str) -> None:
        self._log.appendPlainText(f"{datetime.now():%H:%M:%S}  {message}")

    def _dialog(self, title: str, text: str, warning: bool = False) -> None:
        if self._closing:
            return
        icon = (
            QMessageBox.Icon.Warning if warning else QMessageBox.Icon.Critical
        )
        dialog = QMessageBox(icon, title, text, parent=self)
        dialog.setTextFormat(Qt.TextFormat.PlainText)
        dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        dialog.open()

    def _offer_save(
        self, recording: Recording, text: str, failed: bool
    ) -> None:
        icon = (
            QMessageBox.Icon.Warning
            if failed
            else QMessageBox.Icon.Information
        )
        title = "Partial Recording" if failed else "Recording Ready"
        dialog = QMessageBox(icon, title, text, parent=self)
        dialog.setTextFormat(Qt.TextFormat.PlainText)
        save = dialog.addButton("Save As…", QMessageBox.ButtonRole.AcceptRole)
        dialog.addButton("Later", QMessageBox.ButtonRole.RejectRole)
        dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        dialog.buttonClicked.connect(
            lambda clicked: (
                self._save_as(recording) if clicked is save else None
            )
        )
        dialog.open()

    def _choose_recording(self) -> None:
        if len(self._recordings) == 1:
            self._save_as(self._recordings[0])
            return

        labels = [
            f"Recording {index} — {recording.chirps:,} chirps"
            for index, recording in enumerate(self._recordings, 1)
        ]
        selected, accepted = QInputDialog.getItem(
            self, "Save Recording", "Recording", labels, len(labels) - 1, False
        )
        if accepted:
            self._save_as(self._recordings[labels.index(selected)])

    def _save_as(self, recording: Recording) -> None:
        if self._save_worker is not None or recording not in self._recordings:
            return

        documents = QStandardPaths.writableLocation(
            QStandardPaths.StandardLocation.DocumentsLocation
        )
        default = Path(documents or Path.home()) / (
            f"radar-capture-{datetime.now():%Y%m%d-%H%M%S}.h5"
        )
        name, _ = QFileDialog.getSaveFileName(
            self,
            "Save Radar Recording",
            str(default),
            "HDF5 (*.h5)",
            options=QFileDialog.Option.DontUseNativeDialog,
        )
        if not name:
            return
        destination = Path(name)
        if not destination.suffix:
            destination = destination.with_suffix(".h5")
        if destination.resolve() == recording.path.resolve():
            self._dialog(
                "Save Recording", "Choose a location for the permanent copy."
            )
            return

        worker = SaveWorker(recording, destination)
        self._save_worker = worker
        worker.saved.connect(self._saved)
        worker.failed.connect(lambda error: self._dialog("Save Error", error))
        worker.finished.connect(self._saving_finished)
        worker.start()
        self._update_controls()

    def _saved(self, recording: Recording, destination: Path) -> None:
        self._recordings.remove(recording)
        recording.path.unlink(missing_ok=True)
        self.statusBar().showMessage(f"Saved {destination}", 7000)

    def _saving_finished(self) -> None:
        worker, self._save_worker = self._save_worker, None
        worker.deleteLater()
        self._update_controls()

    def closeEvent(self, event) -> None:
        if self._save_worker is not None:
            self.statusBar().showMessage("Saving recording…", 3000)
            event.ignore()
            return

        if self._worker is not None:
            if not self._closing:
                active = (
                    self._state == State.CAPTURING
                    or self._operation == Message.START
                )
                if self._recordings or active:
                    if not self._confirm_exit():
                        event.ignore()
                        return
                self._closing = True
                self._toggle_connection()
            event.ignore()
            return

        if self._recordings and not self._closing and not self._confirm_exit():
            event.ignore()
            return
        for recording in self._recordings:
            recording.path.unlink(missing_ok=True)
        event.accept()

    def _confirm_exit(self) -> bool:
        return (
            QMessageBox.question(
                self,
                "Exit Radar Capture",
                "Exit and discard any active or unsaved recordings?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            == QMessageBox.StandardButton.Yes
        )
