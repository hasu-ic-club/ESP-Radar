"""Widgets for AWR2243 profile, chirp groups and frame settings."""

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from radar_capture.radar_phys.config import (
    SLOPE_LSB_MHZ_US,
    ChirpGroup,
    Frame,
    Profile,
    RadarConfig,
)

PROFILE_FIELDS = (
    ("start_ghz", "Start frequency (GHz)", 6, "76 to 81 GHz"),
    ("idle_us", "Idle time (µs)", 2, "0 to 5,242.87 µs"),
    ("adc_start_us", "ADC start time (µs)", 2, "0 to 40.95 µs"),
    ("ramp_end_us", "Ramp end time (µs)", 2, "0.01 to 5,000 µs"),
    (
        "slope_mhz_us",
        "Chirp slope (MHz/µs)",
        3,
        f"Nonzero, up to ±{5510 * SLOPE_LSB_MHZ_US:.3f} MHz/µs",
    ),
    ("tx_start_us", "TX start time (µs)", 2, "−40.96 to 40.95 µs"),
    ("adc_samples", "ADC samples", 0, "2 to 1,024 complex samples"),
    ("sample_rate_ksps", "Sample rate (kSps)", 0, "2,000 to 45,000 kSps"),
)
FRAME_FIELDS = (
    ("start", "Chirp start index", 0, "0 to 511"),
    ("end", "Chirp end index", 0, "0 to 511"),
    ("loops", "Loops per frame", 0, "1 to 255"),
    ("count", "Frame count", 0, "0 for continuous, or up to 65,535 frames"),
    ("period_ms", "Frame period (ms)", 5, "0.3 to 1,342 ms"),
)


class ConfigPanel(QGroupBox):
    changed = pyqtSignal()

    def __init__(self) -> None:
        super().__init__("Radar Configuration")
        self._tabs = QTabWidget()
        self._profile, profile_panel = self._form(Profile(), PROFILE_FIELDS)
        self._frame, frame_panel = self._form(Frame(), FRAME_FIELDS)
        frame_panel.layout().addRow(
            QLabel("Frame count 0 runs until stopped.")
        )

        self._chirps = QTableWidget(0, 3)
        self._chirps.setHorizontalHeaderLabels(
            ["Start index", "End index", "TX enable"]
        )
        self._chirps.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self._chirps.verticalHeader().setDefaultSectionSize(39)
        self._chirps.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self._chirps.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )

        chirp_panel = QWidget()
        chirp_layout = QVBoxLayout(chirp_panel)
        chirp_layout.addWidget(
            QLabel("TX0 and TX2 are enabled on this board.")
        )
        chirp_layout.addWidget(self._chirps)
        chirp_actions = QHBoxLayout()
        self._add = QPushButton("Add group")
        self._remove = QPushButton("Remove group")
        self._add.clicked.connect(self._add_chirp)
        self._remove.clicked.connect(self._remove_chirp)
        self._chirps.itemSelectionChanged.connect(self._sync_chirp_actions)
        chirp_actions.addWidget(self._add)
        chirp_actions.addWidget(self._remove)
        chirp_layout.addLayout(chirp_actions)
        self._append_chirp(ChirpGroup())

        self._tabs.addTab(profile_panel, "Profile 0")
        self._tabs.addTab(chirp_panel, "Chirps")
        self._tabs.addTab(frame_panel, "Frame")
        layout = QVBoxLayout(self)
        layout.addWidget(self._tabs, 1)

        self.apply_button = QPushButton("Apply Configuration")
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.apply_button.setObjectName("primary")
        self.start_button.setObjectName("primary")
        layout.addWidget(self.apply_button)
        capture_actions = QHBoxLayout()
        capture_actions.addWidget(self.start_button)
        capture_actions.addWidget(self.stop_button)
        layout.addLayout(capture_actions)

    def _form(self, defaults, fields):
        panel = QWidget()
        form = QFormLayout(panel)
        form.setVerticalSpacing(12)
        widgets = {}
        for name, label, decimals, help_text in fields:
            widget = _number_box(getattr(defaults, name), decimals)
            widget.setToolTip(help_text)
            widget.valueChanged.connect(self.changed)
            form.addRow(label, widget)
            widgets[name] = widget
        return widgets, panel

    def _append_chirp(self, chirp: ChirpGroup) -> None:
        row = self._chirps.rowCount()
        self._chirps.insertRow(row)
        for column, value in enumerate((chirp.start, chirp.end)):
            field = _number_box(value, 0)
            field.setAccessibleName(
                "Chirp start" if column == 0 else "Chirp end"
            )
            field.valueChanged.connect(self.changed)
            self._chirps.setCellWidget(row, column, field)

        mask = QComboBox()
        mask.setAccessibleName("TX enable")
        for text, value in (
            ("Off", 0),
            ("TX0", 1),
            ("TX2", 4),
            ("TX0 + TX2", 5),
        ):
            mask.addItem(text, value)
        mask.setCurrentIndex(mask.findData(chirp.tx_mask))
        mask.currentIndexChanged.connect(self.changed)
        self._chirps.setCellWidget(row, 2, mask)
        self._chirps.selectRow(row)
        self._sync_chirp_actions()

    def _add_chirp(self) -> None:
        last = max(chirp.end for chirp in self._chirp_values())
        index = min(max(last + 1, 0), 511)
        self._append_chirp(ChirpGroup(index, index))
        self.changed.emit()

    def _remove_chirp(self) -> None:
        row = self._chirps.currentRow()
        if row >= 0 and self._chirps.rowCount() > 1:
            self._chirps.removeRow(row)
            self._sync_chirp_actions()
            self.changed.emit()

    def _sync_chirp_actions(self) -> None:
        self._add.setEnabled(self._chirps.rowCount() < 512)
        self._remove.setEnabled(
            self._chirps.rowCount() > 1 and self._chirps.currentRow() >= 0
        )

    def _chirp_values(self) -> tuple[ChirpGroup, ...]:
        return tuple(
            ChirpGroup(
                self._chirps.cellWidget(row, 0).value(),
                self._chirps.cellWidget(row, 1).value(),
                self._chirps.cellWidget(row, 2).currentData(),
            )
            for row in range(self._chirps.rowCount())
        )

    def settings(self) -> RadarConfig:
        """Read the fields so validation can run before sending anything."""
        return RadarConfig(
            profile=Profile(**_values(self._profile)),
            chirps=self._chirp_values(),
            frame=Frame(**_values(self._frame)),
        )

    def set_editable(self, editable: bool) -> None:
        self._tabs.setEnabled(editable)


def _number_box(value, decimals: int) -> QSpinBox | QDoubleSpinBox:
    if decimals:
        widget = QDoubleSpinBox()
        widget.setDecimals(decimals)
        widget.setSingleStep(10 ** -min(2, decimals))
    else:
        widget = QSpinBox()
    # Keep invalid values editable so Apply can report the affected setting.
    widget.setRange(-1_000_000, 1_000_000)
    widget.setKeyboardTracking(False)
    widget.setValue(value)
    return widget


def _values(widgets) -> dict:
    return {name: widget.value() for name, widget in widgets.items()}
