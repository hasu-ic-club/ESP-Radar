"""Four live AWR2243 Range FFT plots."""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QGridLayout, QGroupBox

from radar_capture.radar_phys.config import RX_CHANNELS
from radar_capture.usb_comm.protocol import Spectrum

_TRACE_COLORS = ("#56dcc7", "#7eaaff", "#c7a1ff", "#ffbd76")
_AXIS_COLOR = "#a4b3c6"


class RangePlots(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Live Range FFT")
        self._range = None
        self._plots = []
        self._curves = []
        layout = QGridLayout(self)
        layout.setSpacing(10)

        for index in range(RX_CHANNELS):
            widget = pg.PlotWidget(background="#101824")
            widget.setMinimumSize(260, 175)
            plot = widget.getPlotItem()
            plot.setTitle(f"RX {index}", color="#e0ecfb", size="11pt")
            plot.setLabel("bottom", "Range", units="m", color=_AXIS_COLOR)
            plot.getAxis("bottom").enableAutoSIPrefix(False)
            plot.setLabel("left", "FFT magnitude (dB)", color=_AXIS_COLOR)
            plot.showGrid(x=True, y=True, alpha=0.18)
            for name in ("left", "bottom"):
                axis = plot.getAxis(name)
                axis.setPen("#455166")
                axis.setTextPen(_AXIS_COLOR)
            plot.setMouseEnabled(x=True, y=True)
            plot.setMenuEnabled(False)
            curve = plot.plot(pen=pg.mkPen(_TRACE_COLORS[index], width=1.5))
            self._plots.append(plot)
            self._curves.append(curve)
            layout.addWidget(widget, index // 2, index % 2)

    def prepare(self, range_axis: np.ndarray) -> None:
        self._range = range_axis
        for plot, curve in zip(self._plots, self._curves):
            curve.setData([], [])
            plot.enableAutoRange(y=True)
            if len(range_axis) > 1:
                plot.setXRange(0, float(range_axis[-1]), padding=0.01)

    def display(self, spectrum: Spectrum) -> None:
        """Plot ten times log10 of magnitude with a finite zero floor."""
        magnitude = np.abs(spectrum.values)
        db = 10.0 * np.log10(np.maximum(magnitude, 1e-12))
        for curve, values in zip(self._curves, db):
            curve.setData(self._range, values)
