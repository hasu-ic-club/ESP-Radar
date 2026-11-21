"""Launch the Radar Capture desktop application."""

import sys

from PyQt6.QtCore import QLocale
from PyQt6.QtWidgets import QApplication

from radar_capture.ui.style import STYLESHEET
from radar_capture.ui.window import MainWindow


def main() -> int:
    QLocale.setDefault(QLocale(QLocale.Language.English))
    application = QApplication(sys.argv)
    application.setApplicationName("ESP Radar Capture")
    application.setStyle("Fusion")
    application.setStyleSheet(STYLESHEET)
    window = MainWindow()
    window.show()
    return application.exec()


if __name__ == "__main__":
    sys.exit(main())
