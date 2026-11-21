"""Shared dark instrument palette for the desktop application."""

STYLESHEET = """
QWidget {
    background: #151d29;
    color: #e4edf7;
    font-size: 13px;
}
QGroupBox {
    border: 1px solid #334154;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 9px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 5px;
    color: #b9cee7;
}
QPushButton, QComboBox, QSpinBox, QDoubleSpinBox {
    border: 1px solid #3b4d64;
    border-radius: 4px;
    padding: 5px 8px;
    background: #202c3b;
}
QPushButton { min-height: 18px; }
QPushButton:hover { background: #30425a; }
QPushButton#primary {
    background: #14594f;
    border-color: #367f73;
    color: #edfffb;
    font-weight: 600;
}
QPushButton#primary:hover { background: #206e62; }
QWidget:disabled { color: #8491a4; }
QPushButton:disabled, QPushButton#primary:disabled {
    color: #8491a4;
    background: #202a37;
    border-color: #303d50;
}
QPlainTextEdit, QTableWidget {
    background: #101824;
    border: 1px solid #2f3e52;
    border-radius: 4px;
    selection-background-color: #284d66;
}
QPlainTextEdit { font-family: monospace; }
QHeaderView::section {
    background: #233245;
    color: #cee1f5;
    border: 0;
    padding: 6px;
}
QTabWidget::pane { border: 1px solid #334154; }
QTabBar::tab {
    padding: 7px 13px;
    background: #202b3a;
    border-bottom: 2px solid #202b3a;
}
QTabBar::tab:selected { border-bottom-color: #56dcc7; }
QStatusBar, QStatusBar QLabel {
    background: #101824;
    color: #cad9ec;
    padding: 3px;
}
QSplitter::handle { background: #151d29; }
"""
