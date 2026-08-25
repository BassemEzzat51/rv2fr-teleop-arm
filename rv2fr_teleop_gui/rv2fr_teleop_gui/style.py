"""One dark, flat stylesheet applied at the QApplication level (main_window.py's
main()), so every panel gets a consistent look without each widget file needing its
own styling code. Pure Qt/QSS -- no rclpy import here (PROJECT_SPEC.md ss7).

Status labels use objectName-based selectors (STATUS_OK/STATUS_ERROR/STATUS_NEUTRAL)
rather than inline setStyleSheet() calls, so color changes stay in one place. Qt's QSS
engine doesn't repaint automatically when a widget's objectName/dynamic property
changes after the widget is already shown -- call refresh_style(widget) after changing
one, or the old color sticks until something else forces a repaint.
"""
from PyQt6.QtWidgets import QWidget

ACCENT = '#7aa2f7'
OK = '#9ece6a'
ERROR = '#f7768e'
NEUTRAL = '#e0af68'

STATUS_OK = 'StatusOk'
STATUS_ERROR = 'StatusError'
STATUS_NEUTRAL = 'StatusNeutral'

STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: #1e1f22;
    color: #e6e6e6;
    font-family: "Segoe UI", "Ubuntu", "DejaVu Sans", sans-serif;
    font-size: 13px;
}}

QGroupBox {{
    background-color: #2a2b2e;
    border: 1px solid #3c3d40;
    border-radius: 8px;
    margin-top: 16px;
    padding: 12px 10px 10px 10px;
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    top: 2px;
    padding: 0 6px;
    color: {ACCENT};
}}

QLabel {{
    background: transparent;
}}

QPushButton {{
    background-color: #3b3d42;
    border: 1px solid #4a4c50;
    border-radius: 6px;
    padding: 6px 16px;
    color: #e6e6e6;
}}
QPushButton:hover {{
    background-color: #46484d;
    border-color: {ACCENT};
}}
QPushButton:pressed {{
    background-color: #2c2d30;
}}
QPushButton:disabled {{
    color: #6c6e72;
    background-color: #2c2d30;
    border-color: #3c3d40;
}}

QSlider::groove:horizontal {{
    height: 6px;
    background: #3b3d42;
    border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: {ACCENT};
    width: 16px;
    margin: -6px 0;
    border-radius: 8px;
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT};
    border-radius: 3px;
}}

QDoubleSpinBox, QListWidget, QLineEdit {{
    background-color: #232427;
    border: 1px solid #3c3d40;
    border-radius: 4px;
    padding: 3px 6px;
    color: #e6e6e6;
    selection-background-color: {ACCENT};
}}
QListWidget::item {{
    padding: 3px 2px;
}}
QListWidget::item:selected {{
    background-color: #3b3d42;
}}

QCheckBox {{
    spacing: 8px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid #4a4c50;
    border-radius: 3px;
    background: #232427;
}}
QCheckBox::indicator:checked {{
    background: {ACCENT};
    border-color: {ACCENT};
}}

QScrollArea {{
    border: none;
    background: transparent;
}}
QScrollArea > QWidget > QWidget {{
    background: transparent;
}}

QLabel#{STATUS_OK} {{
    color: {OK};
    font-weight: 600;
}}
QLabel#{STATUS_ERROR} {{
    color: {ERROR};
    font-weight: 600;
}}
QLabel#{STATUS_NEUTRAL} {{
    color: {NEUTRAL};
    font-weight: 600;
}}

QLabel#HeaderTitle {{
    font-size: 18px;
    font-weight: 700;
    color: #ffffff;
}}

QLabel#CameraFeed {{
    background-color: #0f1012;
    border: 1px solid #3c3d40;
    border-radius: 6px;
    color: #6c6e72;
}}
"""


def refresh_style(widget: QWidget):
    """Force Qt to re-evaluate this widget's QSS after its objectName or a dynamic
    property changed at runtime -- without this the old style (e.g. status color)
    stays applied until something unrelated forces a repaint."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)
