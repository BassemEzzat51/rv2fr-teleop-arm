import sys

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QScrollArea, QFrame, QGroupBox, QComboBox, QPushButton,
)
from PyQt6.QtCore import Qt

from rv2fr_teleop_gui.ros_interface_node import RosInterfaceNode
from rv2fr_teleop_gui.joint_jog_panel import JointJogPanel
from rv2fr_teleop_gui.camera_view_panel import CameraViewPanel
from rv2fr_teleop_gui.cartesian_jog_panel import CartesianJogPanel
from rv2fr_teleop_gui.scene_control_panel import SceneControlPanel
from rv2fr_teleop_gui.style import STYLESHEET, STATUS_OK, STATUS_ERROR, STATUS_NEUTRAL, refresh_style

# Matches rv2fr_perception/color_pose_estimator.py's KNOWN_OBJECTS names (the
# /rv2fr_perception/{name}_pose topic suffixes), not scene_control_panel's spawned
# instance names -- perception tracks by color class, not by spawned instance.
TRACKABLE_OBJECTS = ['cube', 'cylinder']


class ObjectTrackingPanel(QGroupBox):
    """Start/stop visual tracking of a perception-detected object (no gripper on this
    robot, so tracking means hovering/pointing near the object via moveit_servo's POSE
    command mode -- see RosInterfaceNode.start_object_tracking())."""

    def __init__(self, ros_interface_node, parent=None):
        super().__init__('Object Tracking', parent)
        self._node = ros_interface_node
        layout = QVBoxLayout(self)

        self.object_combo = QComboBox()
        self.object_combo.addItems(TRACKABLE_OBJECTS)
        layout.addWidget(self.object_combo)

        self.toggle_button = QPushButton('Start Tracking')
        self.toggle_button.setCheckable(True)
        self.toggle_button.clicked.connect(self._on_toggle)
        layout.addWidget(self.toggle_button)

        self.status_label = QLabel('Idle')
        self.status_label.setObjectName(STATUS_NEUTRAL)
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        self._node.object_tracking_status_changed.connect(self._on_status)

    def _on_toggle(self, checked: bool):
        if checked:
            self.object_combo.setEnabled(False)
            self.toggle_button.setText('Stop Tracking')
            self._node.start_object_tracking(self.object_combo.currentText())
        else:
            self.object_combo.setEnabled(True)
            self.toggle_button.setText('Start Tracking')
            self._node.stop_object_tracking()

    def _on_status(self, message: str, is_error: bool):
        self.status_label.setText(('ERROR: ' if is_error else '') + message)
        self.status_label.setObjectName(STATUS_ERROR if is_error else STATUS_OK)
        refresh_style(self.status_label)


def _hline():
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setFrameShadow(QFrame.Shadow.Sunken)
    line.setStyleSheet('color: #3c3d40;')
    return line


class MainWindow(QMainWindow):
    def __init__(self, ros_interface_node):
        super().__init__()
        self._node = ros_interface_node
        self.setWindowTitle('RV-2FR Teleop')
        self.setMinimumSize(1100, 650)

        central = QWidget()
        outer_layout = QVBoxLayout(central)
        outer_layout.setContentsMargins(14, 12, 14, 12)
        outer_layout.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel('RV-2FR Teleop Control')
        title.setObjectName('HeaderTitle')
        header.addWidget(title)
        header.addStretch(1)

        self.status_label = QLabel('Waiting for /joint_states...')
        self.status_label.setObjectName(STATUS_NEUTRAL)
        header.addWidget(self.status_label)
        self._node.connection_status_changed.connect(self._on_connection_status)

        header.addSpacing(18)

        self.jog_status_label = QLabel('Jog mode: joint')
        self.jog_status_label.setObjectName(STATUS_NEUTRAL)
        header.addWidget(self.jog_status_label)
        self._node.jog_status_changed.connect(self._on_jog_status)

        outer_layout.addLayout(header)
        outer_layout.addWidget(_hline())

        panels_layout = QHBoxLayout()
        panels_layout.setSpacing(12)

        # Joint Jog can have as many rows as the robot has actuated joints -- wrap it
        # (and the panels below it) in a scroll area so a robot with more joints, or a
        # shorter window, never squeezes the other panels instead of just scrolling.
        left_column_widget = QWidget()
        left_column = QVBoxLayout(left_column_widget)
        left_column.setSpacing(10)
        left_column.setContentsMargins(0, 0, 0, 0)
        left_column.addWidget(JointJogPanel(self._node))
        left_column.addStretch(1)

        left_scroll = QScrollArea()
        left_scroll.setWidget(left_column_widget)
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        panels_layout.addWidget(left_scroll, stretch=3)

        right_column_widget = QWidget()
        right_column = QVBoxLayout(right_column_widget)
        right_column.setSpacing(10)
        right_column.setContentsMargins(0, 0, 0, 0)
        right_column.addWidget(CartesianJogPanel(self._node))
        right_column.addWidget(ObjectTrackingPanel(self._node))
        right_column.addWidget(SceneControlPanel(self._node))
        right_column.addStretch(1)

        right_scroll = QScrollArea()
        right_scroll.setWidget(right_column_widget)
        right_scroll.setWidgetResizable(True)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        panels_layout.addWidget(right_scroll, stretch=2)

        panels_layout.addWidget(CameraViewPanel(self._node), stretch=2)
        outer_layout.addLayout(panels_layout)

        self.setCentralWidget(central)

    def _on_connection_status(self, connected: bool):
        self.status_label.setText('● Connected' if connected else '○ Waiting for /joint_states...')
        self.status_label.setObjectName(STATUS_OK if connected else STATUS_NEUTRAL)
        refresh_style(self.status_label)

    def _on_jog_status(self, message: str, is_error: bool):
        self.jog_status_label.setText(('ERROR: ' if is_error else '') + message)
        self.jog_status_label.setObjectName(STATUS_ERROR if is_error else STATUS_NEUTRAL)
        refresh_style(self.jog_status_label)

    def closeEvent(self, event):
        self._node.shutdown()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)

    node = RosInterfaceNode()
    node.start()

    window = MainWindow(node)
    window.resize(1400, 820)
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
