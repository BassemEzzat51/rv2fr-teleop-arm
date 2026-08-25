"""Live view of the fixed eye-to-hand camera. Pure Qt widget code -- no `import
rclpy` here (PROJECT_SPEC.md ss7); image bytes arrive via RosInterfaceNode's
camera_image_received signal.

This is a QoL/sanity-check feature, but it also previews exactly what a future VLA
would "see" through the fixed eye-to-hand camera -- worth getting the aspect ratio
and orientation right now rather than later.
"""
from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt

from rv2fr_teleop_gui.style import refresh_style

_QIMAGE_FORMAT_NAMES = {
    'rgb8': QImage.Format.Format_RGB888,
    'bgr8': QImage.Format.Format_BGR888,
    'mono8': QImage.Format.Format_Grayscale8,
}


class CameraViewPanel(QGroupBox):
    def __init__(self, ros_interface_node, parent=None):
        super().__init__('Eye-to-Hand Camera', parent)
        self._node = ros_interface_node

        layout = QVBoxLayout(self)
        self.image_label = QLabel('No image received yet')
        self.image_label.setObjectName('CameraFeed')
        refresh_style(self.image_label)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.image_label)

        self._node.camera_image_received.connect(self._on_camera_image)

    def _on_camera_image(self, data: bytes, width: int, height: int, bytes_per_line: int, encoding: str):
        qt_format = _QIMAGE_FORMAT_NAMES.get(encoding)
        if qt_format is None:
            return
        image = QImage(data, width, height, bytes_per_line, qt_format)
        pixmap = QPixmap.fromImage(image).scaled(
            self.image_label.width(), self.image_label.height(),
            Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(pixmap)
