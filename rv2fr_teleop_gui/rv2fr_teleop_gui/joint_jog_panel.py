"""Joint jog panel: one slider + spinbox + live readback per actuated joint.

Pure Qt widget code -- no `import rclpy` here (PROJECT_SPEC.md ss7). All ROS
interaction goes through the RosInterfaceNode instance passed in, via its public
methods and signals.
"""
import math

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QDoubleSpinBox, QGroupBox
)
from PyQt6.QtCore import Qt, QTimer

SLIDER_STEPS_PER_DEGREE = 10  # slider resolution: 0.1 deg per step
# Coalesce window for slider-drag jog commands. Without this, QSlider.valueChanged
# fires on every pixel of drag movement, and each tick used to immediately publish a
# brand-new JointTrajectory with its own fresh 0.5s time-from-start -- dragging for a
# second could emit dozens of competing trajectories, which is what produced the
# jerky/high-latency motion the user reported. Coalescing to one send per ~30ms
# (well under send_joint_positions' 0.5s default duration, so motion still looks
# continuous) fixes that without changing the underlying command shape.
JOG_DEBOUNCE_MS = 30


def rad_to_deg(rad):
    return rad * 180.0 / math.pi


def deg_to_rad(deg):
    return deg * math.pi / 180.0


class JointRow(QWidget):
    def __init__(self, joint_spec, on_target_changed):
        super().__init__()
        self.name = joint_spec.name
        self._on_target_changed = on_target_changed

        lower_deg = rad_to_deg(joint_spec.lower)
        upper_deg = rad_to_deg(joint_spec.upper)

        layout = QHBoxLayout(self)
        layout.addWidget(QLabel(self.name), stretch=2)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(int(lower_deg * SLIDER_STEPS_PER_DEGREE))
        self.slider.setMaximum(int(upper_deg * SLIDER_STEPS_PER_DEGREE))
        self.slider.setValue(0 if lower_deg <= 0 <= upper_deg else int(lower_deg * SLIDER_STEPS_PER_DEGREE))
        layout.addWidget(self.slider, stretch=5)

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setSuffix(' deg')
        self.spinbox.setDecimals(1)
        self.spinbox.setRange(lower_deg, upper_deg)
        self.spinbox.setValue(self.slider.value() / SLIDER_STEPS_PER_DEGREE)
        layout.addWidget(self.spinbox, stretch=1)

        self.readback_label = QLabel('actual: -- deg')
        self.readback_label.setMinimumWidth(110)
        layout.addWidget(self.readback_label, stretch=1)

        self._suppress = False
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)

    def _on_slider_changed(self, value):
        if self._suppress:
            return
        deg = value / SLIDER_STEPS_PER_DEGREE
        self._suppress = True
        self.spinbox.setValue(deg)
        self._suppress = False
        self._on_target_changed(self.name, deg_to_rad(deg))

    def _on_spinbox_changed(self, deg):
        if self._suppress:
            return
        self._suppress = True
        self.slider.setValue(int(deg * SLIDER_STEPS_PER_DEGREE))
        self._suppress = False
        self._on_target_changed(self.name, deg_to_rad(deg))

    def set_readback(self, position_rad):
        self.readback_label.setText(f'actual: {rad_to_deg(position_rad):.1f} deg')


class JointJogPanel(QGroupBox):
    def __init__(self, ros_interface_node, parent=None):
        super().__init__('Joint Jog', parent)
        self._node = ros_interface_node
        self._targets = {}  # joint_name -> rad, seeded from readback once available

        # Throttles rapid-fire slider drag events to one send per JOG_DEBOUNCE_MS --
        # see the constant's comment above. Leading-edge: the first change in a burst
        # sends immediately (so a single click/tap still feels instant) and starts a
        # cooldown; any further changes during the cooldown just update _targets and
        # get flushed once the cooldown timer fires, so a continuous drag still moves
        # the arm smoothly instead of only sending once the drag stops.
        self._pending_send = False
        self._send_timer = QTimer(self)
        self._send_timer.setSingleShot(True)
        self._send_timer.setInterval(JOG_DEBOUNCE_MS)
        self._send_timer.timeout.connect(self._on_cooldown_elapsed)

        layout = QVBoxLayout(self)
        self._rows = {}
        for joint_spec in self._node.joints:
            row = JointRow(joint_spec, self._on_target_changed)
            self._rows[joint_spec.name] = row
            # Seed from the row's own actual initial value (it clamps to lower_deg
            # when the joint's range excludes 0 deg), not a hardcoded 0.0 -- a
            # hardcoded 0.0 here silently diverged from the slider's displayed
            # position for any joint whose limits exclude 0.
            self._targets[joint_spec.name] = deg_to_rad(row.slider.value() / SLIDER_STEPS_PER_DEGREE)
            layout.addWidget(row)

        self._node.joint_state_received.connect(self._on_joint_state)

    def _on_target_changed(self, joint_name, target_rad):
        self._targets[joint_name] = target_rad
        if self._send_timer.isActive():
            self._pending_send = True
        else:
            self._node.send_joint_positions(self._targets)
            self._send_timer.start()

    def _on_cooldown_elapsed(self):
        if self._pending_send:
            self._pending_send = False
            self._node.send_joint_positions(self._targets)
            self._send_timer.start()

    def _on_joint_state(self, positions: dict):
        for name, row in self._rows.items():
            if name in positions:
                row.set_readback(positions[name])
