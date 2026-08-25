"""Cartesian jog panel: X/Y/Z/Roll/Pitch/Yaw increment buttons, driving the TCP via
RosInterfaceNode's own /compute_ik-based Cartesian control (PROJECT_SPEC.md ss4.2 step
6) -- NOT moveit_servo, whose own Cartesian/TCP control was found to be permanently
non-functional on this install due to a confirmed-open upstream MoveIt2 bug (see
ros_interface_node.py's module-level comment and docs/decisions.md). Pure Qt widget
code -- no `import rclpy` here; all ROS interaction goes through the RosInterfaceNode
instance passed in.

The Enable checkbox is a local "armed" gate only (no controller switch happens anymore
-- both joint and Cartesian control run through rv2fr_controller), kept so a jog click
still requires a deliberate two-step action rather than being one accidental click away.
"""
from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QCheckBox
from PyQt6.QtCore import QTimer

# unitless [-1, 1] magnitude per jog, scaled by ros_interface_node.py's
# CARTESIAN_LINEAR_SCALE/CARTESIAN_ANGULAR_SCALE.
JOG_MAGNITUDE = 0.5
# Each send_cartesian_twist() call only advances the robot one IK-solve tick -- a click
# needs to resend for a short burst to produce a visible increment, not a single message.
JOG_BURST_MS = 250
JOG_BURST_INTERVAL_MS = 20

_AXES = [
    ('X', 'linear', 0),
    ('Y', 'linear', 1),
    ('Z', 'linear', 2),
    ('Roll', 'angular', 0),
    ('Pitch', 'angular', 1),
    ('Yaw', 'angular', 2),
]


class CartesianJogPanel(QGroupBox):
    def __init__(self, ros_interface_node, parent=None):
        super().__init__('Cartesian Jog (TCP, via IK)', parent)
        self._node = ros_interface_node

        layout = QVBoxLayout(self)

        self.enable_checkbox = QCheckBox('Enable Cartesian jog')
        layout.addWidget(self.enable_checkbox)

        for label, component, index in _AXES:
            row = QHBoxLayout()
            row.addWidget(QLabel(label), stretch=1)

            minus_button = QPushButton('-')
            minus_button.setFixedWidth(40)
            minus_button.clicked.connect(lambda _, c=component, i=index: self._jog(c, i, -JOG_MAGNITUDE))
            row.addWidget(minus_button)

            plus_button = QPushButton('+')
            plus_button.setFixedWidth(40)
            plus_button.clicked.connect(lambda _, c=component, i=index: self._jog(c, i, JOG_MAGNITUDE))
            row.addWidget(plus_button)
            row.addStretch(1)

            layout.addLayout(row)

    def _jog(self, component, index, value):
        if not self.enable_checkbox.isChecked():
            return
        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
        (linear if component == 'linear' else angular)[index] = value
        linear, angular = tuple(linear), tuple(angular)

        ticks_remaining = JOG_BURST_MS // JOG_BURST_INTERVAL_MS
        timer = QTimer(self)
        timer.setInterval(JOG_BURST_INTERVAL_MS)

        def send_tick():
            nonlocal ticks_remaining
            self._node.send_cartesian_twist(linear=linear, angular=angular)
            ticks_remaining -= 1
            if ticks_remaining <= 0:
                timer.stop()
                timer.deleteLater()

        timer.timeout.connect(send_tick)
        timer.start()
        send_tick()
