"""Scene control panel: spawn/delete/reset simple object and obstacle models in the
gz sim world (PROJECT_SPEC.md ss4.2 step 9). Pure Qt widget code -- no `import rclpy`
here; spawning goes through RosInterfaceNode.spawn_object()/delete_object().
"""
import random

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QGridLayout, QPushButton, QListWidget

# Reasonable reach for the RV-2FR in front of its base, clear of the eye-to-hand
# camera mount at rv2fr_gz_bringup/urdf/rv2fr_gz.urdf.xacro's (0.9, 0, 0.9).
SPAWN_X_RANGE = (0.25, 0.5)
SPAWN_Y_RANGE = (-0.3, 0.3)
# half-height above the ground plane so each model rests on it, not embedded in it
_MODELS = {
    'Cube': ('rv2fr_cube', 0.02),
    'Cylinder': ('rv2fr_cylinder', 0.025),
    'Obstacle': ('rv2fr_obstacle', 0.1),
}


class SceneControlPanel(QGroupBox):
    def __init__(self, ros_interface_node, parent=None):
        super().__init__('Scene Control', parent)
        self._node = ros_interface_node
        self._spawned = []  # instance names, for delete-all/reset
        self._counters = {label: 0 for label in _MODELS}

        layout = QVBoxLayout(self)

        # Grid (2 columns) rather than one row -- a row of 3+ full-text buttons
        # doesn't fit in this panel's share of the window width without clipping,
        # confirmed by an actual rendered screenshot, not assumed.
        buttons_grid = QGridLayout()
        for i, label in enumerate(_MODELS):
            button = QPushButton(f'Spawn {label}')
            button.clicked.connect(lambda _, lbl=label: self._spawn(lbl))
            buttons_grid.addWidget(button, i // 2, i % 2)
        layout.addLayout(buttons_grid)

        self.object_list = QListWidget()
        layout.addWidget(self.object_list)

        reset_button = QPushButton('Reset Scene (delete all)')
        reset_button.clicked.connect(self._reset_scene)
        layout.addWidget(reset_button)

    def _spawn(self, label):
        model_name, z = _MODELS[label]
        self._counters[label] += 1
        instance_name = f'{model_name}_{self._counters[label]}'
        x = random.uniform(*SPAWN_X_RANGE)
        y = random.uniform(*SPAWN_Y_RANGE)
        self._node.spawn_object(model_name, instance_name, x, y, z)
        self._spawned.append(instance_name)
        self.object_list.addItem(f'{instance_name}  (x={x:.2f}, y={y:.2f})')

    def _reset_scene(self):
        for instance_name in self._spawned:
            self._node.delete_object(instance_name)
        self._spawned.clear()
        self.object_list.clear()
