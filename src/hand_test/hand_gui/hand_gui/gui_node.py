#!/usr/bin/env python3
"""
Omnihand GUI Controller — 用滑动条控制灵巧手 10 个关节角度

ros2 run hand_gui hand_gui
"""

import sys
import math
import threading

import rclpy
from rclpy.node import Node
from omnihand_node_msgs.msg import MotorAngle, ControlMode

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer

# 关节定义: (名称, 最小角度, 最大角度, 默认角度)
JOINTS = [
    ("1 thumb_roll",  -50,  10,  -30.0),
    ("2 thumb_abad",    0, 100,    1.0),
    ("3 thumb_mcp",   -49,   0,    0.0),
    ("4 index_abad",    0,  12,    4.9),
    ("5 index_pip",     0,  90,    0.0),
    ("6 middle_pip",    0,  90,    0.0),
    ("7 ring_abad",   -10,   0,   -4.9),
    ("8 ring_pip",      0,  90,    0.0),
    ("9 pinky_abad",  -10,   0,   -5.4),
    ("10 pinky_pip",    0,  90,    0.0),
]

# 预设动作
PRESETS = {
    "张开 (Home)": [-30.0, 1.0, 0.0, 4.9, 0.0, 0.0, -4.9, 0.0, -5.4, 0.0],
    "抓握 (Grasp)": [-18.44, 52.60, -28.96, 10.0, 30.0, 30.0, -5.0, 30.0, -5.0, 30.0],
    "全握 (Full)": [-50.0, 100.0, -49.0, 12.0, 90.0, 90.0, -10.0, 90.0, -10.0, 90.0],
}


class HandGuiNode(Node):
    def __init__(self):
        super().__init__('hand_gui_node')
        self.pub_angle = self.create_publisher(
            MotorAngle, '/agihand/omnihand/left/motor_angle_cmd', 10)
        self.pub_mode = self.create_publisher(
            ControlMode, '/agihand/omnihand/left/control_mode_cmd', 10)

        # 发送一次控制模式：全部位置控制
        mode_msg = ControlMode()
        mode_msg.modes = [0] * 10
        self.pub_mode.publish(mode_msg)

    def publish_angles(self, angles_deg):
        msg = MotorAngle()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'angle_frame'
        msg.angles = [deg * math.pi / 180.0 for deg in angles_deg]
        self.pub_angle.publish(msg)

        # 同时发送控制模式保持位置控制
        mode_msg = ControlMode()
        mode_msg.modes = [0] * 10
        self.pub_mode.publish(mode_msg)


class HandGuiWindow(QWidget):
    def __init__(self, ros_node: HandGuiNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Omnihand 关节控制器')
        self.setMinimumWidth(600)

        self.sliders = []
        self.labels = []
        self.angles_deg = [j[3] for j in JOINTS]  # 默认角度

        self._build_ui()

        # 定时发布 (20Hz)
        self.pub_timer = QTimer(self)
        self.pub_timer.timeout.connect(self._publish)
        self.pub_timer.start(50)

        # 定时 spin ROS (不阻塞 GUI)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(10)

    def _build_ui(self):
        main_layout = QVBoxLayout()

        # ---- 关节滑动条 ----
        joint_group = QGroupBox("关节角度 (度)")
        grid = QGridLayout()

        for i, (name, min_deg, max_deg, default) in enumerate(JOINTS):
            label_name = QLabel(name)
            label_name.setFixedWidth(120)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(min_deg * 10))
            slider.setMaximum(int(max_deg * 10))
            slider.setValue(int(default * 10))
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval((max_deg - min_deg) * 10 // 5)

            value_label = QLabel(f"{default:.1f}°")
            value_label.setFixedWidth(60)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            slider.valueChanged.connect(
                lambda val, idx=i, lbl=value_label: self._on_slider_changed(idx, val, lbl))

            grid.addWidget(label_name, i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(value_label, i, 2)

            self.sliders.append(slider)
            self.labels.append(value_label)

        joint_group.setLayout(grid)
        main_layout.addWidget(joint_group)

        # ---- 预设动作按钮 ----
        preset_group = QGroupBox("预设动作")
        preset_layout = QHBoxLayout()
        for preset_name, preset_angles in PRESETS.items():
            btn = QPushButton(preset_name)
            btn.clicked.connect(
                lambda checked, angles=preset_angles: self._apply_preset(angles))
            preset_layout.addWidget(btn)
        preset_group.setLayout(preset_layout)
        main_layout.addWidget(preset_group)

        # ---- 状态栏 ----
        self.status_label = QLabel("就绪")
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)

    def _on_slider_changed(self, idx, raw_value, label):
        deg = raw_value / 10.0
        self.angles_deg[idx] = deg
        label.setText(f"{deg:.1f}°")

    def _apply_preset(self, angles):
        for i, deg in enumerate(angles):
            self.sliders[i].setValue(int(deg * 10))
            self.angles_deg[i] = deg

    def _publish(self):
        self.ros_node.publish_angles(self.angles_deg)
        self.status_label.setText(
            "发布中: [" + ", ".join(f"{a:.1f}" for a in self.angles_deg) + "]")

    def _spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    ros_node = HandGuiNode()

    app = QApplication(sys.argv)
    window = HandGuiWindow(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
