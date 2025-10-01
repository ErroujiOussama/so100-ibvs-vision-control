import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading

JOINT_NAMES = ['1', '2', '3', '4', '5']
NUM_JOINTS = len(JOINT_NAMES)

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_slider_gui')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.last_positions = [0.0] * NUM_JOINTS

    def send_joint_command(self, positions):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.publisher.publish(msg)
        self.last_positions = positions

class SliderWidget(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Robot Joint Sliders')
        self.sliders = []
        self.labels = []
        layout = QVBoxLayout()
        for i, name in enumerate(JOINT_NAMES):
            hbox = QHBoxLayout()
            label = QLabel(f'Joint {name}: 0.00')
            self.labels.append(label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)
            slider.setMaximum(314)
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(lambda value, idx=i: self.on_slider_change(idx, value))
            self.sliders.append(slider)
            hbox.addWidget(label)
            hbox.addWidget(slider)
            layout.addLayout(hbox)
        self.setLayout(layout)

    def on_slider_change(self, idx, value):
        # Convert slider value to radians (-3.14 to 3.14)
        pos = value / 100.0
        self.labels[idx].setText(f'Joint {JOINT_NAMES[idx]}: {pos:.2f}')
        positions = [s.value() / 100.0 for s in self.sliders]
        self.ros_node.send_joint_command(positions)

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = JointPublisher()

    # Start ROS spin in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    widget = SliderWidget(node)
    widget.show()
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()