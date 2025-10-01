import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ikpy.chain import Chain
from ikpy.utils import plot_chain

# Path to your URDF file
URDF_PATH = '/absolute/path/to/so101.urdf'  # Update this path!

# Target position (meters)
TARGET_X = 0.4
TARGET_Y = 0.0
TARGET_Z = 0.3

JOINT_NAMES = ['1', '2', '3', '4', '5', '6']  # Update as needed

class IKPublisher(Node):
    def __init__(self, joint_angles):
        super().__init__('ik_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.publish_trajectory(joint_angles)

    def publish_trajectory(self, joint_angles):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: {joint_angles}")

def main():
    # Load kinematic chain
    robot_chain = Chain.from_urdf_file(URDF_PATH, base_elements=['base'])

    # Desired pose
    target = [TARGET_X, TARGET_Y, TARGET_Z]

    # The orientation vector in ikpy is for the tool, can be left as default
    target_frame = np.eye(4)
    target_frame[:3, 3] = target

    # Compute IK
    joint_angles = robot_chain.inverse_kinematics(target_frame)
    # Remove the first element (corresponds to the fixed base)
    joint_angles = joint_angles[1:len(JOINT_NAMES)+1]

    print("IK Solution:", joint_angles)

    # Publish to ROS2
    rclpy.init()
    node = IKPublisher(joint_angles)
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()