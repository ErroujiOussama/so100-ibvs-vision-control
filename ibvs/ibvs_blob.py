import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
from cv_bridge import CvBridge
import numpy as np

from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
JOINT_NAMES = ['1', '2', '3', '4', '5']
NUM_JOINTS = len(JOINT_NAMES)
REFERENCE_IMAGE_PATH = 'S.png'
FOCAL_LENGTH_PIXELS = 800.0  # adjust based on your camera

# gain
LAMBDA = 0.001  # tuning parameter

class IBVSController(Node):
    def __init__(self):
        super().__init__('ibvs_controller')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)

        self.current_joints = np.zeros(NUM_JOINTS)

        # Reference feature
        ref_img = cv2.imread(REFERENCE_IMAGE_PATH)
        if ref_img is None:
            self.get_logger().error(f"Could not load {REFERENCE_IMAGE_PATH}")
            exit(1)
        self.s_star = self.extract_feature(ref_img)
        if self.s_star is None:
            self.get_logger().error("Reference feature not found")
            exit(1)

        self.get_logger().info(f"Reference feature: {self.s_star}")

    def extract_feature(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy], dtype=np.float32)

    def compute_interaction_matrix(self, s):
        x, y = s
        Z = 1.0  # estimated depth (in meters) — you must refine or estimate this dynamically
        fx = fy = FOCAL_LENGTH_PIXELS

        # Interaction matrix L for 2D point (x, y)
        L = np.array([
            [-fx/Z,      0,   x/Z,  x*y/fx, -(fx**2 + x**2)/fx,    y],
            [     0, -fy/Z,   y/Z,  (fy**2 + y**2)/fy, -x*y/fy,   -x]
        ])
        return L

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        s = self.extract_feature(image)
        if s is None:
            self.get_logger().info("No feature found in current image")
            return

        error = self.s_star - s
        error_norm = np.linalg.norm(error)
        self.get_logger().info(f"Pixel error: {error.tolist()} (norm: {error_norm:.2f})")

        if error_norm < 5.0:
            self.get_logger().info("Target reached.")
            return

        # Compute interaction matrix
        L = self.compute_interaction_matrix(s)

        try:
            L_pinv = np.linalg.pinv(L)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular L matrix.")
            return

        # Control law: camera velocity
        v = -LAMBDA * error
        v_camera = L_pinv @ v

        # Simplified mapping: pretend v_camera directly affects joint positions
        # Replace this with FK/IK mapping for real arm
        delta_q = np.zeros(NUM_JOINTS)
        delta_q[0] = v_camera[0]  # base
        delta_q[1] = v_camera[1]  # shoulder

        new_joints = self.current_joints + delta_q
        self.send_joint_command(new_joints)
        self.current_joints = new_joints

    def send_joint_command(self, positions):
        msg = JointTrajectory()

        # MATCHING the ros2 CLI example exactly
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = ''

        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"✅ Sent joint command: {positions}")


def main():
    rclpy.init()
    node = IBVSController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
