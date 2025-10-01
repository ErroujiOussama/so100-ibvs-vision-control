#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

from models.superpoint import SuperPoint
from models.superglue import SuperGlue

class IBVSSuperGlueNode(Node):
    def __init__(self):
        super().__init__('ibvs_superglue_node')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.match_img_pub = self.create_publisher(Image, '/matches/image', 10)

        self.joint_names = ['1', '2', '3', '4', '5']
        self.current_positions = [0.0] * 5  # Initialize joint positions

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.load_models()
        self.reference = self.load_and_extract_reference('S.png')

    def load_models(self):
        sp_config = {'nms_radius': 4, 'keypoint_threshold': 0.005, 'max_keypoints': 512}
        sg_config = {'sinkhorn_iterations': 20, 'match_threshold': 0.2}
        self.superpoint = SuperPoint(sp_config).eval().to(self.device)
        self.superglue = SuperGlue(sg_config).eval().to(self.device)

    def load_and_extract_reference(self, path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        img_tensor = torch.from_numpy(img / 255.).float()[None, None].to(self.device)
        self.reference_image = img_tensor
        return self.superpoint({'image': img_tensor})

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img_tensor = torch.from_numpy(img / 255.).float()[None, None].to(self.device)
        pred = self.superpoint({'image': img_tensor})

        match_input = {
            'keypoints0': self.reference['keypoints'][0][None],
            'descriptors0': self.reference['descriptors'][0][None],
            'scores0': self.reference['scores'][0][None],
            'keypoints1': pred['keypoints'][0][None],
            'descriptors1': pred['descriptors'][0][None],
            'scores1': pred['scores'][0][None],
            'image0': self.reference_image,
            'image1': img_tensor,
        }

        match_output = self.superglue(match_input)

        kpts0 = match_input['keypoints0'][0].cpu().numpy()
        kpts1 = match_input['keypoints1'][0].cpu().numpy()
        matches = match_output['matches0'][0].cpu().numpy()

        valid = matches > -1
        matched_kpts0 = kpts0[valid]
        matched_kpts1 = kpts1[matches[valid]]

        if len(matched_kpts0) < 4:
            self.get_logger().warn('Not enough matches for IBVS')
            return

        # --- Visualization ---
        ref_img_np = (self.reference_image[0, 0].cpu().numpy() * 255).astype(np.uint8)
        ref_img_color = cv2.cvtColor(ref_img_np, cv2.COLOR_GRAY2BGR)

        height = max(ref_img_color.shape[0], img_color.shape[0])
        width = ref_img_color.shape[1] + img_color.shape[1]
        vis = np.zeros((height, width, 3), dtype=np.uint8)
        vis[:ref_img_color.shape[0], :ref_img_color.shape[1]] = ref_img_color
        vis[:img_color.shape[0], ref_img_color.shape[1]:] = img_color

        for (x0, y0), (x1, y1) in zip(matched_kpts0, matched_kpts1):
            pt0 = (int(x0), int(y0))
            pt1 = (int(x1) + ref_img_color.shape[1], int(y1))
            cv2.circle(vis, pt0, 3, (0, 255, 0), -1)
            cv2.circle(vis, pt1, 3, (0, 255, 0), -1)
            cv2.line(vis, pt0, pt1, (0, 255, 0), 1)

        match_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        self.match_img_pub.publish(match_msg)

        # --- IBVS Control Logic ---
        error = (matched_kpts1 - matched_kpts0).mean(axis=0)  # (dx, dy)

        # Proportional control on x error affecting only joint 1
        delta_joint1 = -0.01 * error[0]  # Tune gain as needed

        # Update only joint 1 position, keep others unchanged
        self.current_positions[0] += delta_joint1

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_positions.copy()
        point.velocities = [0.0] * len(self.current_positions)
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        traj_msg.points.append(point)
        self.joint_pub.publish(traj_msg)

        self.get_logger().info(f'Published joint 1 position: {self.current_positions[0]:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = IBVSSuperGlueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
