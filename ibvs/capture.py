import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 1)
        self.bridge = CvBridge()
        self.saved = False

    def callback(self, msg):
        if not self.saved:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            fname = 'S.png'
            import cv2
            cv2.imwrite(fname, img)
            self.get_logger().info(f"Saved reference image as {fname}")
            self.saved = True

def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()