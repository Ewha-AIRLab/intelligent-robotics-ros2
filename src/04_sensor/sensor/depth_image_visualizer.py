import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthImageVisualizer(Node):
    def __init__(self):
        super().__init__('depth_image_visualizer')

        # Min/max depth range for normalization (meters)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/depth_image',
            self.depth_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Image,
            '/depth_camera/depth_image_visual',
            10,
        )
        self.get_logger().info('DepthImageVisualizer started')

    def depth_callback(self, msg: Image):
        min_depth = self.get_parameter('min_depth').value
        max_depth = self.get_parameter('max_depth').value

        # Convert 32FC1 depth image to numpy float array
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Replace nan/inf with max_depth
        depth = np.nan_to_num(depth, nan=max_depth, posinf=max_depth, neginf=min_depth)

        # Normalize to 0-255: close = white, far = black
        depth_clipped = np.clip(depth, min_depth, max_depth)
        depth_normalized = 1.0 - (depth_clipped - min_depth) / (max_depth - min_depth)
        depth_uint8 = (depth_normalized * 255).astype(np.uint8)

        # Apply colormap for better depth perception (optional: cv2.COLORMAP_BONE for B&W)
        depth_visual = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_BONE)

        out_msg = self.bridge.cv2_to_imgmsg(depth_visual, encoding='bgr8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
