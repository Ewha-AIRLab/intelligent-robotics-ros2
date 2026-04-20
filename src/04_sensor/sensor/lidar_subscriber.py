import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )
        self._count = 0
        self.get_logger().info('LidarSubscriber started — listening on /scan')

    def scan_callback(self, msg: LaserScan):
        # index = (target_angle - angle_min) / angle_increment
        # front of robot = 0 rad
        front_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        front_range = msg.ranges[front_index]

        # Replace nan/inf with a readable string
        if front_range != front_range or front_range == float('inf'):
            front_str = 'out of range'
        else:
            front_str = f'{front_range:.3f} m'

        self._count += 1
        self.get_logger().info(
            f'[{self._count}] LiDAR — front: {front_str} '
            f'(index {front_index}/{len(msg.ranges) - 1})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
