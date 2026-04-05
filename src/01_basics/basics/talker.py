import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TalkerNode(Node):

    def __init__(self):
        # Register this node with the name 'talker'
        super().__init__('talker')

        # Create a publisher on topic 'topic' with message type String
        # Queue size 10: buffer up to 10 messages if subscriber is slow
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Fire timer_callback every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track how many messages have been published
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)

        # get_logger() is preferred over print() — includes timestamp and node name
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = TalkerNode()

    # spin() blocks here and processes callbacks until the node is shut down
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()