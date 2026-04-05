import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ListenerNode(Node):

    def __init__(self):
        # Register this node with the name 'listener'
        super().__init__('listener')

        # Subscribe to topic 'topic' with message type String
        # listener_callback is called every time a new message arrives
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # This callback is invoked by spin() whenever a message is received
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = ListenerNode()

    # spin() blocks here and processes callbacks until the node is shut down
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()