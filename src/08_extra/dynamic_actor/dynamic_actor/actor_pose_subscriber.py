#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ActorPoseSubscriber(Node):
    def __init__(self):
        super().__init__('actor_pose_subscriber')
        self.declare_parameter('actor_name', '')
        actor_name = self.get_parameter('actor_name').get_parameter_value().string_value

        self._subs = []

        if actor_name:
            # Single actor mode: subscribe to the named actor only
            self._subscribe(actor_name)
        else:
            # Multi-actor mode: discover all /actor/*/pose topics at startup,
            # then periodically check for new ones
            self._discover()
            self.create_timer(2.0, self._discover)

    def _subscribe(self, actor_name: str):
        topic = f'/actor/{actor_name}/pose'
        sub = self.create_subscription(PoseStamped, topic, self._make_cb(actor_name), 10)
        self._subs.append(sub)
        self.get_logger().info(f'Subscribing to {topic}')

    def _discover(self):
        subscribed = {s.topic_name for s in self._subs}
        for topic, types in self.get_topic_names_and_types():
            parts = topic.split('/')
            if (len(parts) == 4 and parts[1] == 'actor' and parts[3] == 'pose'
                    and 'geometry_msgs/msg/PoseStamped' in types
                    and topic not in subscribed):
                self._subscribe(parts[2])

    def _make_cb(self, actor_name: str):
        def _cb(msg: PoseStamped):
            p = msg.pose.position
            self.get_logger().info(
                f'[{actor_name}] pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})'
            )
        return _cb


def main():
    rclpy.init()
    rclpy.spin(ActorPoseSubscriber())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
