#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self.goals = [
            [0.785,  0.785],
            [0.0,    0.0  ],
            [-0.785, -0.785],
            [0.0,    0.0  ],
        ]
        self.index = 0
        self.timer = self.create_timer(5.0, self.publish_next)

    def publish_next(self):
        if self.index >= len(self.goals):
            self.get_logger().info('All goals sent.')
            self.timer.cancel()
            return

        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2']

        point = JointTrajectoryPoint()
        point.positions = self.goals[self.index]
        point.time_from_start = Duration(sec=2)

        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'Goal {self.index + 1}: {point.positions}')
        self.index += 1


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
