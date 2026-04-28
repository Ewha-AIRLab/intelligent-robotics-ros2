import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

WAYPOINTS = [
    ( 1.0,  0.0,   0.0),
    ( 1.0,  1.5,  90.0),
    ( 0.0,  1.5, 180.0),
    ( 0.0,  0.0, 270.0),
]


def _make_pose(x, y, yaw_deg, clock):
    yaw = math.radians(yaw_deg)
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = clock.now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._markers = self.create_publisher(MarkerArray, '/waypoints', 10)
        self._current = 0

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info(f'Starting — {len(WAYPOINTS)} waypoints queued.')
        self._publish_markers()
        self._send_next()

    def _send_next(self):
        x, y, yaw = WAYPOINTS[self._current]
        self.get_logger().info(
            f'[{self._current + 1}/{len(WAYPOINTS)}] Navigating to ({x:.2f}, {y:.2f}, {yaw:.0f}°)')
        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y, yaw, self.get_clock())
        self._client.send_goal_async(goal).add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(self._on_result)
            if f.result().accepted else self.get_logger().warn('Goal rejected'))

    def _on_result(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self._current + 1} reached!')
            self._current += 1
            self._publish_markers()
            if self._current < len(WAYPOINTS):
                self._send_next()
            else:
                self.get_logger().info('All waypoints completed!')
        else:
            self.get_logger().warn(f'Navigation failed at waypoint {self._current + 1}')

    def _make_marker(self, ns, mtype, x, y, z, r, g, b, now, mid):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns, m.id = ns, mid
        m.type, m.action = mtype, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.lifetime = Duration(sec=0)
        return m

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        for i, (x, y, yaw_deg) in enumerate(WAYPOINTS):
            r, g, b = (0.5, 0.5, 0.5) if i < self._current else \
                      (0.0, 0.9, 0.0) if i == self._current else \
                      (1.0, 0.6, 0.0)
            yaw = math.radians(yaw_deg)

            sphere = self._make_marker('waypoints', Marker.SPHERE, x, y, 0.15, r, g, b, now, i)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25

            arrow = self._make_marker('waypoint_arrows', Marker.ARROW, x, y, 0.15, r, g, b, now, i)
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.35, 0.06, 0.08

            label = self._make_marker('waypoint_labels', Marker.TEXT_VIEW_FACING, x, y, 0.45, 1.0, 1.0, 1.0, now, i)
            label.scale.z = 0.25
            label.text = str(i + 1)

            ma.markers.extend([sphere, arrow, label])
        self._markers.publish(ma)


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
