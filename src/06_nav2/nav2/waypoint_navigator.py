import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray


# ── Waypoints: (x, y, yaw_degrees) in the map frame ─────────────────────────
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
        self._client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._markers = self.create_publisher(MarkerArray, '/waypoints', 10)

        self._waypoints = WAYPOINTS
        self._current   = 0

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()
        self.get_logger().info(
            f'Starting — {len(self._waypoints)} waypoints queued.')

        self._publish_markers()
        self._send_next()

    # ── Navigation ────────────────────────────────────────────────────────────

    def _send_next(self):
        x, y, yaw = self._waypoints[self._current]
        self.get_logger().info(
            f'[{self._current + 1}/{len(self._waypoints)}] '
            f'Navigating to ({x:.2f}, {y:.2f}, {yaw:.0f}°)')

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y, yaw, self.get_clock())
        self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        ).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        self.get_logger().info(
            f'[WP {self._current + 1}] '
            f'Distance remaining: {msg.feedback.distance_remaining:.2f} m',
            throttle_duration_sec=1.0)

    def _on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Waypoint {self._current + 1} reached!')
            self._current += 1
            self._publish_markers()
            if self._current < len(self._waypoints):
                self._send_next()
            else:
                self.get_logger().info('All waypoints completed!')
        else:
            self.get_logger().warn(
                f'Navigation failed at waypoint {self._current + 1} '
                f'(status {status})')

    # ── Marker visualisation ─────────────────────────────────────────────────

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (x, y, yaw_deg) in enumerate(self._waypoints):
            # Colour: grey = done, green = current target, orange = upcoming
            if i < self._current:
                r, g, b = 0.5, 0.5, 0.5
            elif i == self._current:
                r, g, b = 0.0, 0.9, 0.0
            else:
                r, g, b = 1.0, 0.6, 0.0

            # Sphere at waypoint position
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns     = 'waypoints'
            sphere.id     = i
            sphere.type   = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.15
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 1.0
            sphere.lifetime = Duration(sec=0)
            ma.markers.append(sphere)

            # Arrow showing goal heading
            yaw = math.radians(yaw_deg)
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns     = 'waypoint_arrows'
            arrow.id     = i
            arrow.type   = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.15
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.35   # shaft length
            arrow.scale.y = 0.06   # shaft diameter
            arrow.scale.z = 0.08   # head diameter
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 1.0
            arrow.lifetime = Duration(sec=0)
            ma.markers.append(arrow)

            # Number label
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns     = 'waypoint_labels'
            label.id     = i
            label.type   = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.45
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(i + 1)
            label.lifetime = Duration(sec=0)
            ma.markers.append(label)

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
