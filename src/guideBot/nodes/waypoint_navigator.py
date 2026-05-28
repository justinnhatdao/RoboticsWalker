import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import threading
import sys
import tty
import termios
import math
import time


WAYPOINTS = {
    '1': ('living room', 0.05, -2.93),
    '2': ('bedroom', -6.23, -0.38),
    '3': ('office', -5.89, -5.12),
}

SPAWN_X = 0.12
SPAWN_Y = -0.04


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.status_pub = self.create_publisher(String, '/guide_status', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.deadman_held = False
        self.navigating = False
        self.current_destination = None
        self._goal_handle = None
        self._last_dist_announced = None
        self._initial_pose_published = False
        self._pose_publish_count = 0

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Try publishing initial pose every 2 seconds for first 10 seconds
        self.create_timer(2.0, self.publish_initial_pose_once)

        print('')
        print('=====================================')
        print('        GUIDE ROBOT - READY')
        print('=====================================')
        print('')
        print('Hold SPACE = dead-man switch')
        print('')
        print('While holding SPACE, press:')
        print('  1 = Living Room')
        print('  2 = Bedroom')
        print('  3 = Office')
        print('')
        print('Release SPACE at any time to stop.')
        print('Press Q to quit.')
        print('')
        print('=====================================')
        print('')

        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

    def publish_initial_pose_once(self):
        if self._pose_publish_count >= 5:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = SPAWN_X
        msg.pose.pose.position.y = SPAWN_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        self.initial_pose_pub.publish(msg)
        self._pose_publish_count += 1
        self.get_logger().info(f'Initial pose published ({self._pose_publish_count}/5)')

    def announce(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        print(f'\n  >> {message}\n')

    def cancel_navigation(self):
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.navigating = False

    def cancel_done_callback(self, future):
        self.navigating = False
        self._goal_handle = None
        self.current_destination = None
        self._last_dist_announced = None
        print('  Navigation cancelled.')

    def navigate_to(self, key):
        if key not in WAYPOINTS:
            return

        if self.navigating:
            print('  Already navigating — release SPACE to cancel first.')
            return

        room_name, x, y = WAYPOINTS[key]

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            print('  Nav2 not ready yet. Try again in a moment.')
            return

        self.navigating = True
        self.current_destination = room_name
        self._last_dist_announced = None

        self.announce(f'Heading to {room_name}...')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if self._last_dist_announced is None or abs(dist - self._last_dist_announced) > 1.0:
            print(f'  {self.current_destination}: {dist:.1f}m remaining')
            self._last_dist_announced = dist
            msg = String()
            msg.data = f'Navigating to {self.current_destination} — {dist:.1f}m remaining'
            self.status_pub.publish(msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('  Goal rejected by Nav2.')
            self.navigating = False
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.navigating = False
        self._goal_handle = None
        self._last_dist_announced = None
        destination = self.current_destination
        self.current_destination = None

        if destination:
            self.announce(f'Arrived at {destination}')

        print('  Ready for next destination.')
        print('  Hold SPACE + press 1, 2, or 3 to navigate.')
        print('')

    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)

                if ch == ' ':
                    self.deadman_held = True

                elif ch in ('1', '2', '3'):
                    if self.deadman_held:
                        self.navigate_to(ch)
                    else:
                        print('  Hold SPACE first, then press a number.')

                elif ch == 'q' or ch == '\x03':
                    print('  Quitting...')
                    break

                else:
                    if self.deadman_held:
                        self.deadman_held = False
                        if self.navigating:
                            self.announce(
                                'Dead-man switch released — stopping navigation.')
                            self.cancel_navigation()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print('  Keyboard stopped. Window will stay open.')
            while rclpy.ok():
                time.sleep(1)


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