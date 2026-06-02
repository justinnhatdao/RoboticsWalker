import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import threading
import time
import sys
import tty
import termios
from pynput import keyboard


# Named waypoints I defined by reading the coordinates from RViz after the map was built
# Format: key -> (room label, x, y) in the map frame
WAYPOINTS = {
    '1': ('living room', 0.05, -2.93),
    '2': ('bedroom', -6.23, -0.38),
    '3': ('office', -5.89, -5.12),
}

# The robot's spawn position in the world, used to set the initial pose estimate
# so Nav2's AMCL localizer knows where to start on the map
SPAWN_X = 0.12
SPAWN_Y = -0.04


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Publishes status strings so other nodes or a UI can track what the robot is doing
        self.status_pub = self.create_publisher(String, '/guide_status', 10)

        # Action client that sends navigation goals to Nav2's navigate_to_pose server
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.deadman_held = False          # True only while SPACE is held down
        self.navigating = False            # Prevents sending a new goal mid-navigation
        self.current_destination = None    # Room name of the active goal
        self._goal_handle = None           # Kept so I can cancel the goal if needed
        self._last_dist_announced = None   # Throttles distance print-outs to every 1 m
        self._current_key = None           # Waypoint key of the active goal
        self._paused_key = None            # Waypoint key saved when deadman is released mid-nav
        self._initial_pose_published = False
        self._pose_publish_count = 0

        # Publisher for the AMCL initial pose, tells Nav2 where the robot starts on the map
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Retry publishing the initial pose every 2 s for the first 10 s (5 attempts)
        # because Nav2/AMCL might not be ready the moment this node starts
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
        msg.header.stamp = Time()  # zero stamp lets AMCL use the current sim time
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
        # Publish to /guide_status — the guide_announcer node subscribes and prints it
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def cancel_navigation(self):
        # Cancel the current Nav2 goal asynchronously; result handled in the callback
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.navigating = False

    def cancel_done_callback(self, future):
        self.navigating = False
        self._goal_handle = None
        self.current_destination = None
        self._current_key = None
        self._last_dist_announced = None
        # _paused_key is intentionally NOT cleared here.
        # on_press needs it to resume if Space was re-pressed before cancel finished.
        if self.deadman_held and self._paused_key:
            key_to_resume = self._paused_key
            self._paused_key = None  # clear before navigating to avoid a double-send
            self.announce(f'Resuming navigation to {WAYPOINTS[key_to_resume][0]}...')
            self.navigate_to(key_to_resume)

    def navigate_to(self, key):
        if key not in WAYPOINTS:
            return

        # Block a second goal from being sent while one is already in progress
        if self.navigating:
            print('  Already navigating — release SPACE to cancel first.')
            return

        room_name, x, y = WAYPOINTS[key]

        # Wait up to 3 seconds for the Nav2 action server before giving up
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            print('  Nav2 not ready yet. Try again in a moment.')
            return

        self.navigating = True
        self.current_destination = room_name
        self._current_key = key
        self._paused_key = None
        self._last_dist_announced = None

        self.announce(f'Heading to {room_name}...')

        # Build the NavigateToPose goal with the target coordinates in the map frame
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0  # Don't care about final heading

        # Send the goal asynchronously and register callbacks for the response and feedback
        send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        # Only print/publish when the remaining distance changes by more than 1 m
        # to avoid spamming the terminal with continuous updates
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
        # Store the handle so I can cancel mid-navigation if the dead-man switch is released
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.navigating = False
        self._goal_handle = None
        self._last_dist_announced = None
        self._current_key = None
        # by a deadman release and we still need it to resume on next Space press
        destination = self.current_destination
        self.current_destination = None

        if destination:
            self.announce(f'Arrived at {destination}')

        print('  Ready for next destination.')
        print('  Hold SPACE + press 1, 2, or 3 to navigate.')
        print('')

    def keyboard_loop(self):
        def on_press(key):
            if key == keyboard.Key.space:
                # Guard against OS key-repeat: only act on the first press,
                # not the flood of repeats the OS sends while the key is held.
                if not self.deadman_held:
                    self.deadman_held = True
                    # Only resume here if cancel has already finished (navigating=False).
                    # If cancel is still in flight, cancel_done_callback will see
                    # deadman_held=True and trigger the resume automatically.
                    if self._paused_key and not self.navigating:
                        key_to_resume = self._paused_key
                        self._paused_key = None  # clear before navigating to avoid double-send
                        print(f'  Resuming navigation to {WAYPOINTS[key_to_resume][0]}...')
                        self.announce(f'Resuming navigation to {WAYPOINTS[key_to_resume][0]}...')
                        self.navigate_to(key_to_resume)
            else:
                try:
                    ch = key.char
                except AttributeError:
                    return
                if ch in ('1', '2', '3'):
                    if self.deadman_held:
                        self._paused_key = None
                        self.navigate_to(ch)
                    else:
                        print('  Hold SPACE first, then press a number.')
                elif ch == 'q':
                    print('  Quitting...')
                    return False  # stops the listener

        def on_release(key):
            if key == keyboard.Key.space:
                self.deadman_held = False
                if self.navigating:
                    self._paused_key = self._current_key
                    destination = self.current_destination
                    print(f'\n  Deadman released — navigation stopped.')
                    print(f'  Hold SPACE to resume to {destination}.\n')
                    self.announce(
                        f'You have let go of the deadman switch. '
                        f'Hold SPACE to resume navigation to {destination}.'
                    )
                    self.cancel_navigation()
                elif self._paused_key:
                    print(f'\n  Hold SPACE to resume to {WAYPOINTS[self._paused_key][0]}.\n')
            elif key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
                return False  # Ctrl+C stops the listener

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

        print('  Keyboard stopped. Window will stay open.')
        while rclpy.ok():
            time.sleep(1)


def main():
    # Put the terminal in cbreak mode so Space and other keys are intercepted cleanly
    # by pynput without being echoed to stdout as characters.
    old_term_settings = None
    if sys.stdin.isatty():
        fd = sys.stdin.fileno()
        old_term_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

    rclpy.init()
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Restore the terminal to its original settings on exit
        if old_term_settings is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term_settings)


if __name__ == '__main__':
    main()