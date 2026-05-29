import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from guide_robot_interfaces.msg import RobotMode
import random

# Speed constants tuned to keep the robot slow enough for safe mapping
# without covering the same ground too quickly
FORWARD_SPEED = 0.18
TURN_SPEED = 0.35
BACKUP_SPEED = -0.12  # Negative because we're reversing

# Distance thresholds I chose by testing in the sim,
# front needs more clearance since we're moving toward it
OBSTACLE_DISTANCE = 0.8
SIDE_DISTANCE = 0.5

# Three states the robot cycles through to avoid obstacles
STATE_FORWARD = 'forward'
STATE_TURNING = 'turning'
STATE_BACKUP  = 'backup'


class Wanderer(Node):
    def __init__(self):
        super().__init__('wanderer')

        # Publishes velocity commands to drive the robot autonomously
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Reads the LiDAR scan to detect obstacles in all directions
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Listens for mode changes from the teleop node so I can pause autonomous
        # driving whenever an operator takes manual control
        self.create_subscription(RobotMode, 'robot_mode', self.override_callback, 10)

        self.paused = False        # Set to True when the teleop node is in control

        self.state = STATE_FORWARD
        self.turn_direction = 1.0  # +1 = left, -1 = right
        self.state_ticks = 0       # Countdown used to hold a state for a fixed number of loops

        # Default to large values so the robot doesn't react before the first scan arrives
        self.front_dist = 999.0
        self.left_dist  = 999.0
        self.right_dist = 999.0

        # Run the control loop at 10 Hz, fast enough to react to obstacles in time
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Wanderer started (mapping mode - slow)')

    def override_callback(self, msg):
        # Pause or resume based on whatever the teleop node published to robot_mode
        self.paused = msg.manual_active

    def scan_callback(self, msg):
        r = msg.ranges
        n = len(r)
        mn, mx = msg.range_min, msg.range_max

        # Front cone: indices covering +/-30 degrees around 0 degrees (0-30 and 330-360)
        # Filter out inf/nan by only keeping values within the sensor's valid range
        front_vals = [r[i] for i in list(range(0, int(n * 30 / 360))) +
                      list(range(int(n * 330 / 360), n))
                      if mn < r[i] < mx]
        self.front_dist = min(front_vals) if front_vals else 999.0

        # Left cone: 30-120 degrees, used to decide which way to turn when blocked
        left_vals = [r[i] for i in range(int(n * 30 / 360), int(n * 120 / 360))
                     if mn < r[i] < mx]
        self.left_dist = min(left_vals) if left_vals else 999.0

        # Right cone: 240-330 degrees, mirrored from the left
        right_vals = [r[i] for i in range(int(n * 240 / 360), int(n * 330 / 360))
                      if mn < r[i] < mx]
        self.right_dist = min(right_vals) if right_vals else 999.0

    def control_loop(self):
        # Do nothing while the operator has manual control, teleop publishes cmd_vel instead
        if self.paused:
            return
        msg = Twist()
        front_blocked = self.front_dist < OBSTACLE_DISTANCE
        left_blocked  = self.left_dist  < SIDE_DISTANCE
        right_blocked = self.right_dist < SIDE_DISTANCE
        # All three directions blocked means the robot is cornered, need to back out first
        cornered = front_blocked and left_blocked and right_blocked

        if self.state == STATE_BACKUP:
            # Reverse straight for a fixed number of ticks, then transition to turning
            msg.linear.x = BACKUP_SPEED
            msg.angular.z = 0.0
            self.state_ticks -= 1
            if self.state_ticks <= 0:
                self.state = STATE_TURNING
                self.state_ticks = random.randint(15, 25)
                # Turn toward whichever side has more space
                self.turn_direction = 1.0 if self.left_dist >= self.right_dist else -1.0

        elif self.state == STATE_TURNING:
            msg.linear.x = 0.0
            msg.angular.z = TURN_SPEED * self.turn_direction
            self.state_ticks -= 1
            # Don't go back to forward until the front is clear AND the timer has expired
            if self.state_ticks <= 0 and not front_blocked:
                self.state = STATE_FORWARD

        else:  # STATE_FORWARD
            if cornered:
                # Can't turn, must back up first
                self.state = STATE_BACKUP
                self.state_ticks = 10
            elif front_blocked:
                self.state = STATE_TURNING
                self.state_ticks = random.randint(15, 25)
                self.turn_direction = 1.0 if self.left_dist >= self.right_dist else -1.0
            else:
                # Proportional drift toward the more open side keeps the robot away from walls
                # without making it turn aggressively when the path is clear
                openness_diff = self.left_dist - self.right_dist
                drift = 0.15 * (openness_diff / 5.0)
                drift = max(-0.25, min(0.25, drift))  # Clamp so drift never dominates
                msg.linear.x = FORWARD_SPEED
                msg.angular.z = drift

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Wanderer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
