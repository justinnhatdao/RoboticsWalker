import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

FORWARD_SPEED = 1.5
TURN_SPEED = 2.5
BACKUP_SPEED = -0.8
OBSTACLE_DISTANCE = 0.6   # start turning when this close ahead
SIDE_DISTANCE = 0.4       # minimum clearance on sides

STATE_FORWARD  = 'forward'
STATE_TURNING  = 'turning'
STATE_BACKUP   = 'backup'


class Wanderer(Node):
    def __init__(self):
        super().__init__('wanderer')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.state = STATE_FORWARD
        self.turn_direction = 1.0
        self.state_ticks = 0

        self.front_dist = 999.0
        self.left_dist  = 999.0
        self.right_dist = 999.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Wanderer started')

    def sector_min(self, ranges, range_min, range_max, start_deg, end_deg, total):
        indices = range(int(start_deg * total / 360), int(end_deg * total / 360))
        vals = [ranges[i] for i in indices if range_min < ranges[i] < range_max]
        return min(vals) if vals else 999.0

    def scan_callback(self, msg):
        r = msg.ranges
        n = len(r)
        mn, mx = msg.range_min, msg.range_max

        # Front: -30 to +30 degrees (wraps around index 0)
        front_vals = [r[i] for i in list(range(0, int(n * 30 / 360))) +
                      list(range(int(n * 330 / 360), n))
                      if mn < r[i] < mx]
        self.front_dist = min(front_vals) if front_vals else 999.0

        # Left: 30–120 degrees
        left_vals = [r[i] for i in range(int(n * 30 / 360), int(n * 120 / 360))
                     if mn < r[i] < mx]
        self.left_dist = min(left_vals) if left_vals else 999.0

        # Right: 240–330 degrees
        right_vals = [r[i] for i in range(int(n * 240 / 360), int(n * 330 / 360))
                      if mn < r[i] < mx]
        self.right_dist = min(right_vals) if right_vals else 999.0

    def control_loop(self):
        msg = Twist()
        front_blocked = self.front_dist < OBSTACLE_DISTANCE
        left_blocked  = self.left_dist  < SIDE_DISTANCE
        right_blocked = self.right_dist < SIDE_DISTANCE
        cornered = front_blocked and left_blocked and right_blocked

        if self.state == STATE_BACKUP:
            msg.linear.x = BACKUP_SPEED
            msg.angular.z = 0.0
            self.state_ticks -= 1
            if self.state_ticks <= 0:
                self.state = STATE_TURNING
                self.state_ticks = random.randint(12, 20)
                # turn toward whichever side has more space
                self.turn_direction = 1.0 if self.left_dist >= self.right_dist else -1.0

        elif self.state == STATE_TURNING:
            msg.linear.x = 0.0
            msg.angular.z = TURN_SPEED * self.turn_direction
            self.state_ticks -= 1
            if self.state_ticks <= 0 and not front_blocked:
                self.state = STATE_FORWARD

        else:  # FORWARD
            if cornered:
                self.state = STATE_BACKUP
                self.state_ticks = 8
            elif front_blocked:
                self.state = STATE_TURNING
                self.state_ticks = random.randint(10, 18)
                self.turn_direction = 1.0 if self.left_dist >= self.right_dist else -1.0
            else:
                # slight random drift so it doesn't retrace the exact same path
                drift = random.uniform(-0.3, 0.3)
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
