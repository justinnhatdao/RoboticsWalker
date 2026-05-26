import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from guide_robot_interfaces.msg import RobotMode
import random

FORWARD_SPEED = 0.18
TURN_SPEED = 0.35
BACKUP_SPEED = -0.12
OBSTACLE_DISTANCE = 0.8
SIDE_DISTANCE = 0.5

STATE_FORWARD = 'forward'
STATE_TURNING = 'turning'
STATE_BACKUP  = 'backup'


class Wanderer(Node):
    def __init__(self):
        super().__init__('wanderer')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(RobotMode, 'robot_mode', self.override_callback, 10)
        self.paused = False

        self.state = STATE_FORWARD
        self.turn_direction = 1.0
        self.state_ticks = 0

        self.front_dist = 999.0
        self.left_dist  = 999.0
        self.right_dist = 999.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Wanderer started (mapping mode - slow)')

    def override_callback(self, msg):
        self.paused = msg.manual_active

    def scan_callback(self, msg):
        r = msg.ranges
        n = len(r)
        mn, mx = msg.range_min, msg.range_max

        front_vals = [r[i] for i in list(range(0, int(n * 30 / 360))) +
                      list(range(int(n * 330 / 360), n))
                      if mn < r[i] < mx]
        self.front_dist = min(front_vals) if front_vals else 999.0

        left_vals = [r[i] for i in range(int(n * 30 / 360), int(n * 120 / 360))
                     if mn < r[i] < mx]
        self.left_dist = min(left_vals) if left_vals else 999.0

        right_vals = [r[i] for i in range(int(n * 240 / 360), int(n * 330 / 360))
                      if mn < r[i] < mx]
        self.right_dist = min(right_vals) if right_vals else 999.0

    def control_loop(self):
        if self.paused:
            return
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
                self.state_ticks = random.randint(15, 25)
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
                self.state_ticks = 10
            elif front_blocked:
                self.state = STATE_TURNING
                self.state_ticks = random.randint(15, 25)
                self.turn_direction = 1.0 if self.left_dist >= self.right_dist else -1.0
            else:
                openness_diff = self.left_dist - self.right_dist
                drift = 0.15 * (openness_diff / 5.0)
                drift = max(-0.25, min(0.25, drift))
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