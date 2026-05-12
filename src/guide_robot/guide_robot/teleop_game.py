import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading

class GameTeleop(Node):
    def __init__(self):
        super().__init__('game_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.running = True
        
        self.timer = self.create_timer(0.1, self.publish_vel)
        print("\n=== Game-Style Teleop ===")
        print("W = Forward")
        print("S = Backward")
        print("A = Turn Left")
        print("D = Turn Right")
        print("Space = Stop")
        print("Q = Quit")
        print("========================\n")

    def publish_vel(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)

    def process_key(self, key):
        if key == 'w':
            self.linear = 0.5
            self.angular = 0.0
        elif key == 's':
            self.linear = -0.5
            self.angular = 0.0
        elif key == 'a':
            self.linear = 0.0
            self.angular = 1.0
        elif key == 'd':
            self.linear = 0.0
            self.angular = -1.0
        elif key == ' ':
            self.linear = 0.0
            self.angular = 0.0
        elif key == 'q':
            self.linear = 0.0
            self.angular = 0.0
            self.running = False

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key

def main():
    rclpy.init()
    node = GameTeleop()
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    while node.running:
        key = get_key()
        node.process_key(key.lower())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()