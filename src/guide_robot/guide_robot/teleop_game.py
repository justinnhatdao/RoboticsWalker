import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

LINEAR_SPEED = 2.0
ANGULAR_SPEED = 3.5


class GameTeleop(Node):
    def __init__(self):
        super().__init__('game_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.held_keys = set()
        self.running = True
        self.timer = self.create_timer(0.05, self.publish_vel)

        print("\n=== Teleop ===")
        print("W = Forward")
        print("S = Backward")
        print("A = Turn Left in place")
        print("D = Turn Right in place")
        print("Space = Stop  |  Q = Quit")
        print("==============\n")

    def publish_vel(self):
        linear = 0.0
        angular = 0.0

        if 'w' in self.held_keys:
            linear = LINEAR_SPEED
        elif 's' in self.held_keys:
            linear = -LINEAR_SPEED
        elif 'a' in self.held_keys:
            angular = ANGULAR_SPEED
        elif 'd' in self.held_keys:
            angular = -ANGULAR_SPEED

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def on_press(self, key):
        try:
            ch = key.char.lower()
            if ch == 'q':
                self.running = False
                return False
            self.held_keys.add(ch)
        except AttributeError:
            if key == keyboard.Key.space:
                self.held_keys.clear()

    def on_release(self, key):
        try:
            self.held_keys.discard(key.char.lower())
        except AttributeError:
            pass


def main():
    rclpy.init()
    node = GameTeleop()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    with keyboard.Listener(on_press=node.on_press, on_release=node.on_release) as listener:
        listener.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
