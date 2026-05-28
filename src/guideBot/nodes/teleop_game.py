import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from guide_robot_interfaces.msg import RobotMode
from pynput import keyboard

LINEAR_SPEED = 0.18
ANGULAR_SPEED = 0.35

DRIVE_KEYS = {'w', 's', 'a', 'd'}


class GameTeleop(Node):
    def __init__(self):
        super().__init__('game_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.override_pub = self.create_publisher(RobotMode, 'robot_mode', 10)
        self.current_key = None
        self.override_active = False
        self.running = True
        self.timer = self.create_timer(0.05, self.publish_vel)

        print("\n")
        print("=" * 40)
        print("         GUIDE ROBOT TELEOP")
        print("=" * 40)
        print("  W = Forward")
        print("  S = Backward")
        print("  A = Turn Left")
        print("  D = Turn Right")
        print("  Space = Stop movement")
        print("  R = Resume auto-mapping")
        print("  Q = Quit")
        print("=" * 40)
        print("\n  Auto-mapping active.")
        print("  Press WASD to take manual control.\n")

    def set_override(self, active):
        if self.override_active != active:
            self.override_active = active
            msg = RobotMode()
            msg.manual_active = active
            msg.source = 'operator' if active else 'wanderer'
            self.override_pub.publish(msg)
            if active:
                print("\n" + "=" * 40)
                print("  MANUAL OVERRIDE ON")
                print("  Press R to resume auto-mapping")
                print("=" * 40 + "\n")
            else:
                print("\n" + "=" * 40)
                print("  AUTO-MAPPING RESUMED")
                print("  Press WASD to take control again")
                print("=" * 40 + "\n")

    def publish_vel(self):
        if not self.override_active:
            return
        linear = 0.0
        angular = 0.0
        if self.current_key == 'w':
            linear = LINEAR_SPEED
        elif self.current_key == 's':
            linear = -LINEAR_SPEED
        elif self.current_key == 'a':
            angular = ANGULAR_SPEED
        elif self.current_key == 'd':
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
            elif ch == 'r':
                self.current_key = None
                self.set_override(False)
            elif ch in DRIVE_KEYS:
                self.current_key = ch
                self.set_override(True)
        except AttributeError:
            if key == keyboard.Key.space:
                self.current_key = None
                if self.override_active:
                    print("\n  [Stopped]\n")


def main():
    rclpy.init()
    node = GameTeleop()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    with keyboard.Listener(on_press=node.on_press) as listener:
        listener.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()