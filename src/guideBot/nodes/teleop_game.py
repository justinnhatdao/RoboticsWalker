import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from guide_robot_interfaces.msg import RobotMode
from pynput import keyboard

# Speed values tuned so the waffle moves at a safe, controllable pace in the sim
LINEAR_SPEED = 0.18
ANGULAR_SPEED = 0.35

# Only these keys trigger movement — anything else is ignored or handled separately
DRIVE_KEYS = {'w', 's', 'a', 'd'}


class GameTeleop(Node):
    def __init__(self):
        super().__init__('game_teleop')

        # Publishes velocity commands that Gazebo/the robot actually acts on
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publishes the current mode so other nodes (like the wanderer) know
        # whether a human is in control or the robot should act autonomously
        self.override_pub = self.create_publisher(RobotMode, 'robot_mode', 10)

        self.current_key = None       # Tracks which drive key is currently held
        self.override_active = False  # Whether manual control is currently on
        self.running = True

        # Publishes velocity at 20 Hz so movement stays smooth and responsive
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
        # Only publish a mode change when the state actually changes to avoid flooding the topic
        if self.override_active != active:
            self.override_active = active
            msg = RobotMode()
            msg.manual_active = active
            # Tell other nodes whether an operator or the wanderer is in control
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
        # Do nothing if manual control is off — let the autonomous node drive
        if not self.override_active:
            return
        linear = 0.0
        angular = 0.0
        # Map each key to the correct axis: linear.x for forward/back, angular.z for turning
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
                # Release manual control and hand driving back to the autonomous node
                self.current_key = None
                self.set_override(False)
            elif ch in DRIVE_KEYS:
                self.current_key = ch
                self.set_override(True)  # First WASD press activates manual override
        except AttributeError:
            # Special keys (like space) don't have a .char — handle them here
            if key == keyboard.Key.space:
                self.current_key = None  # Zero out velocity but keep override active
                if self.override_active:
                    print("\n  [Stopped]\n")


def main():
    rclpy.init()
    node = GameTeleop()

    # Spin the node on a background thread so the ROS callbacks keep running
    # while the main thread is blocked waiting for keyboard input
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Block here listening for keypresses; returns when the listener is stopped (Q pressed)
    with keyboard.Listener(on_press=node.on_press) as listener:
        listener.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
