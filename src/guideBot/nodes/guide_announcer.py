import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GuideAnnouncer(Node):

    def __init__(self):
        super().__init__('guide_announcer')

        # Subscribe to the status topic published by waypoint_navigator
        self.create_subscription(String, '/guide_status', self.status_callback, 10)

        print('')
        print('=====================================')
        print('      GUIDE ANNOUNCER - READY')
        print('=====================================')
        print('Listening on /guide_status...')
        print('')

    def status_callback(self, msg):
        print(f'\n  >> {msg.data}\n')


def main():
    rclpy.init()
    node = GuideAnnouncer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
